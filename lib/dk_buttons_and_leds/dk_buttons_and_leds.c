/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/device.h>
#include <hal/nrf_gpio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <nrfx.h>
#include <hal/nrf_gpio.h>
#include <dk_buttons_and_leds.h>

LOG_MODULE_REGISTER(dk_buttons_and_leds, CONFIG_DK_LIBRARY_LOG_LEVEL);

#define BUTTONS_NODE DT_PATH(buttons)
#define LEDS_NODE DT_PATH(leds)

#define GPIO0_DEV DEVICE_DT_GET(DT_NODELABEL(gpio0))
#define GPIO1_DEV DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio1))

/* GPIO0, GPIO1 and GPIO expander devices require different interrupt flags. */
#define FLAGS_GPIO_0_1_ACTIVE GPIO_INT_LEVEL_ACTIVE
#define FLAGS_GPIO_EXP_ACTIVE (GPIO_INT_EDGE | GPIO_INT_HIGH_1 | GPIO_INT_LOW_0 | GPIO_INT_ENABLE)

#define GPIO_SPEC_AND_COMMA(button_or_led) GPIO_DT_SPEC_GET(button_or_led, gpios),


#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif
#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};


static const struct gpio_dt_spec buttons[] = {
#if DT_NODE_EXISTS(BUTTONS_NODE)
	DT_FOREACH_CHILD(BUTTONS_NODE, GPIO_SPEC_AND_COMMA)
#endif
};

static const struct gpio_dt_spec leds[] = {
#if DT_NODE_EXISTS(LEDS_NODE)
	DT_FOREACH_CHILD(LEDS_NODE, GPIO_SPEC_AND_COMMA)
#endif
};

#if CONFIG_PWM
#define LED_PWM_NODE_ID	 DT_COMPAT_GET_ANY_STATUS_OKAY(pwm_leds)

const char *led_label[] = {
	DT_FOREACH_CHILD_SEP_VARGS(LED_PWM_NODE_ID, DT_PROP_OR, (,), label, NULL)
};
#endif

#ifdef CONFIG_TRUSTED_EXECUTION_NONSECURE
#define TEST_PARTITION	slot1_ns_partition
#else
#define TEST_PARTITION	slot1_partition
#endif

#define TEST_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(TEST_PARTITION)

#define FLASH_PAGE_SIZE   4096

const struct device *flash_dev = TEST_PARTITION_DEVICE;

enum state {
	STATE_WAITING,
	STATE_SCANNING,
};

static enum state state;
static struct k_work_delayable buttons_scan;
static button_handler_t button_handler_cb;
static atomic_t my_buttons;
static struct gpio_callback gpio_cb;
static struct k_spinlock lock;
static sys_slist_t button_handlers;
static struct k_mutex button_handler_mut;
static bool irq_enabled;

static int callback_ctrl(bool enable)
{
	int err = 0;
	gpio_flags_t flags;

	/* This must be done with irqs disabled to avoid pin callback
	 * being fired before others are still not activated.
	 */
	for (size_t i = 0; (i < ARRAY_SIZE(buttons)) && !err; i++) {
		if (enable) {
			flags = ((buttons[i].port == GPIO0_DEV || buttons[i].port == GPIO1_DEV) ?
					 FLAGS_GPIO_0_1_ACTIVE :
					 FLAGS_GPIO_EXP_ACTIVE);
		} else {
			flags = GPIO_INT_DISABLE;
		}

		err = gpio_pin_interrupt_configure_dt(&buttons[i], flags);
		if (err) {
			LOG_ERR("GPIO IRQ config failed, err: %d", err);
			return err;
		}
	}

	return err;
}

static uint32_t get_buttons(void)
{
	uint32_t ret = 0;
	for (size_t i = 0; i < ARRAY_SIZE(buttons); i++) {
		int val;

		val = gpio_pin_get_dt(&buttons[i]);
		if (val < 0) {
			LOG_ERR("Cannot read gpio pin");
			return 0;
		}
		if (val) {
			ret |= 1U << i;
		}
	}

	return ret;
}

static void button_handlers_call(uint32_t button_state, uint32_t has_changed)
{
	struct button_handler *handler;

	if (button_handler_cb != NULL) {
		button_handler_cb(button_state, has_changed);
	}

	if (IS_ENABLED(CONFIG_DK_LIBRARY_DYNAMIC_BUTTON_HANDLERS)) {
		k_mutex_lock(&button_handler_mut, K_FOREVER);
		SYS_SLIST_FOR_EACH_CONTAINER(&button_handlers, handler, node) {
			handler->cb(button_state, has_changed);
		}
		k_mutex_unlock(&button_handler_mut);
	}
}

KEY_REGEDIT key_info = {
	// .get_key_state = (get_key_state_cb_t)gd_eval_key_state_get,
	.key_status = {
		{BUTTON_NONE, 0, 0, 0, 0},
		{BUTTON_NONE, 0, 0, 0, 0},
		{BUTTON_NONE, 0, 0, 0, 0},
		{BUTTON_NONE, 0, 0, 0, 0},
		{BUTTON_NONE, 0, 0, 0, 0},
		{BUTTON_NONE, 0, 0, 0, 0},
		{BUTTON_NONE, 0, 0, 0, 0},
		{BUTTON_NONE, 0, 0, 0, 0},
		{BUTTON_NONE, 0, 0, 0, 0},
		{BUTTON_NONE, 0, 0, 0, 0},
	},
};

KEY_INFO get_key_info_value(int key_index)
{
	return key_info.key_status[key_index].key_value;
}

static KEY_INFO get_key_status(KEY_STATUS *key_status, KEY_INFO key_value, uint32_t key_all)
{
	uint8_t key;
	uint32_t temp32;

	// key = key_info.get_key_state(key_status->key_indx) ? 0 : 1;
	key = key_all & BIT(key_status->key_indx) ? 1 : 0;
	if (key_value.value.status.Bit.press_status != key) {
		// LOG_INF("key=%d", key);
		key_value.value.event.all = 0;
		if (key == KEY_PRESS_IO_STATUS) {
			key_status->press_time = k_uptime_get_32();
			// 释放时间
			temp32 = k_uptime_get_32() - key_status->release_time;
			// LOG_INF("Release event time=%u", temp32);
		} else {
			key_status->release_time = k_uptime_get_32();
			// 按下时间
			temp32 = k_uptime_get_32() - key_status->press_time;
			// LOG_INF("Press event time=%u", temp32);
			if (temp32 < SHORT_KEY_DELAY) {
				// LOG_INF("SHORT_KEY_DELAY");
				key_value.value.event.Bit.multi_press = 1;
				if (key_value.value.status.Bit.mutle_num == 15) {
					key_value.value.status.Bit.mutle_num = 9;
				}
				key_value.value.status.Bit.mutle_num++;
			} else if (temp32 < LONG_KEY_DELAY) {
				// LOG_INF("LONG_KEY_DELAY");
				key_value.value.event.Bit.short_press = 1;
				key_value.value.status.Bit.mutle_num = 0;
				key_status->release_time = k_uptime_get_32() + LONG_KEY_DELAY;
			} else {
				// LOG_INF("LONG_KEY");
				key_value.value.event.Bit.long_press = 1;
				temp32 /= 1000u;
				if (temp32 > 15) {
					key_value.value.status.Bit.long_time = 15;
				} else {
					key_value.value.status.Bit.long_time = temp32;
				}
				key_status->release_time = k_uptime_get_32() + LONG_KEY_DELAY;
			}
		}
		key_value.value.event.Bit.press = 1;
		key_value.value.status.Bit.press_status = key;
	} else {
		if (key == KEY_PRESS_IO_STATUS) {
			// 按下时间
			temp32 = k_uptime_get_32() - key_status->press_time;
			// LOG_INF("Press time=%u", temp32);
			if (temp32 < SHORT_KEY_DELAY) {
				// LOG_INF("SHORT_KEY_DELAY");
			} else if (temp32 < LONG_KEY_DELAY) {
				// LOG_INF("LONG_KEY_DELAY");
				if (key_value.value.status.Bit.mutle_num == 1) {
					// LOG_INF("SHORT_KEY_DELAY");
					// key_value.value.event.all = 0;
					key_value.value.event.Bit.short_press = 1;
					key_value.value.status.Bit.mutle_num = 0;
				}
			} else {
				// LOG_INF("LONG_KEY");
				key_value.value.event.all = 0;
				key_value.value.event.Bit.long_press = 1;
				temp32 /= 1000u;
				if (temp32 > 14) {
					key_value.value.status.Bit.long_time = 10 + (temp32 % 5);
				} else {
					key_value.value.status.Bit.long_time = temp32;
				}
			}
		} else {
			// 释放时间
			temp32 = k_uptime_get_32() - key_status->release_time;
			// LOG_INF("Release time=%u", temp32);
			if (temp32 < SHORT_KEY_DELAY) {
				// LOG_INF("SHORT_KEY_DELAY");
			} else if (temp32 < LONG_KEY_DELAY) {
				// LOG_INF("LONG_KEY_DELAY");
				if (key_value.value.event.Bit.multi_press && key_value.value.status.Bit.mutle_num == 1) {
					// LOG_INF("LONG_KEY_DELAY");
					key_value.value.event.all = 0;
					key_value.value.event.Bit.short_press = 1;
					key_value.value.status.Bit.mutle_num = 0;
				} else {
					key_value.all = 0;
				}
				key_status->release_time = k_uptime_get_32() + LONG_KEY_DELAY;
			} else {
				// LOG_INF("LONG_KEY");
				key_value.all = 0;
				key_status->release_time = k_uptime_get_32() + LONG_KEY_DELAY;
			}
		}
	}

	return key_value;
}

static void buttons_scan_fn(struct k_work *work)
{
	int err;
	static uint32_t last_button_scan;
	static bool initial_run = true;
	uint32_t button_scan;

	if (irq_enabled) {
		/* Disable GPIO interrupts for edge triggered devices.
		 * Devices that are configured with active high interrupts are already disabled.
		 */
		err = callback_ctrl(false);
		if (err) {
			LOG_ERR("Cannot disable callbacks");
			return;
		}

		irq_enabled = false;
	}

	button_scan = get_buttons();
	atomic_set(&my_buttons, (atomic_val_t)button_scan);

	uint32_t button_event = 0;
	uint32_t button_stop = 0;
	KEY_INFO temp_key;

	for (uint8_t i = 0; i < ARRAY_SIZE(buttons); i++) {
		temp_key = get_key_status(&(key_info.key_status[i]), key_info.key_status[i].key_value, button_scan);
		if (temp_key.all != key_info.key_status[i].key_value.all) {
			key_info.key_status[i].key_value = temp_key;
			// LOG_INF("key_status[%d]=%08X", i, temp_key.all);
			if (temp_key.value.event.all) {
				button_event |= 1 << i;
			}
		} else {
#if defined(CONFIG_DISPLAY) && (CONFIG_DISPLAY == 1)
			if (temp_key.value.event.Bit.long_press && oled_status >= OLED_STATUS_ADJUST0) {
				uint32_t temp32 = k_uptime_get_32() - key_info.key_status[i].press_time;
				// LOG_INF("temp32=%d, long_time=%d", temp32, temp_key.value.status.Bit.long_time);
				if (temp_key.value.status.Bit.long_time <= 2) {
					if (temp32 % LONG_KEY_SPEED0 < 50) {
						// LOG_INF("LONG_KEY_SPEED0");
						button_event |= 1 << i;
					}
				} else if (temp_key.value.status.Bit.long_time <= 5) {
					if (temp32 % LONG_KEY_SPEED1 < 50) {
						// LOG_INF("LONG_KEY_SPEED1");
						button_event |= 1 << i;
					}
				} else {
					// LOG_INF("LONG_KEY_SPEED2");
					button_event |= 1 << i;
				}
			}
#endif
		}
		button_stop |= temp_key.all;
	}
	if (button_event) {
		button_handlers_call(button_scan, button_event);
	}

	last_button_scan = button_scan;

	if (button_scan != 0 || button_stop)
	{
		k_work_reschedule(&buttons_scan,
		  K_MSEC(CONFIG_DK_LIBRARY_BUTTON_SCAN_INTERVAL));
	} else {
		/* If no button is pressed module can switch to callbacks */
		int err = 0;

		k_spinlock_key_t key = k_spin_lock(&lock);

		switch (state) {
		case STATE_SCANNING:
			state = STATE_WAITING;
			err = callback_ctrl(true);
			irq_enabled = true;
			break;

		default:
			/* Do nothing */
			break;
		}

		k_spin_unlock(&lock, key);

		if (err) {
			LOG_ERR("Cannot enable callbacks");
		}
	}
}

int dk_leds_init(void)
{
	int err;

	for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
		err = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT);
		if (err) {
			LOG_ERR("Cannot configure LED gpio");
			return err;
		}
	}

	return dk_set_leds_state(DK_NO_LEDS_MSK, DK_ALL_LEDS_MSK);
}

static void button_pressed(const struct device *gpio_dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	int err;
	k_spinlock_key_t key = k_spin_lock(&lock);

	switch (state) {
	case STATE_WAITING:
		if (gpio_dev == GPIO0_DEV || gpio_dev == GPIO1_DEV) {
			/* GPIO0 & GPIO1 has active high triggered interrupts and must be
			 * disabled here to avoid successive events from blocking the
			 * buttons_scan_fn function.
			 */
			err = callback_ctrl(false);
			if (err) {
				LOG_ERR("Failed disabling interrupts");
			}
			irq_enabled = false;
		}

		state = STATE_SCANNING;
		k_work_reschedule(&buttons_scan, K_MSEC(1));
		break;

	case STATE_SCANNING:
	default:
		/* Do nothing */
		break;
	}

	k_spin_unlock(&lock, key);
}

int dk_buttons_init(button_handler_t button_handler)
{
	int err;

	button_handler_cb = button_handler;

	if (IS_ENABLED(CONFIG_DK_LIBRARY_DYNAMIC_BUTTON_HANDLERS)) {
		k_mutex_init(&button_handler_mut);
	}

	for (size_t i = 0; i < ARRAY_SIZE(buttons); i++) {
		/* Enable pull resistor towards the inactive voltage. */
		gpio_flags_t flags =
			buttons[i].dt_flags & GPIO_ACTIVE_LOW ?
			GPIO_PULL_UP : GPIO_PULL_DOWN;
		if (buttons[i].dt_flags >> 4) {
			err = gpio_pin_configure_dt(&buttons[i], GPIO_INPUT | flags);
		} else {
			err = gpio_pin_configure_dt(&buttons[i], GPIO_INPUT);
		}

		if (err) {
			LOG_ERR("Cannot configure button gpio");
			return err;
		}
	}

	uint32_t pin_mask = 0;

	for (size_t i = 0; i < ARRAY_SIZE(buttons); i++) {
		/* Module starts in scanning mode and will switch to
		 * callback mode if no button is pressed.
		 */
		err = gpio_pin_interrupt_configure_dt(&buttons[i],
						      GPIO_INT_DISABLE);
		if (err) {
			LOG_ERR("Cannot disable callbacks()");
			return err;
		}

		pin_mask |= BIT(buttons[i].pin);
	}

	gpio_init_callback(&gpio_cb, button_pressed, pin_mask);

	for (size_t i = 0; i < ARRAY_SIZE(buttons); i++) {
		key_info.key_status[i].key_indx = i;
		err = gpio_add_callback(buttons[i].port, &gpio_cb);
		if (err) {
			LOG_ERR("Cannot add callback");
			return err;
		}
	}

	k_work_init_delayable(&buttons_scan, buttons_scan_fn);

	state = STATE_SCANNING;

	k_work_schedule(&buttons_scan, K_NO_WAIT);

	dk_read_buttons(NULL, NULL);

	atomic_set(&my_buttons, (atomic_val_t)get_buttons());

	return 0;
}

#ifdef CONFIG_DK_LIBRARY_DYNAMIC_BUTTON_HANDLERS
void dk_button_handler_add(struct button_handler *handler)
{
	k_mutex_lock(&button_handler_mut, K_FOREVER);
	sys_slist_append(&button_handlers, &handler->node);
	k_mutex_unlock(&button_handler_mut);
}

int dk_button_handler_remove(struct button_handler *handler)
{
	bool found;

	k_mutex_lock(&button_handler_mut, K_FOREVER);
	found = sys_slist_find_and_remove(&button_handlers, &handler->node);
	k_mutex_unlock(&button_handler_mut);

	return found ? 0 : -ENOENT;
}
#endif

void dk_read_buttons(uint32_t *button_state, uint32_t *has_changed)
{
	static uint32_t last_state;
	uint32_t current_state = atomic_get(&my_buttons);

	if (button_state != NULL) {
		*button_state = current_state;
	}

	if (has_changed != NULL) {
		*has_changed = (current_state ^ last_state);
	}

	last_state = current_state;
}

uint32_t dk_get_buttons(void)
{
	return atomic_get(&my_buttons);
}

int dk_set_leds(uint32_t leds)
{
	return dk_set_leds_state(leds, DK_ALL_LEDS_MSK);
}

int dk_set_leds_state(uint32_t leds_on_mask, uint32_t leds_off_mask)
{
	if ((leds_on_mask & ~DK_ALL_LEDS_MSK) != 0 ||
	   (leds_off_mask & ~DK_ALL_LEDS_MSK) != 0) {
		return -EINVAL;
	}

	for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
		int val, err;

		if (BIT(i) & leds_on_mask) {
			val = 1;
		} else if (BIT(i) & leds_off_mask) {
			val = 0;
		} else {
			continue;
		}

		err = gpio_pin_set_dt(&leds[i], val);
		if (err) {
			LOG_ERR("Cannot write LED gpio");
			return err;
		}
	}

	return 0;
}

int dk_set_led(uint8_t led_idx, uint32_t val)
{
	int err;

	if (led_idx >= ARRAY_SIZE(leds)) {
		// LOG_ERR("LED index out of the range");
		return -EINVAL;
	}
	err = gpio_pin_set_dt(&leds[led_idx], val);
	if (err) {
		LOG_ERR("Cannot write LED gpio");
	}
	return err;
}

int dk_toggle_led(uint8_t led_idx)
{
	int err;

	if (led_idx >= ARRAY_SIZE(leds)) {
		// LOG_ERR("LED index out of the range");
		return -EINVAL;
	}
	err = gpio_pin_toggle_dt(&leds[led_idx]);
	if (err) {
		LOG_ERR("Cannot toggle LED gpio");
	}
	return err;
}

int dk_set_led_on(uint8_t led_idx)
{
	return dk_set_led(led_idx, 1);
}

int dk_set_led_off(uint8_t led_idx)
{
	return dk_set_led(led_idx, 0);
}

static const struct device *led_pwm;
static uint8_t beep_brightness;
int dk_pwmled_init(void)
{
	int err = 0;

	uint8_t led;

#if CONFIG_BOARD_NRF21540DK_NRF52840 == 0 && CONFIG_PWM
	led_pwm = DEVICE_DT_GET(LED_PWM_NODE_ID);
	if (!device_is_ready(led_pwm)) {
		LOG_ERR("Device %s is not ready", led_pwm->name);
		return -1;
	}
#endif

	return err;
}

int dk_pwmled_set(uint8_t brightness)
{
	int err = -1;
#if CONFIG_BOARD_NRF21540DK_NRF52840 == 0 && CONFIG_PWM
	if (brightness == 0) {
		err = led_on(led_pwm, 0);
	} else {
		err = led_set_brightness(led_pwm, 0, brightness);
	}
	if (err < 0) {
		LOG_ERR("err=%d brightness=%d", err, brightness);
	}
	beep_brightness = brightness;
#endif

	return err;
}

uint8_t dk_pwmled_get(void)
{
	return beep_brightness;
}

int dk_adcs_init(void)
{
	int err;

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!device_is_ready(adc_channels[i].dev)) {
			LOG_INF("ADC controller device %s not ready", adc_channels[i].dev->name);
			return 0;
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			LOG_INF("Could not setup channel #%d (%d)", i, err);
			return 0;
		}
	}

	return err;
}

int dk_get_adc(int16_t *adc_value)
{
	int err;
	uint16_t buf;

	int32_t val_mv;

	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {

		// LOG_INF("- %s, channel %d: ",
		// 		adc_channels[i].dev->name,
		// 		adc_channels[i].channel_id);

		(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

		err = adc_read(adc_channels[i].dev, &sequence);
		if (err < 0) {
			LOG_INF("Could not read (%d)", err);
			continue;
		}

		/*
			* If using differential mode, the 16 bit value
			* in the ADC sample buffer should be a signed 2's
			* complement value.
			*/
		val_mv = (int32_t)((int16_t)buf);
		err = adc_raw_to_millivolts_dt(&adc_channels[i],
							&val_mv);
		/* conversion to mV may not be supported, skip if not */
		if (err < 0) {
			LOG_INF(" (value in mV not available)");
		} else {
			// LOG_INF("val_mv=%dmV", val_mv);
			*(adc_value + i) = val_mv;
		}
	}

	return err;
}

int dk_flash_init(void)
{
	int err = 0;

#if CONFIG_BOARD_NRF21540DK_NRF52840 == 0
	if (!device_is_ready(flash_dev)) {
		LOG_INF("Flash device not ready");
		// err = -1;
	}
#endif

	return err;
}

int dk_flash_contral(uint8_t type, uint8_t *buf, uint16_t len, uint32_t addr)
{
	int err = -1;
	// LOG_INF("type=%d, addr=%X", type, addr);
#if CONFIG_BOARD_NRF21540DK_NRF52840 == 0
	if (type == 0) {
		err = flash_read(flash_dev, addr, buf, len);
		if (err) {
			LOG_INF("Flash read failed!");
		}
	} else if (type == 1) {
		err = flash_write(flash_dev, addr, buf, len);
		if (err) {
			LOG_INF("Flash write failed!");
		}
	} else if (type == 2) {
		err = flash_erase(flash_dev, addr, FLASH_PAGE_SIZE);
		if (err) {
			LOG_INF("Flash erase failed!");
		}
	}
	// LOG_HEXDUMP_INF(buf, len, __FUNCTION__);
#endif

	return err;
}
