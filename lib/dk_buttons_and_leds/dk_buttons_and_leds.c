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
#if defined(CONFIG_DISPLAY) && (CONFIG_DISPLAY == 1)
#include <zephyr/drivers/display.h>
#include "oledfont.h"
#endif

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

#if defined(CONFIG_DISPLAY) && (CONFIG_DISPLAY == 1)

const struct device *display_dev;
struct display_capabilities capabilities;
struct display_buffer_descriptor buf_desc;
static uint8_t OLED_GRAM[4][128];
static struct k_work_delayable oled_scan;
uint8_t oled_status = 0;
static uint16_t oled_close_timer;
extern uint8_t oled_status_switch(uint8_t new_status);

static void oled_scan_fn(struct k_work *work);

void OLED_Refresh(void)
{
	display_write(display_dev, 0, 0, &buf_desc, OLED_GRAM);
}

// 清屏函数
void OLED_Clear(void)
{
	memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
	OLED_Refresh();
}

// 画点
// x:0~127
// y:0~63
// t:1 填充 0,清空
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
	if (t) {
		OLED_GRAM[y >> 3][x] |= (1 << (y & 0x07));
	} else {
		OLED_GRAM[y >> 3][x] &= ~(1 << (y & 0x07));
	}
}

// 画线
// x1,y1:起点坐标
// x2,y2:结束坐标
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode)
{
	uint16_t t;
	int xerr = 0, yerr = 0, delta_x, delta_y, distance;
	int incx, incy, uRow, uCol;
	delta_x = x2 - x1; // 计算坐标增量
	delta_y = y2 - y1;
	uRow = x1; // 画线起点坐标
	uCol = y1;
	if (delta_x > 0) {
		incx = 1; // 设置单步方向
	} else if (delta_x == 0) {
		incx = 0; // 垂直线
	} else {
		incx = -1;
		delta_x = -delta_x;
	}
	if (delta_y > 0) {
		incy = 1;
	} else if (delta_y == 0) {
		incy = 0; // 水平线
	} else {
		incy = -1;
		delta_y = -delta_x;
	}
	if (delta_x > delta_y) {
		distance = delta_x; // 选取基本增量坐标轴
	} else {
		distance = delta_y;
	}
	for (t = 0; t < distance + 1; t++) {
		OLED_DrawPoint(uRow, uCol, mode); // 画点
		xerr += delta_x;
		yerr += delta_y;
		if (xerr > distance) {
			xerr -= distance;
			uRow += incx;
		}
		if (yerr > distance) {
			yerr -= distance;
			uCol += incy;
		}
	}
}

// 矩形
// x1,y1:起点坐标
// x2,y2:结束坐标
// mode:1 填充 0,清空
void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t mode)
{
	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			OLED_DrawPoint(x + j, y + i, mode); // 画点
		}
	}
}

// x,y:圆心坐标
// r:圆的半径
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
	int a, b, num;
	a = 0;
	b = r;
	while (2 * b * b >= r * r) {
		OLED_DrawPoint(x + a, y - b, 1);
		OLED_DrawPoint(x - a, y - b, 1);
		OLED_DrawPoint(x - a, y + b, 1);
		OLED_DrawPoint(x + a, y + b, 1);

		OLED_DrawPoint(x + b, y + a, 1);
		OLED_DrawPoint(x + b, y - a, 1);
		OLED_DrawPoint(x - b, y - a, 1);
		OLED_DrawPoint(x - b, y + a, 1);

		a++;
		num = (a * a + b * b) - r * r; // 计算画的点离圆心的距离
		if (num > 0) {
			b--;
			a--;
		}
	}
}

// 在指定位置显示一个字符,包括部分字符
// x:0~127
// y:0~63
// size1:选择字体 6x8/6x12/8x16/12x24
// mode:0,反色显示;1,正常显示
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode)
{
    uint8_t i, m, temp, size2, chr1;
    uint8_t x0 = x, y0 = y;
    if (size1 == 8) {
		size2 = 6;
    } else {
		size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2); // 得到字体一个字符对应点阵集所占的字节数
    }
    chr1 = chr - ' '; // 计算偏移后的值
    for (i = 0; i < size2; i++) {
		if (size1 == 8) {
			temp = asc2_0806[chr1][i];
		} // 调用0806字体
		else if (size1 == 12) {
			temp = asc2_1206[chr1][i];
		} // 调用1206字体
		else if (size1 == 16) {
			temp = asc2_1608[chr1][i];
		} // 调用1608字体
		else if (size1 == 24) {
			temp = asc2_2412[chr1][i];
		} // 调用2412字体
		else {
			return;
		}
		for (m = 0; m < 8; m++) {
			if (temp & 0x01) {
					OLED_DrawPoint(x, y, mode);
			} else {
					OLED_DrawPoint(x, y, !mode);
			}
			temp >>= 1;
			y++;
		}
		x++;
		if ((size1 != 8) && ((x - x0) == size1 / 2)) {
			x = x0;
			y0 = y0 + 8;
		}
		y = y0;
    }
}

// 显示字符串
// x,y:起点坐标
// size1:字体大小
//*chr:字符串起始地址
// mode:0,反色显示;1,正常显示
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1, uint8_t mode)
{
    while ((*chr >= ' ') && (*chr <= '~')) // 判断是不是非法字符!
    {
		OLED_ShowChar(x, y, *chr, size1, mode);
		if (size1 == 8) {
			x += 6;
		} else {
			x += size1 / 2;
		}
		chr++;
    }
}

// m^n
int32_t OLED_Pow(int8_t m, int8_t n)
{
    int32_t result = 1;
    while (n--) {
		result *= m;
    }
    return result;
}

// 显示数字
// x,y :起点坐标
// len :数字的位数
// size:字体大小
// mode:0,反色显示;1,正常显示
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t size1, uint8_t mode)
{
    uint8_t t, temp, m = 0;
	char sign_num = '+';
	LOG_INF("%s=%d", __FUNCTION__, num);

	if (num < 0) {
		num = -num;
		sign_num = '-';
	}
	num /= 10;

    if (size1 == 8) {
		m = 2;
    }
	int i = 0;
    for (; i < len; i++) {
		temp = (num / OLED_Pow(10, len - i - 1)) % 10;
		if (temp != 0) {
			break;
		}
		uint8_t w, h;
		if (size1 == 8) {
			h = 8;
			w = 6;
		} // 调用0806字体
		else if (size1 == 12) {
			h = 12;
			w = 6;
		} // 调用1206字体
		else if (size1 == 16) {
			h = 16;
			w = 8;
		} // 调用1608字体
		else if (size1 == 24) {
			h = 24;
			w = 12;
		} // 调用2412字体
		else {
			return;
		}
		if (i == len - 1) {
			OLED_ShowChar(x + (size1 / 2 + m) * i - 6, y + size1 / 2 - 3, sign_num, 8, true);
			OLED_ShowChar(x + (size1 / 2 + m) * i, y, '0', size1, mode);
		} else {
			OLED_DrawRectangle(x + (size1 / 2 + m) * i - 6, y + size1 / 2 - 3, 6, 8, false);
			OLED_DrawRectangle(x + (size1 / 2 + m) * i, y, w, h, false);
		}
    }
	for (t = i; t < len; t++) {
		temp = (num / OLED_Pow(10, len - t - 1)) % 10;
		// LOG_INF("%d ", temp);
		if (i == t) {
			OLED_ShowChar(x + (size1 / 2 + m) * t - 6, y + size1 / 2 - 3, sign_num, 8, true);
		}
		if (temp == 0) {
			OLED_ShowChar(x + (size1 / 2 + m) * t, y, '0', size1, mode);
		} else {
			OLED_ShowChar(x + (size1 / 2 + m) * t, y, temp + '0', size1, mode);
		}
	}
}

// 显示汉字
// x,y:起点坐标
// num:汉字对应的序号
// mode:0,反色显示;1,正常显示
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode)
{
    uint8_t m, temp;
    uint8_t x0 = x, y0 = y;
    uint16_t i, size3 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) *
		   size1; // 得到字体一个字符对应点阵集所占的字节数
    for (i = 0; i < size3; i++) {
		if (size1 == 16) {
			temp = Hzk1[num][i];
		} // 调用16*16字体
		else if (size1 == 24) {
			temp = Hzk2[num][i];
		} // 调用24*24字体
		else if (size1 == 32) {
			temp = Hzk3[num][i];
		} // 调用32*32字体
		else if (size1 == 64) {
			temp = Hzk4[num][i];
		} // 调用64*64字体
		else {
			return;
		}
		for (m = 0; m < 8; m++) {
			if (temp & 0x01) {
					OLED_DrawPoint(x, y, mode);
			} else {
					OLED_DrawPoint(x, y, !mode);
			}
			temp >>= 1;
			y++;
		}
		x++;
		if ((x - x0) == size1) {
			x = x0;
			y0 = y0 + 8;
		}
		y = y0;
    }
}

// num 显示汉字的个数
// space 每一遍显示的间隔
// mode:0,反色显示;1,正常显示
void OLED_ScrollDisplay(uint8_t num, uint8_t space, uint8_t mode)
{
    uint8_t i, n, t = 0, m = 0, r;
    while (1) {
		if (m == 0) {
			OLED_ShowChinese(128, 8, t, 16,
					 mode); // 写入一个汉字保存在OLED_GRAM[][]数组中
			t++;
		}
		if (t == num) {
			for (r = 0; r < 16 * space; r++) // 显示间隔
			{
					for (i = 1; i < 144; i++) {
						for (n = 0; n < 4; n++) {
							OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
						}
					}
					OLED_Refresh();
			}
			t = 0;
		}
		m++;
		if (m == 16) {
			m = 0;
		}
		for (i = 1; i < 144; i++) // 实现左移
		{
			for (n = 0; n < 4; n++) {
					OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
			}
		}
		OLED_Refresh();
    }
}

// x,y：起点坐标
// sizex,sizey,图片长宽
// BMP[]：要写入的图片数组
// mode:0,反色显示;1,正常显示
void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], uint8_t mode)
{
    uint16_t j = 0;
    uint8_t i, n, temp, m;
    uint8_t x0 = x, y0 = y;
    sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
    for (n = 0; n < sizey; n++) {
		for (i = 0; i < sizex; i++) {
			temp = BMP[j];
			j++;
			for (m = 0; m < 8; m++) {
				if (temp & 0x01) {
					OLED_DrawPoint(x, y, mode);
				} else {
					OLED_DrawPoint(x, y, !mode);
				}
				temp >>= 1;
				y++;
			}
			x++;
			if ((x - x0) == sizex) {
					x = x0;
					y0 = y0 + 8;
			}
			y = y0;
		}
    }
}

int dk_oled_init(void)
{
	int err = 0;

	if (!device_is_ready(flash_dev)) {
		LOG_INF("Flash device not ready");
		// err = -1;
	}
	size_t buf_size = 0;

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device %s not found. Aborting sample.",
			display_dev->name);
		return 0;
	}

	LOG_INF("Display sample for %s", display_dev->name);
	display_get_capabilities(display_dev, &capabilities);

	LOG_HEXDUMP_INF(&capabilities, sizeof(capabilities), "capabilities");

	buf_desc.buf_size = (capabilities.x_resolution * capabilities.y_resolution) >> 3;
	buf_desc.width = capabilities.x_resolution;
	buf_desc.pitch = capabilities.x_resolution;
	buf_desc.height = capabilities.y_resolution;

	buf_size = capabilities.x_resolution * capabilities.y_resolution / 8;

	LOG_INF("buf_size=%d", buf_size);

	display_blanking_on(display_dev);

	oled_status_switch(OLED_STATUS_INIT);

	k_work_init_delayable(&oled_scan, oled_scan_fn);

	k_work_schedule(&oled_scan, K_MSEC(100));


	return err;
}

void oled_close_temperature(uint8_t indx)
{
	LOG_INF("%s=%d", __FUNCTION__, indx);
	const uint8_t offset_central = 4;
	// 计算温度显示起始坐标，按温度值左右两边离屏幕中心的固定偏移排列
	uint8_t offset = 63 + indx + OLED_Pow(-1, indx + 1) * (6 + offset_central  + (1 - indx) * (36 + 10) + indx * 6);
	LOG_INF("offset=%d", offset);
	OLED_DrawRectangle(offset - 6, 7, 36 + 10 + 1 + 6, 25, false);
	OLED_ShowString(offset, 7, "- -", 24, true);

	OLED_Refresh();
}

static uint8_t flicker_num[2];
static uint8_t flicker_indx;
void oled_update_temperature(uint8_t is_centigrade, int16_t temperature0, int16_t temperature1)
{
	LOG_INF("temperature=%d,%d C", temperature0, temperature1);
	int16_t temperature[] = { temperature0, temperature1 };

	for (int i = 0; i < 2; i++) {
		const uint8_t offset_central = 4;
		// 计算温度显示起始坐标，按温度值左右两边离屏幕中心的固定偏移排列
		uint8_t offset = 63 + i + OLED_Pow(-1, i + 1) * (6 + offset_central  + (1 - i) * (36 + 10) + i * 6);
		if (temperature[i] != 0x7fff) {
			OLED_ShowChar(offset + 36, 23, '.', 8, true);
			OLED_ShowNum(offset, 7, temperature[i], 3, 24, true);
			if (temperature[i] < 0) {
				temperature[i] = -temperature[i];
			}
			OLED_ShowChar(offset + 41, 23, '0' + temperature[i] % 10, 8, true);
			flicker_num[i] = ((temperature[i] / 10) % 10) + '0';
		} else {
			OLED_DrawRectangle(offset - 6, 7, 36 + 10 + 1 + 6, 25, false);
			OLED_ShowString(offset, 7, "- -", 24, true);
			flicker_num[i] = '-';
		}
	}
	OLED_ShowChar(60, 16, is_centigrade ? 'C' : 'F', 16, true);	// 温度单位 F

	OLED_Refresh();
}

void oled_update_charge(uint8_t charge0, uint8_t charge1)
{
	LOG_INF("temperature=%d,%d %%", charge0, charge1);
	uint8_t charge[] = { charge0, charge1 };

	for (int i = 0; i < 2; i++) {
		const uint8_t offset_central = 70;
		// 计算温度显示起始坐标，按温度值左右两边离屏幕中心的固定偏移排列
		uint8_t offset = 10 + offset_central * i;
		if (charge[i] != 0xff) {
			OLED_ShowPicture(offset, 0, 18, 7, gImage_charge_array[charge[i] / 10], true);
		} else {
			OLED_DrawRectangle(offset, 0, 18, 7, false);
		}
	}

	OLED_Refresh();
}

void oled_update_charge_big(uint8_t charge)
{
	LOG_INF("temperature=%d %%", charge);

	// 计算温度显示起始坐标，按温度值左右两边离屏幕中心的固定偏移排列
	OLED_ShowPicture(60, 0, 8, 16, gImage_charge_big_array[charge / 10], true);

	OLED_Refresh();
}

void oled_flicker_set(void)
{
	switch (oled_status) {
	case OLED_STATUS_SHOW_NOW:
	case OLED_STATUS_SHOW_TARGET: {
		oled_status_switch(OLED_STATUS_ADJUST0);
		break;
	}
	case OLED_STATUS_ADJUST0: {
		oled_status_switch(OLED_STATUS_ADJUST1);
		break;
	}
	case OLED_STATUS_ADJUST1: {
		oled_status_switch(OLED_STATUS_SHOW_TARGET);
		break;
	}
	default: {
		oled_status_switch(OLED_STATUS_ADJUST0);
		break;
	}
	}
}

static void oled_scan_fn(struct k_work *work)
{
	uint16_t next_delay = 500;
	static uint8_t flicker_state;
	static uint8_t old_status;
	const uint8_t offset_central = 4;

	if (old_status != oled_status) {
		LOG_INF("%s=%d,%d", __FUNCTION__, old_status, oled_status);
		oled_close_timer = OLED_SLEEP_TIMEOUT;
		switch (oled_status) {
		case OLED_STATUS_IDLE: {
			display_blanking_on(display_dev);
			break;
		}
		case OLED_STATUS_INIT: {
			OLED_Clear();
			// OLED_ShowString(10, 0, "Meating", 8, true);
			OLED_DrawLine(58, 0, 58, 31, true);		// 左竖线
			OLED_DrawLine(69, 0, 69, 31, true);		// 右竖线
			// OLED_ShowString(80, 0, "Meating", 8, true);
			oled_close_temperature(0);
			oled_close_temperature(1);
			// oled_update_temperature(0, 888);
			OLED_ShowChar(60, 16, 'F', 16, true);	// 温度单位 F

			OLED_Refresh();
			oled_status_switch(OLED_STATUS_SHOW_NOW);
			break;
		}
		case OLED_STATUS_SHOW_NOW: {
			if (old_status == OLED_STATUS_ADJUST1) {
				uint8_t indx = 1;
				uint8_t offset_old = 63 + indx + OLED_Pow(-1, indx + 1) * (6 + offset_central  + (1 - indx) * (36 + 10) + indx * 6);
				OLED_ShowChar(offset_old + 24, 7, flicker_num[indx], 24, true);
			}
			break;
		}
		case OLED_STATUS_SHOW_TARGET: {

			break;
		}
		case OLED_STATUS_ADJUST0: {
			flicker_state = true;

			break;
		}
		case OLED_STATUS_ADJUST1: {
			uint8_t indx = 0;
			uint8_t offset_old = 63 + indx + OLED_Pow(-1, indx + 1) * (6 + offset_central  + (1 - indx) * (36 + 10) + indx * 6);
			OLED_ShowChar(offset_old + 24, 7, flicker_num[indx], 24, true);
			flicker_state = true;
			break;
		}
		default:
			break;
		}
		if (old_status == OLED_STATUS_IDLE) {
			display_blanking_off(display_dev);
		}
		old_status = oled_status;
	} else {
		switch (oled_status) {
		case OLED_STATUS_IDLE: {

			break;
		}
		case OLED_STATUS_INIT: {

			break;
		}
		case OLED_STATUS_SHOW_NOW: {

			break;
		}
		case OLED_STATUS_SHOW_TARGET: {

			break;
		}
		case OLED_STATUS_ADJUST0:
		case OLED_STATUS_ADJUST1: {
			uint8_t indx = oled_status - OLED_STATUS_ADJUST0;
			uint8_t offset = 63 + indx + OLED_Pow(-1, indx + 1) * (6 + offset_central  + (1 - indx) * (36 + 10) + indx * 6);

			if (flicker_state) {
				OLED_DrawRectangle(offset + 24, 7, 12, 24, false);
			} else {
				OLED_ShowChar(offset + 24, 7, flicker_num[indx], 24, true);
			}
			flicker_state = !flicker_state;

			break;
		}
		default:
			break;
		}

	}

	if (oled_status != OLED_STATUS_IDLE) {
		oled_close_timer -= next_delay;
		// LOG_INF("oled_close_timer=%d", oled_close_timer);
		if (oled_close_timer == 0) {
			oled_status_switch(OLED_STATUS_IDLE);
		}
	}

	OLED_Refresh();

	k_work_reschedule(&oled_scan, K_MSEC(next_delay));
}

void refresh_oled_time(void)
{
	// LOG_INF("%s=%d", __FUNCTION__, oled_close_timer);
	oled_close_timer = OLED_SLEEP_TIMEOUT;
}

#else

uint8_t oled_status;
int dk_oled_init(void){ return 0; }
void oled_update_temperature(uint8_t is_centigrade, int16_t temperature0, int16_t temperature1){}
void oled_flicker_set(void){}
void refresh_oled_time(void){}
void oled_update_charge(uint8_t charge0, uint8_t charge1){}
void oled_update_charge_big(uint8_t charge){}

#endif
