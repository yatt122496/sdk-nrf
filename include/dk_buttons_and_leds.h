/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef DK_BUTTON_AND_LEDS_H__
#define DK_BUTTON_AND_LEDS_H__

/** @file dk_buttons_and_leds.h
 * @brief Module for handling buttons and LEDs on Nordic DKs.
 * @defgroup dk_buttons_and_leds DK buttons and LEDs
 * @{
 */

#include <zephyr/types.h>
#include <zephyr/sys/slist.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DK_NO_LEDS_MSK    (0)
#define DK_LED1           0
#define DK_LED2           1
#define DK_LED3           2
#define DK_LED4           3
#define DK_LED1_MSK       BIT(DK_LED1)
#define DK_LED2_MSK       BIT(DK_LED2)
#define DK_LED3_MSK       BIT(DK_LED3)
#define DK_LED4_MSK       BIT(DK_LED4)
#define DK_ALL_LEDS_MSK   (DK_LED1_MSK | DK_LED2_MSK |\
			   DK_LED3_MSK | DK_LED4_MSK)

#define DK_NO_BTNS_MSK   (0)
#define DK_BTN1          0
#define DK_BTN2          1
#define DK_BTN3          2
#define DK_BTN4          3
#define DK_BTN1_MSK      BIT(DK_BTN1)
#define DK_BTN2_MSK      BIT(DK_BTN2)
#define DK_BTN3_MSK      BIT(DK_BTN3)
#define DK_BTN4_MSK      BIT(DK_BTN4)
#define DK_ALL_BTNS_MSK  (DK_BTN1_MSK | DK_BTN2_MSK | \
			  DK_BTN3_MSK | DK_BTN4_MSK)
/**
 * @typedef button_handler_t
 * @brief Callback that is executed when a button state change is detected.
 *
 * @param button_state Bitmask of button states.
 * @param has_changed Bitmask that shows which buttons have changed.
 */
typedef void (*button_handler_t)(uint32_t button_state, uint32_t has_changed);

/** Button handler list entry. */
struct button_handler {
	button_handler_t cb; /**< Callback function. */
	sys_snode_t node; /**< Linked list node, for internal use. */
};

/** @brief Initialize the library to control the LEDs.
 *
 *  @retval 0           If the operation was successful.
 *                      Otherwise, a (negative) error code is returned.
 */
int dk_leds_init(void);

/** @brief Initialize the library to read the button state.
 *
 *  @param  button_handler Callback handler for button state changes.
 *
 *  @retval 0           If the operation was successful.
 *                      Otherwise, a (negative) error code is returned.
 */
int dk_buttons_init(button_handler_t button_handler);

/** @brief Add a dynamic button handler callback.
 *
 * In addition to the button handler function passed to
 * @ref dk_buttons_init, any number of button handlers can be added and removed
 * at runtime.
 *
 * @param[in] handler Handler structure. Must point to statically allocated
 * memory.
 */
void dk_button_handler_add(struct button_handler *handler);

/** @brief Remove a dynamic button handler callback.
 *
 * @param[in] handler Handler to remove.
 *
 * @retval 0 Successfully removed the handler.
 * @retval -ENOENT This button handler was not present.
 */
int dk_button_handler_remove(struct button_handler *handler);

/** @brief Read current button states.
 *
 *  @param button_state Bitmask of button states.
 *  @param has_changed Bitmask that shows which buttons have changed.
 */
void dk_read_buttons(uint32_t *button_state, uint32_t *has_changed);

/** @brief Get current button state from internal variable.
 *
 *  @return Bitmask of button states.
 */
uint32_t dk_get_buttons(void);

/** @brief Set value of LED pins as specified in one bitmask.
 *
 *  @param  leds Bitmask that defines which LEDs to turn on and off.
 *
 *  @retval 0           If the operation was successful.
 *                      Otherwise, a (negative) error code is returned.
 */
int dk_set_leds(uint32_t leds);


/** @brief Set value of LED pins as specified in two bitmasks.
 *
 *  @param  leds_on_mask  Bitmask that defines which LEDs to turn on.
 *                        If this bitmask overlaps with @p leds_off_mask,
 *                        @p leds_on_mask has priority.
 *
 *  @param  leds_off_mask Bitmask that defines which LEDs to turn off.
 *                        If this bitmask overlaps with @p leds_on_mask,
 *                        @p leds_on_mask has priority.
 *
 *  @retval 0           If the operation was successful.
 *                      Otherwise, a (negative) error code is returned.
 */
int dk_set_leds_state(uint32_t leds_on_mask, uint32_t leds_off_mask);

/** @brief Set a single LED value.
 *
 *  This function turns a single LED on or off.
 *
 *  @param led_idx Index of the LED.
 *  @param val     Value for the LED: 1 - turn on, 0 - turn off
 *
 *  @retval 0           If the operation was successful.
 *                      Otherwise, a (negative) error code is returned.
 *
 *  @sa dk_set_led_on, dk_set_led_off
 */
int dk_set_led(uint8_t led_idx, uint32_t val);

int dk_toggle_led(uint8_t led_idx);

/** @brief Turn a single LED on.
 *
 *  @param led_idx Index of the LED.
 *
 *  @retval 0           If the operation was successful.
 *                      Otherwise, a (negative) error code is returned.
 */
int dk_set_led_on(uint8_t led_idx);

/** @brief Turn a single LED off.
 *
 *  @param led_idx Index of the LED.
 *
 *  @retval 0           If the operation was successful.
 *                      Otherwise, a (negative) error code is returned.
 */
int dk_set_led_off(uint8_t led_idx);

typedef union
{
	unsigned long all;
	struct {
		union {
			unsigned char all;
			struct {
				unsigned char press :1;
				unsigned char short_press :1;
				unsigned char multi_press :1;
				unsigned char long_press :1;
				unsigned char reserve :4;
			} Bit;
		} event;
		union {
			unsigned char all;
			struct {
				unsigned char reserve;
			} Bit;
		} reserve_byte;
		union {
			unsigned short all;
			struct {
				unsigned short press_status :1;
				unsigned short reserve :7;
				unsigned short mutle_num :4;
				unsigned short long_time :4;
			} Bit;
		} status;
	} value;
} KEY_INFO;

typedef struct
{
	int key_indx;
	// uint32_t delay_time;
	uint32_t press_time;
	uint32_t release_time;
	KEY_INFO key_value;
} KEY_STATUS;

#define	KEY_NUM_MAX				10

typedef uint8_t (* get_key_state_cb_t)(int key);
typedef struct
{
	get_key_state_cb_t get_key_state;
	KEY_STATUS key_status[KEY_NUM_MAX];
} KEY_REGEDIT;


#define	OLED_SLEEP_TIMEOUT		60000

#define BUTTON_NONE				-1

#define	TIMER_10MS				10
#define	LONG_KEY_DELAY			(1000u)
#define	LONG_KEY_SPEED0			(200u)
#define	LONG_KEY_SPEED1			(100u)
#define	LONG_KEY_SPEED2			(50u)
#define	SHORT_KEY_DELAY			(200u)

#define	KEY_PRESS_IO_STATUS		1

KEY_INFO get_key_info_value(int key_index);
uint32_t get_key_info_event(void);

int dk_adcs_init(void);

int dk_get_adc(int16_t *adc_value);

int dk_pwmled_init(void);

int dk_pwmled_set(uint8_t brightness);

uint8_t dk_pwmled_get(void);

int dk_flash_init(void);

int dk_flash_contral(uint8_t type, uint8_t *buf, uint16_t len, uint32_t addr);

typedef uint8_t (* button_active_cb_t)(KEY_INFO key);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* DK_BUTTON_AND_LEDS_H__ */
