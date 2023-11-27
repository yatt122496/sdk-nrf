/*
 * Copyright (c) 2021 Nordic Semiconductor
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_BBQOVN_CLIENT_H_
#define BT_BBQOVN_CLIENT_H_

/**
 * @file
 * @defgroup bt_bbqovn_client Heart Rate Service Client
 * @{
 * @brief Heart Rate (HR) Service Client API.
 */

#include <zephyr/bluetooth/conn.h>
#include <bluetooth/gatt_dm.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/sys/atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRF_BLE_LINK_COUNT							CONFIG_BT_MAX_CONN
#define SLAVE_MAX_NUM								0
#define MASTER_MAX_NUM								(CONFIG_BT_MAX_CONN - SLAVE_MAX_NUM)
// #define MASTER_MAX_NUM								(CONFIG_BT_MAX_CONN - CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT)
// #define SLAVE_MAX_NUM								CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT

#define	USER_VBAT_LOW_VALUE				(3400)	// 940mv
#define	USER_VBAT_FILTER_NUM			3
#define	USER_VBAT_SLEEP_VALUE			(USER_VBAT_LOW_VALUE - 500)
#define	USER_VBAT_SLEEP_FILTER_NUM		3

#define	USER_SCAN_FILTER_RSSI_MAX			-65
#define	USER_SCAN_FILTER_RSSI_STEP			-5
#define	USER_SCAN_FILTER_RSSI_MIN			-100

#define	USER_CHARGE_FULL_FILTER_NUM			5
#define	USER_CHARGE_FULL_VALUE_MAX			0
#define	USER_CHARGE_FULL_VALUE_MIN			-2

#define	USER_CHARGE_NULL_VALUE_MAX			-3
#define	USER_CHARGE_NULL_VALUE_MIN			-4
#define	USER_CHARGE_NULL_FILTER_NUM			5


typedef enum {
	BEEP_MODE_WARNING = 0,
	BEEP_MODE_BBQ_TURN,
	BEEP_MODE_BBQ_SUCCESS,
	BEEP_MODE_DANGEROUS,
	BEEP_MODE_TEST,
} beep_mode_t;

struct bt_bbqovn_measurement {

	/** Battery percentage. */
	uint8_t power;

	/** temperature sign +-. */
	uint8_t sign;

	/** BBQOVN temperature * 10. */
	uint8_t temperature[3];

};

/**@brief Data structure of the Heart Rate Measurement characteristic.
 */
typedef union {
	struct bbqovn_client_measurement {
		/** BBQOVN Probe Number. */
		uint8_t indx;

		/** Separator ':'. */
		uint8_t split0;

		/** Last 3 bytes of BBQOVN MAC address. */
		uint8_t mac[6];

		/** Separator ':'. */
		uint8_t split1;

		struct bt_bbqovn_measurement bbqovn;

		/** END SIGN '#'. */
		uint8_t end;
	} bt;
	struct bbqovn_test_send {
		/** BBQOVN TEST Number. */
		uint8_t indx;

		uint8_t len;

		uint8_t data[12];

		uint8_t end;
	} bt_test;
} bbqovn_client_measurement_t;

/* Helper forward structure declaration representing Heart Rate Service Client instance.
 * Needed for callback declaration that are using instance structure as argument.
 */
struct bt_bbqovn_client;

/**@brief Heart Rate Measurement notification callback.
 *
 * This function is called every time the client receives a notification
 * with Heart Rate Measurement data.
 *
 * @param[in] bbqovn_c Heart Rate Service Client instance.
 * @param[in] meas Heart Rate Measurement received data.
 * @param[in] err 0 if the notification is valid.
 *                Otherwise, contains a (negative) error code.
 */
typedef void (*bt_bbqovn_client_notify_cb)(struct bt_bbqovn_client *bbqovn_c,
					const struct bt_bbqovn_measurement *meas,
					int err);

/**@brief bbqovn write callback.
 *
 * @param[in] hrs_c Heart Rate client instance.
 * @param[in] err 0 if write operation succeeded.
 *                Otherwise, contains an error code.
 */
typedef void (*bt_bbqovn_client_write_cb)(struct bt_bbqovn_client *bbqovn_c,
				       uint8_t err);

/**@brief Heart Rate Measurement characteristic structure.
 */
struct bt_bbqovn_client_hr_meas {
	/** Value handle. */
	uint16_t handle;

	/** Handle of the characteristic CCC descriptor. */
	uint16_t ccc_handle;

	/** GATT subscribe parameters for notification. */
	struct bt_gatt_subscribe_params notify_params;

	/** Notification callback. */
	bt_bbqovn_client_notify_cb notify_cb;
};

/**@brief bbqovn write characteristic structure.
 */
struct bt_bbqovn_client_write {
	/** Value handle. */
	uint16_t handle;

	/** Write parameters. */
	struct bt_gatt_write_params write_params;

	/** Write complete callback. */
	bt_bbqovn_client_write_cb write_cb;
};

/**@brief Heart Rate Service Client instance structure.
 *        This structure contains status information for the client.
 */
struct bt_bbqovn_client {
	/** Connection object. */
	struct bt_conn *conn;

	/** Heart Rate Measurement characteristic. */
	struct bt_bbqovn_client_hr_meas measurement_char;

	struct bt_bbqovn_client_write cp_char;

	bbqovn_client_measurement_t data_bak;

	bbqovn_client_measurement_t data;

	/** Internal state. */
	uint8_t indx;

	uint8_t unit;

	uint8_t turn;

	int16_t now_temperature;

	int16_t target_temperature_bak;

	int16_t target_temperature;

	/** Internal state. */
	atomic_t state;
};

/**@brief Function for initializing the Heart Rate Service Client.
 *
 * @param[in, out] bbqovn_c Heart Rate Service Client instance. This structure must be
 *                       supplied by the application. It is initialized by
 *                       this function and will later be used to identify
 *                       this particular client instance.
 *
 * @retval 0 If the client was initialized successfully.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_bbqovn_client_init(struct bt_bbqovn_client *bbqovn_c);

/**@brief Subscribe to Heart Rate Measurement notification.
 *
 * This function writes CCC descriptor of the Heart Rate Measurement characteristic
 * to enable notification.
 *
 * @param[in] bbqovn_c Heart Rate Service Client instance.
 * @param[in] notify_cb   Notification callback.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_bbqovn_client_measurement_subscribe(struct bt_bbqovn_client *bbqovn_c,
					bt_bbqovn_client_notify_cb notify_cb);

/**@brief Remove subscription to the Heart Rate Measurement notification.
 *
 * This function writes CCC descriptor of the Heart Rate Measurement characteristic
 * to disable notification.
 *
 * @param[in] bbqovn_c Heart Rate Service Client instance.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_bbqovn_client_measurement_unsubscribe(struct bt_bbqovn_client *bbqovn_c);

/**@brief Function for assigning handles to Heart Rate Service Client instance.
 *
 * @details Call this function when a link has been established with a peer to
 *          associate the link to this instance of the module. This makes it
 *          possible to handle several links and associate each link to a particular
 *          instance of this module.
 *
 * @param[in]     dm     Discovery object.
 * @param[in,out] bbqovn_c  Heart Rate Service Client instance for associating the link.
 *
 * @retval 0 If the operation is successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_bbqovn_client_handles_assign(struct bt_gatt_dm *dm, struct bt_bbqovn_client *bbqovn_c);

int bt_bbqovn_data_reset(struct bt_bbqovn_client *bbqovn_c);

/**@brief Write BBQOVN characteristic.
 *
 * @param[in] bbqovn_c BBQOVN Service Client instance.
 * @param[in] value    Control Point value to write.
 * @param[in] write_cb Write complete callback.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_bbqovn_client_control_point_write(struct bt_bbqovn_client *bbqovn_c,
				      uint8_t *buf, uint8_t len,
				      bt_bbqovn_client_write_cb write_cb);

#ifdef __cplusplus
}
#endif

/**
 *@}
 */

#endif /* BT_BBQOVN_CLIENT_H_ */
