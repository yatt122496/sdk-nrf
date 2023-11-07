/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <bluetooth/services/bbqovn_client.h>
#include <zephyr/bluetooth/services/bbqovn.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bbqovn_client, CONFIG_BT_BBQOVN_CLIENT_LOG_LEVEL);

#define BBQOVN_MEASUREMENT_NOTIFY_ENABLED BIT(0)
#define BBQOVN_MEASUREMENT_READ_IN_PROGRES BIT(1)
#define BBQOVN_SENSOR_LOCATION_READ_IN_PROGRES BIT(2)
#define BBQOVN_CONTROL_POINT_WRITE_PENDING BIT(3)

#define BBQOVN_MEASUREMENT_FLAGS_VALUE_FORMAT      BIT(0)
#define BBQOVN_MEASUREMENT_FLAGS_CONTACT_DETECTED  BIT(1)
#define BBQOVN_MEASUREMENT_FLAGS_CONTACT_SUPPORTED BIT(2)
#define BBQOVN_MEASUREMENT_FLAGS_ENERGY_EXPENDED   BIT(3)
#define BBQOVN_MEASUREMENT_FLAGS_RR_INTERVALS      BIT(4)

#ifndef CONFIG_BT_BBQOVN_DX2003_TEST_EN
#define CONFIG_BT_BBQOVN_DX2003_TEST_EN	0
#endif

static void bbqovn_reinit(struct bt_bbqovn_client *bbqovn_c)
{
	bbqovn_c->measurement_char.handle = 0;
	bbqovn_c->measurement_char.ccc_handle = 0;
	bbqovn_c->measurement_char.notify_cb = NULL;
	bbqovn_c->cp_char.handle = 0;
	bbqovn_c->cp_char.write_cb = NULL;

	// bbqovn_c->conn = NULL;
	bbqovn_c->state = ATOMIC_INIT(0);
}

static uint8_t on_bbqovn_measurement_notify(struct bt_conn *conn,
					 struct bt_gatt_subscribe_params *params,
					 const void *data, uint16_t length)
{
	int err = 0;
	struct bt_bbqovn_client *bbqovn_c;
	struct bt_bbqovn_measurement hr_measurement;

	bbqovn_c = CONTAINER_OF(params, struct bt_bbqovn_client, measurement_char.notify_params);

	if (!data) {
		atomic_clear_bit(&bbqovn_c->state, BBQOVN_MEASUREMENT_NOTIFY_ENABLED);
		LOG_DBG("[UNSUBSCRIBE] from BBQOVN characterictic");

		return BT_GATT_ITER_STOP;
	}

	LOG_HEXDUMP_INF(data, length, "[NOTIFICATION] BBQOVN: ");
	if (length <= 5) {
		memcpy(&hr_measurement, data, length);
	} else {
		memcpy(&hr_measurement, data, 5);
	}

	if (bbqovn_c->measurement_char.notify_cb) {
		bbqovn_c->measurement_char.notify_cb(bbqovn_c, &hr_measurement, err);
	}

	return BT_GATT_ITER_CONTINUE;
}

int bt_bbqovn_client_measurement_subscribe(struct bt_bbqovn_client *bbqovn_c,
					bt_bbqovn_client_notify_cb notify_cb)
{
	int err;
	struct bt_gatt_subscribe_params *params = &bbqovn_c->measurement_char.notify_params;

	if (!bbqovn_c || !notify_cb) {
		return -EINVAL;
	}

	if (atomic_test_and_set_bit(&bbqovn_c->state, BBQOVN_MEASUREMENT_NOTIFY_ENABLED)) {
		LOG_ERR("BBQOVN characterisic notifications already enabled.");
		return -EALREADY;
	}

	bbqovn_c->measurement_char.notify_cb = notify_cb;

	params->ccc_handle = bbqovn_c->measurement_char.ccc_handle;
	params->value_handle = bbqovn_c->measurement_char.handle;
	params->value = BT_GATT_CCC_NOTIFY;
	params->notify = on_bbqovn_measurement_notify;

	atomic_set_bit(params->flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

	err = bt_gatt_subscribe(bbqovn_c->conn, params);
	if (err) {
		atomic_clear_bit(&bbqovn_c->state, BBQOVN_MEASUREMENT_NOTIFY_ENABLED);
		LOG_ERR("Subscribe to BBQOVN characteristic failed");
	} else {
		LOG_DBG("Subscribed to BBQOVN characteristic");
	}

	return err;
}

int bt_bbqovn_client_measurement_unsubscribe(struct bt_bbqovn_client *bbqovn_c)
{
	int err;

	if (!bbqovn_c) {
		return -EINVAL;
	}

	if (!atomic_test_bit(&bbqovn_c->state, BBQOVN_MEASUREMENT_NOTIFY_ENABLED)) {
		return -EFAULT;
	}

	err = bt_gatt_unsubscribe(bbqovn_c->conn, &bbqovn_c->measurement_char.notify_params);
	if (err) {
		LOG_ERR("Unsubscribing from BBQOVN characteristic failed, err: %d",
			err);
	} else {
		atomic_clear_bit(&bbqovn_c->state, BBQOVN_MEASUREMENT_NOTIFY_ENABLED);
		LOG_DBG("Unsubscribed from BBQOVN characteristic");
	}

	return err;
}

int bt_bbqovn_client_handles_assign(struct bt_gatt_dm *dm, struct bt_bbqovn_client *bbqovn_c)
{
	const struct bt_gatt_dm_attr *gatt_service_attr =
			bt_gatt_dm_service_get(dm);
	const struct bt_gatt_service_val *gatt_service =
			bt_gatt_dm_attr_service_val(gatt_service_attr);
	const struct bt_gatt_dm_attr *gatt_chrc;
	const struct bt_gatt_dm_attr *gatt_desc;

	if (!dm || !bbqovn_c) {
		return -EINVAL;
	}

	#if CONFIG_BT_BBQOVN_DX2003_TEST_EN
		if (bt_uuid_cmp(gatt_service->uuid, BT_UUID_DECLARE_16(DX2003_UUID_BASE)))
	#else
		if (bt_uuid_cmp(gatt_service->uuid, BT_UUID_DECLARE_128(BBQOVN_UUID_BASE)))
	#endif
		{
			return -ENOTSUP;
		}
	LOG_DBG("Getting handles from BBQOVN service.");

	bbqovn_reinit(bbqovn_c);

	/* BBQOVN characteristic */
	#if CONFIG_BT_BBQOVN_DX2003_TEST_EN
		gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_DECLARE_16(DX2003_NOTIFY_UUID_BASE));
	#else
		gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_DECLARE_128(BBQOVN_NOTIFY_UUID_BASE));
	#endif
	if (!gatt_chrc) {
		LOG_ERR("No BBQOVN characteristic found.");
		return -EINVAL;
	}

	#if CONFIG_BT_BBQOVN_DX2003_TEST_EN
		gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc,
							BT_UUID_DECLARE_16(DX2003_NOTIFY_UUID_BASE));
	#else
		gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc,
							BT_UUID_DECLARE_128(BBQOVN_NOTIFY_UUID_BASE));
	#endif
	if (!gatt_desc) {
		LOG_ERR("No BBQOVN characteristic value found.");
		return -EINVAL;
	}
	bbqovn_c->measurement_char.handle = gatt_desc->handle;

	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_GATT_CCC);
	if (!gatt_desc) {
		LOG_ERR("No BBQOVN CCC descriptor found.");
		return -EINVAL;
	}

	bbqovn_c->measurement_char.ccc_handle = gatt_desc->handle;

	LOG_DBG("BBQOVN characteristic found");

	/* BBQOVN Write characteristic */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_DECLARE_16(BBQOVN_CHAR_UUID_SENSOR));
	if (!gatt_chrc) {
		LOG_DBG("No BBQOVN Write characteristic found");
	} else {
		gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc,
						    BT_UUID_DECLARE_16(BBQOVN_CHAR_UUID_SENSOR));
		if (!gatt_desc) {
			LOG_ERR("No BBQOVN Write characteristic value found.");
			// return -EINVAL;
		} else {
			bbqovn_c->cp_char.handle = gatt_desc->handle;

			LOG_DBG("BBQOVN Write characteristic found");
		}
	}

	/* Finally - save connection object */
	bbqovn_c->conn = bt_gatt_dm_conn_get(dm);

	return 0;
}

static void on_cp_write(struct bt_conn *conn, uint8_t err,
					struct bt_gatt_write_params *params)
{
	struct bt_bbqovn_client *bbqovn_c;
	bt_bbqovn_client_write_cb write_cb;

	bbqovn_c = CONTAINER_OF(params, struct bt_bbqovn_client, cp_char.write_params);

	write_cb = bbqovn_c->cp_char.write_cb;
	atomic_clear_bit(&bbqovn_c->state, BBQOVN_CONTROL_POINT_WRITE_PENDING);

	if (write_cb) {
		write_cb(bbqovn_c, err);
	}
}

int bt_bbqovn_client_control_point_write(struct bt_bbqovn_client *bbqovn_c,
				      uint8_t *buf, uint8_t len,
				      bt_bbqovn_client_write_cb write_cb)
{
	int err;
	struct bt_gatt_write_params *params;

	if (!bbqovn_c->conn) {
		return -EINVAL;
	}

	if (!bbqovn_c->cp_char.handle) {
		return -ENOTSUP;
	}

	if (atomic_test_and_set_bit(&bbqovn_c->state, BBQOVN_CONTROL_POINT_WRITE_PENDING)) {
		return -EBUSY;
	}

	bbqovn_c->cp_char.write_cb = write_cb;

	params = &bbqovn_c->cp_char.write_params;

	params->data = buf;
	params->length = len;
	params->offset = 0;
	params->handle = bbqovn_c->cp_char.handle;
	params->func = on_cp_write;

	err = bt_gatt_write(bbqovn_c->conn, params);
	if (err) {
		LOG_ERR("Writing BBQOVN characteristic failed, err %d", err);
		atomic_clear_bit(&bbqovn_c->state, BBQOVN_CONTROL_POINT_WRITE_PENDING);
	}

	return err;
}

int bt_bbqovn_client_init(struct bt_bbqovn_client *bbqovn_c)
{
	if (!bbqovn_c) {
		return -EINVAL;
	}

	memset(bbqovn_c, 0, sizeof(*bbqovn_c));
	bt_bbqovn_data_reset(bbqovn_c);

	return 0;
}

int bt_bbqovn_data_reset(struct bt_bbqovn_client *bbqovn_c)
{
	if (!bbqovn_c) {
		return -EINVAL;
	}

	memset(&bbqovn_c->data, 0, sizeof(bbqovn_client_measurement_t));

	bbqovn_c->data.bt.split0 = ':';
	bbqovn_c->data.bt.split1 = ':';
	bbqovn_c->data.bt.end = '#';
	bbqovn_c->data_bak.bt.bbqovn.power = 0xff;
	bbqovn_c->turn = 100;
	bbqovn_c->now_temperature = 0xffff;
	bbqovn_c->target_temperature_bak = 0xffff;
	bbqovn_c->target_temperature = 0xffff;

	return 0;
}
