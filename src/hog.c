/** @file
 *  @brief HoG Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

extern int64_t connected_time;

#define HID_KEY_A 0x04
#define HID_KEY_D 0x07

#define GPIO0_NODE DT_NODELABEL(gpio0)
extern bool is_right_board;
extern bool is_connected;
static uint8_t simulate_input;

static const struct bt_gatt_attr *input_report_attr;

static uint8_t empty_report[8] = {0};

// analog input
#include <zephyr/drivers/adc.h>
extern int16_t adc_buffer;
extern struct bt_conn *current_conn;
#define ADC_CHANNEL_ID 1
#define ADC_RESOLUTION 10

enum {
	HIDS_REMOTE_WAKE = BIT(0),
	HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
	uint16_t version; /* version number of base USB HID Specification */
	uint8_t code; /* country HID Device hardware is localized for. */
	uint8_t flags;
} __packed;

struct hids_report {
	uint8_t id; /* report id */
	uint8_t type; /* report type */
} __packed;

static struct hids_info info = {
	.version = 0x0000,
	.code = 0x00,
	.flags = HIDS_NORMALLY_CONNECTABLE,
};

enum {
	HIDS_INPUT = 0x01,
	HIDS_OUTPUT = 0x02,
	HIDS_FEATURE = 0x03,
};

static struct hids_report input = {
	.id = 0x01,
	.type = HIDS_INPUT,
};

static uint8_t ctrl_point;
static uint8_t report_map[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)

    0x05, 0x07,       //   Usage Page (Keyboard)
    0x19, 0xE0,       //   Usage Minimum (Left Ctrl)
    0x29, 0xE7,       //   Usage Maximum (Right GUI)
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x08,
    0x81, 0x02,       //   Input (Data,Var,Abs) Modifier byte

    0x75, 0x08,
    0x95, 0x01,
    0x81, 0x03,       //   Input (Const) Reserved

    0x75, 0x08,
    0x95, 0x06,
    0x15, 0x00,
    0x25, 0x65,
    0x05, 0x07,
    0x19, 0x00,
    0x29, 0x65,
    0x81, 0x00,       //   Input (Data,Array) Keycodes

    0xC0              // End Collection
};

static ssize_t read_info(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr, void *buf,
			  uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
				 sizeof(struct hids_info));
}

static ssize_t read_report_map(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, report_map,
				 sizeof(report_map));
}

static ssize_t read_report(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr, void *buf,
			   uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
				 sizeof(struct hids_report));
}

static ssize_t read_input_report(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr, void *buf,
				 uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, empty_report, sizeof(empty_report));
}

static ssize_t write_ctrl_point(struct bt_conn *conn,
				const struct bt_gatt_attr *attr,
				const void *buf, uint16_t len, uint16_t offset,
				uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(ctrl_point)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}
static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    simulate_input = (value & BT_GATT_CCC_NOTIFY) != 0;
    printk("CCC changed: value=0x%04x simulate_input=%d\n", value, simulate_input);
}

// /* Require encryption. */
#define SAMPLE_BT_PERM_READ BT_GATT_PERM_READ_ENCRYPT
#define SAMPLE_BT_PERM_WRITE BT_GATT_PERM_WRITE_ENCRYPT

/* HID Service Declaration */
BT_GATT_SERVICE_DEFINE(hog_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_info, NULL, &info),

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_report_map, NULL, NULL),

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        read_input_report, NULL, NULL),

    BT_GATT_CCC(input_ccc_changed,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF,
        BT_GATT_PERM_READ,
        read_report, NULL, &input),

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT,
        BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE_ENCRYPT,
        NULL, write_ctrl_point, &ctrl_point),
);


void hog_init(void)
{
	input_report_attr = &hog_svc.attrs[6];
}

#define SW0_NODE DT_ALIAS(sw0)

static void send_keyboard_report(uint8_t keycode, bool pressed)
{
    if (!is_connected || !simulate_input || !current_conn) {
        return;
    }

    uint8_t report[8] = {0};

    if (pressed) {
        report[2] = keycode;  // HID keycode
    }

    bt_gatt_notify(current_conn, input_report_attr,
                   report, sizeof(report));
}

#define FILTER_SHIFT   3   // 1/8  smoothing (~200ms)a
#define BASE_SHIFT     9   // 1/512 baseline (~15s)
#define SLOPE_TH      2   // slope trigger threshold
#define DEADZONE       2   // ignore tiny noise

void hog_button_loop(void)
{
    if (!simulate_input || !is_connected || !current_conn) {
        return;
    }

    static bool key_is_pressed = false;

    // filter state
    static bool initialized = false;
    static int32_t filtered;
    static int32_t baseline;
    static int32_t prev_signal;

    int raw = adc_buffer;


    if (!initialized) {
    filtered = raw;
    baseline = raw;
    prev_signal = 0;
    initialized = true;
    return;   // skip first cycle
    }

    // LOW-PASS FILTER (remove fast ADC noise)
    filtered += (raw - filtered) >> FILTER_SHIFT; // shifting by 3 bit so divided by 8

    // BASELINE TRACK (remove slow drift / skin diff)
    baseline += (filtered - baseline) >> BASE_SHIFT; // divided by 512

    // CENTERED SIGNAL (user independent)
    int signal = filtered - baseline;

    // DERIVATIVE (detect rising/falling edges)
    int slope = signal - prev_signal;
    prev_signal = signal;

    uint8_t keycode = is_right_board ? HID_KEY_D : HID_KEY_A;

    printk("raw=%d filt=%d base=%d sig=%d slope=%d\n",
           raw, filtered, baseline, signal, slope);

    // small noise guard
    if (slope > -DEADZONE && slope < DEADZONE) {
        return;
    }

    // edge
    if (slope > SLOPE_TH && !key_is_pressed) {
        send_keyboard_report(keycode, true);   // press
        key_is_pressed = true;
    }

    if (slope < -SLOPE_TH && key_is_pressed) {
        send_keyboard_report(0, false);        // release
        key_is_pressed = false;
    }
}

// #define ADC_THRESHOLD 295   // tune this
// #define DEBOUNCE_MS   100

// void hog_button_loop(void)
// {
//     if (!simulate_input || !is_connected || !current_conn) {
//     return;
// 	}
	
// 	static bool key_is_pressed = false;

//     bool above = adc_buffer >= ADC_THRESHOLD;
//     uint8_t keycode = is_right_board ? HID_KEY_D : HID_KEY_A;

// 	printk("adc=%d threshold=%d\n", adc_buffer, ADC_THRESHOLD);

// 	printk("DBG right=%d adc=%d above=%d sim=%d conn=%d\n",
//        is_right_board,
//        adc_buffer,
//        adc_buffer >= ADC_THRESHOLD,
//        simulate_input,
//        current_conn != NULL);

//     /* Signal just went HIGH → press key */
//     if (above && !key_is_pressed) {
//         send_keyboard_report(keycode, true);   // key down
//         key_is_pressed = true;
//     }

//     /* Signal just went LOW → release key */
//     if (!above && key_is_pressed) {
//         send_keyboard_report(0, false);        // key up
//         key_is_pressed = false;
//     }
// }