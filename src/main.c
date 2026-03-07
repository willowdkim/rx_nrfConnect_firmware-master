/* main.c - Application main entry point */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include "hog.h"
#include <zephyr/kernel.h>

// analog input
#include <zephyr/drivers/adc.h>
#include <nrfx_saadc.h>

bool is_right_board = false;
bool is_connected = false;

int64_t connected_time = 0;
/* GPIO ports */
#define GPIO0_NODE DT_NODELABEL(gpio0)
#define GPIO1_NODE DT_NODELABEL(gpio1)

/* Pins */
#define SIDE_PIN  27   // P0.27
#define LED1_PIN   11   // P1.11
#define LED2_PIN   13   // P1.13
#define LED3_PIN   15   // P1.15

#define DEVICE_ID 67 // name of the pair

// analog input
#define ADC_NODE DT_NODELABEL(adc)

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
int16_t adc_buffer;

#define ADC_CHANNEL_ID 1
#define ADC_RESOLUTION 10

/* Custom LED Service UUID */
#define BT_UUID_LED_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_LED_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

static struct bt_uuid_128 led_service_uuid = BT_UUID_INIT_128(BT_UUID_LED_SERVICE_VAL);
static struct bt_uuid_128 led_char_uuid    = BT_UUID_INIT_128(BT_UUID_LED_CHAR_VAL);

static uint8_t led_value = 0;
static ssize_t led_write_cb(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len,
                            uint16_t offset, uint8_t flags)
{
    if (len != 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    led_value = ((uint8_t *)buf)[0];

    const struct device *gpio1 = DEVICE_DT_GET(GPIO1_NODE);
    
    gpio_pin_configure(gpio1, LED3_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio1, LED2_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio1, LED1_PIN, GPIO_OUTPUT_INACTIVE);
    
    gpio_pin_set(gpio1, LED3_PIN, led_value ? 1 : 0);

    printk("LED set to %d via BLE\n", led_value);

    return len;
}

BT_GATT_SERVICE_DEFINE(led_svc,
    BT_GATT_PRIMARY_SERVICE(&led_service_uuid),
    BT_GATT_CHARACTERISTIC(&led_char_uuid.uuid,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL,
        led_write_cb,
        &led_value),
);


static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
			  BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
			  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

// fix 1
// static const struct bt_data sd[] = {
//     BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
//             sizeof(CONFIG_BT_DEVICE_NAME) - 1),
// };

struct bt_conn *current_conn;

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (%u)\n", err);
        return;
    }

    current_conn = bt_conn_ref(conn);
    is_connected = true;

	bt_conn_set_security(conn, BT_SECURITY_L2);

    printk("Connected\n");
}


static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (current_conn) {
    bt_conn_unref(current_conn);
    current_conn = NULL;
}
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected from %s, reason 0x%02x %s\n", addr,
	       reason, bt_hci_err_to_str(reason));
	is_connected = false;
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %s(%d)\n", addr, level,
		       bt_security_err_to_str(err), err);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};


// static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
// {
// 	char addr[BT_ADDR_LE_STR_LEN];

// 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// 	printk("Passkey for %s: %06u\n", addr, passkey);
// }

// static void auth_cancel(struct bt_conn *conn)
// {
// 	char addr[BT_ADDR_LE_STR_LEN];

// 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// 	printk("Pairing cancelled: %s\n", addr);
// }

// static struct bt_conn_auth_cb auth_cb_display = {
// 	.passkey_display = auth_passkey_display,
// 	.passkey_entry = NULL,
// 	.cancel = auth_cancel,
// };

static struct adc_channel_cfg channel_cfg = {
.gain             = ADC_GAIN_1_6,
.reference        = ADC_REF_INTERNAL,
.acquisition_time = ADC_ACQ_TIME_DEFAULT,
.channel_id       = ADC_CHANNEL_ID,
.input_positive   = NRF_SAADC_INPUT_AIN1, // P0.03
};

struct adc_sequence sequence = {
.channels    = BIT(ADC_CHANNEL_ID),
.buffer      = &adc_buffer,
.buffer_size = sizeof(adc_buffer),
.resolution  = ADC_RESOLUTION,
.oversampling = 4,
};

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    hog_init();

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    /* Set the GAP name BEFORE advertising */
    char bt_name[32];
    snprintf(bt_name, sizeof(bt_name),
            "Circle %s {%d}",
            is_right_board ? "Right" : "Left",
            DEVICE_ID);
    bt_set_name(bt_name);


    /* Start advertising WITHOUT a name */    
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME,
                          ad, ARRAY_SIZE(ad),
                          NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }

    printk("Advertising started as %s\n", bt_get_name());
}


int main(void)
{
	int err;
    const struct device *gpio0 = DEVICE_DT_GET(GPIO0_NODE);
    const struct device *gpio1 = DEVICE_DT_GET(GPIO1_NODE);
    // static bool led_state = false;

    // Check GPIO readiness
    if (!device_is_ready(gpio0) || !device_is_ready(gpio1)) {
        printk("GPIO not ready\n");
        return 0;
    }

	if (!device_is_ready(adc_dev)) {
		printk("ADC not ready\n");
		return 0;
	}
	err = adc_channel_setup(adc_dev, &channel_cfg);
	if (err) {
			printk("adc_channel_setup failed (%d)\n", err);
		return 0;
	}

    // Configure GPIOs set up
    gpio_pin_configure(gpio0, SIDE_PIN, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure(gpio1, LED1_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio1, LED2_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio1, LED3_PIN, GPIO_OUTPUT_INACTIVE);


    // Decide Left / Right ONCE at boot
    int val = gpio_pin_get(gpio0, SIDE_PIN);
    char deviceName[32];

    if (val == 0) {
        snprintf(deviceName, sizeof(deviceName),
                "Circle Right {%d}", DEVICE_ID);
        is_right_board = true;
        gpio_pin_set(gpio1, LED2_PIN, 1);
    } else {
        snprintf(deviceName, sizeof(deviceName),
                "Circle Left {%d}", DEVICE_ID);
        is_right_board = false;
            gpio_pin_set(gpio1, LED3_PIN, 1);
    }
    printk("Device name %s\n", deviceName);

    err = bt_enable(bt_ready);   
    if (err) {
        printk("Bluetooth init failed (%d)\n", err);
        return 0;
    }    
    while (1) {

        if (!is_connected) {
            gpio_pin_toggle(gpio1, LED1_PIN);
            k_sleep(K_MSEC(300));
            continue;
        }

        gpio_pin_set(gpio1, LED1_PIN, 1);

        if (!adc_read(adc_dev, &sequence)) {
            hog_button_loop();
        }

        k_sleep(K_MSEC(20));
    }


    // while (1) {
    //     if (!is_connected) {
    //         while (!is_connected) {
    //             gpio_pin_set(gpio1, LED1_PIN, 0);
    //             k_sleep(K_MSEC(300)); // Sleep for 100ms when disconnected
    //             gpio_pin_set(gpio1, LED1_PIN, 1);
    //             k_sleep(K_MSEC(300)); // Sleep for 100ms when disconnected

    //         }
    //     }
    //     else {
    //             gpio_pin_set(gpio1, LED1_PIN, 1);
    //     }
        
    //     int err = adc_read(adc_dev, &sequence);

    //     if (!err) {
    //         hog_button_loop();  	
    //     }
    //     k_sleep(K_MSEC(10)); // 10ms interval
        
            // no passkey needed
            // if (IS_ENABLED(CONFIG_SAMPLE_BT_USE_AUTHENTICATION)) {
            //     bt_conn_auth_cb_register(&auth_cb_display);
            // }
            // printk("Init done: %s\n", deviceName);
}