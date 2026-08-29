/* main.c - Circle Rx application main entry point */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/settings/settings.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <hal/nrf_saadc.h>

#include "hog.h"

#define DEVICE_SIDE    1
#define DEVICE_NUMBER  1

#define DEVICE_SIDE_LEFT   0
#define DEVICE_SIDE_RIGHT  1

#define GPIO1_NODE DT_NODELABEL(gpio1)

#define CAL_LED_PIN 15   /* P1.15 */

#define SAMPLE_PERIOD_MS     30
#define LED_BLINK_PERIOD_MS  300

#define ADC_NODE        DT_NODELABEL(adc)
#define ADC_CHANNEL_ID  2
#define ADC_RESOLUTION  10

#define IMU_NODE DT_NODELABEL(lsm6dsox)


static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static const struct device *gpio1_dev = DEVICE_DT_GET(GPIO1_NODE);
static const struct device *imu_dev = DEVICE_DT_GET(IMU_NODE);

int16_t adc_buffer;

bool is_right_board = false;
bool is_connected = false;

struct bt_conn *current_conn = NULL;

static char device_name[32];
static bool calibration_led_finished = false;
static bool imu_ready = false;

static struct adc_channel_cfg channel_cfg = {
    .gain             = ADC_GAIN_1_6,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = ADC_CHANNEL_ID,
    .input_positive   = NRF_SAADC_INPUT_AIN2, /* P0.04 / AIN2 = VLOG */
};

static struct adc_sequence sequence = {
    .channels     = BIT(ADC_CHANNEL_ID),
    .buffer       = &adc_buffer,
    .buffer_size  = sizeof(adc_buffer),
    .resolution   = ADC_RESOLUTION,
    .oversampling = 4,
};

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),

    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
        BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
        BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static void build_device_name(void)
{
#if DEVICE_SIDE == DEVICE_SIDE_RIGHT
    is_right_board = true;
    snprintf(device_name, sizeof(device_name),
             "Circle Rx Right %d", DEVICE_NUMBER);
#else
    is_right_board = false;
    snprintf(device_name, sizeof(device_name),
             "Circle Rx Left %d", DEVICE_NUMBER);
#endif
}

static void cal_led_set(bool on)
{
    gpio_pin_set(gpio1_dev, CAL_LED_PIN, on ? 1 : 0);
}

static int gpio_init(void)
{
    if (!device_is_ready(gpio1_dev)) {
        printk("GPIO1 not ready\n");
        return -ENODEV;
    }

    int err = gpio_pin_configure(gpio1_dev, CAL_LED_PIN, GPIO_OUTPUT_INACTIVE);
    if (err) {
        printk("Failed to configure calibration LED: %d\n", err);
        return err;
    }

    cal_led_set(false);
    return 0;
}

static int adc_init(void)
{
    if (!device_is_ready(adc_dev)) {
        printk("ADC not ready\n");
        return -ENODEV;
    }

    int err = adc_channel_setup(adc_dev, &channel_cfg);
    if (err) {
        printk("adc_channel_setup failed: %d\n", err);
        return err;
    }

    return 0;
}

static int imu_init(void)
{
    if (!device_is_ready(imu_dev)) {
        printk("LSM6DSOX IMU not ready\n");
        imu_ready = false;
        return -ENODEV;
    }

    imu_ready = true;
    printk("LSM6DSOX IMU ready\n");
    return 0;
}

static void read_and_print_imu(void)
{
    if (!imu_ready) {
        return;
    }

    struct sensor_value accel[3];
    struct sensor_value gyro[3];

    int err = sensor_sample_fetch(imu_dev);
    if (err) {
        printk("IMU sample fetch failed: %d\n", err);
        return;
    }

    err = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    if (err) {
        printk("IMU accel read failed: %d\n", err);
        return;
    }

    err = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    if (err) {
        printk("IMU gyro read failed: %d\n", err);
        return;
    }

    printk("IMU_ACCEL,%d.%06d,%d.%06d,%d.%06d\n",
           accel[0].val1, accel[0].val2,
           accel[1].val1, accel[1].val2,
           accel[2].val1, accel[2].val2);

    printk("IMU_GYRO,%d.%06d,%d.%06d,%d.%06d\n",
           gyro[0].val1, gyro[0].val2,
           gyro[1].val1, gyro[1].val2,
           gyro[2].val1, gyro[2].val2);
}

static int start_advertising(void)
{
    int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME,
                              ad, ARRAY_SIZE(ad),
                              NULL, 0);

    if (err) {
        printk("Advertising failed to start: %d\n", err);
        return err;
    }

    printk("Advertising started as %s\n", bt_get_name());
    return 0;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed: %u\n", err);
        return;
    }

    current_conn = bt_conn_ref(conn);
    is_connected = true;
    calibration_led_finished = false;

    bt_conn_set_security(conn, BT_SECURITY_L2);

    printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    is_connected = false;
    calibration_led_finished = false;
    cal_led_set(false);

    printk("Disconnected, reason 0x%02x %s\n",
           reason, bt_hci_err_to_str(reason));
}

static void security_changed(struct bt_conn *conn,
                             bt_security_t level,
                             enum bt_security_err err)
{
    if (!err) {
        printk("Security changed: level %u\n", level);
    } else {
        printk("Security failed: level %u err %s(%d)\n",
               level, bt_security_err_to_str(err), err);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .security_changed = security_changed,
};

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed: %d\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    hog_init();

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        err = settings_load();
        if (err) {
            printk("Settings load failed: %d\n", err);
        }
    }

    err = bt_set_name(device_name);
    if (err) {
        printk("bt_set_name failed: %d\n", err);
        return;
    }

    start_advertising();
}

static void update_calibration_led(int64_t now_ms,
                                   int64_t *last_blink_ms,
                                   bool *blink_state)
{
    if (calibration_led_finished) {
        cal_led_set(false);
        return;
    }

    if (hog_is_collecting_calibration()) {
        cal_led_set(true);
        return;
    }

    if (hog_is_waiting_for_rising_edge()) {
        if ((now_ms - *last_blink_ms) >= LED_BLINK_PERIOD_MS) {
            *last_blink_ms = now_ms;
            *blink_state = !(*blink_state);
            cal_led_set(*blink_state);
        }
        return;
    }

    cal_led_set(false);
    calibration_led_finished = true;
}

int main(void)
{
    int err;
    int64_t last_blink_ms = 0;
    bool blink_state = false;

    build_device_name();

    printk("Starting %s\n", device_name);

    err = gpio_init();
    if (err) {
        return 0;
    }

    err = adc_init();
    if (err) {
        return 0;
    }

    err = imu_init();
    if (err) {
        printk("Continuing without IMU\n");
    }

    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed immediately: %d\n", err);
        return 0;
    }

    while (1) {
        int64_t now_ms = k_uptime_get();

        if (!is_connected) {
            cal_led_set(false);
            k_sleep(K_MSEC(SAMPLE_PERIOD_MS));
            continue;
        }

        err = adc_read(adc_dev, &sequence);
        if (!err) {
            hog_button_loop();
        } else {
            printk("ADC read failed: %d\n", err);
        }

        read_and_print_imu();

        update_calibration_led(now_ms, &last_blink_ms, &blink_state);

        k_sleep(K_MSEC(SAMPLE_PERIOD_MS));
    }

    return 0;
}