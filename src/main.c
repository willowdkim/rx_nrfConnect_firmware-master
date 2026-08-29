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

/* NCS Nordic UART Service - shows up as the UART console in nRF Connect. */
#include <bluetooth/services/nus.h>

#include "hog.h"

#define DEVICE_SIDE    1
#define DEVICE_NUMBER  1

#define DEVICE_SIDE_LEFT   0
#define DEVICE_SIDE_RIGHT  1

#define GPIO1_NODE DT_NODELABEL(gpio1)

/* LED_A / LED_B / LED_C on the Circle board (Rev 2 status sequence):
 *
 *   1. Searching for Bluetooth   yellow LED flashing, others off
 *   2. Buffer before calibration all three cycling (chase)
 *   3. Calibration in progress   all three blinking together
 *   4. Calibration complete      all three steady on
 */
#define LED_A_PIN 11  /* P1.11 */
#define LED_B_PIN 13  /* P1.13 */
#define LED_C_PIN 15  /* P1.15 */

/* Which pin carries the yellow LED, used alone for state 1. */
#define LED_YELLOW_PIN LED_A_PIN

#define SAMPLE_PERIOD_MS     8
#define LED_BLINK_PERIOD_MS  300  /* states 1 and 3 */
#define LED_CHASE_PERIOD_MS  150  /* state 2, per step - 450 ms per full cycle */

#define ADC_NODE        DT_NODELABEL(adc)
#define ADC_CHANNEL_ID  1  /* P0.03 / AIN1 = VLOG */
#define ADC_RESOLUTION  12   /* 0.88 mV/LSB, vs 3.5 mV at 10-bit */

#define IMU_NODE DT_NODELABEL(lsm6dsox)

/* SAADC scaling: gain 1/6 against the internal 0.6 V reference gives a
 * 0 .. 3.6 V input range.
 *
 * Per the nRF52840 Product Specification the conversion is
 *   RESULT = (V(P) - V(N)) * (GAIN / REFERENCE) * 2^RESOLUTION
 * so full scale maps to 2^RESOLUTION counts - the divisor is 4096, NOT 4095.
 * Identical to raw_adc_to_mv() in hog.c so both agree exactly.
 */
#define ADC_FULL_SCALE_MV 3600
#define ADC_MAX_COUNTS    (1 << ADC_RESOLUTION)

/* How often to push a reading to the nRF Connect UART console, in ms.
 * Deliberately NOT tied to SAMPLE_PERIOD_MS: the detector needs fast
 * sampling, but BLE cannot carry a notification every 8 ms and no human
 * can read 125 lines/sec anyway.
 */
#define NUS_REPORT_MS 50


static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static const struct device *gpio1_dev = DEVICE_DT_GET(GPIO1_NODE);
static const struct device *imu_dev = DEVICE_DT_GET(IMU_NODE);

int16_t adc_buffer;

bool is_right_board = false;
bool is_connected = false;

struct bt_conn *current_conn = NULL;

static char device_name[32];
static bool imu_ready = false;

static struct adc_channel_cfg channel_cfg = {
    .gain             = ADC_GAIN_1_6,
    .reference        = ADC_REF_INTERNAL,
    /* DEFAULT is 10 us, which the SAADC spec rates for a source resistance
     * up to 100 kOhm. If VLOG is not buffered by an op-amp, raise this to
     * ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40) -> 800 kOhm, otherwise the
     * sample-and-hold under-reads by an amount that varies with contact
     * impedance.
     */
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = ADC_CHANNEL_ID,
    .input_positive   = NRF_SAADC_INPUT_AIN1, /* P0.03 / AIN1 = VLOG */
};

static struct adc_sequence sequence = {
    .channels     = BIT(ADC_CHANNEL_ID),
    .buffer       = &adc_buffer,
    .buffer_size  = sizeof(adc_buffer),
    .resolution   = ADC_RESOLUTION,

    /* Zephyr oversampling is an EXPONENT: each sample is averaged from
     * 2^oversampling conversions. 4 -> 16x (previous), 6 -> 64x, which halves
     * random noise. At ~12 us per conversion that is ~0.8 ms per sample, well
     * inside the 8 ms period. The SAADC hardware maximum is 8 (256x); if you
     * also raise acquisition_time to 40 us, 6 costs ~2.7 ms - still fits, but
     * do not go higher without rechecking the timing budget.
     */
    .oversampling = 6,
};

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),

    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
        BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
        BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

/* ------------------------------------------------------------------ */
/* Nordic UART Service (BLE "UART" console in the nRF Connect app)      */
/* ------------------------------------------------------------------ */

/* True once the phone subscribes to NUS TX. Not static: hog.c uses it so the
 * touch detector also runs for a console-only (non-HID) subscriber.
 */
bool nus_ready;

static int32_t raw_adc_to_mv(int16_t raw)
{
    if (raw < 0) {
        raw = 0;
    }

    return ((int32_t)raw * ADC_FULL_SCALE_MV + (ADC_MAX_COUNTS / 2)) /
           ADC_MAX_COUNTS;
}

static void nus_received(struct bt_conn *conn,
                         const uint8_t *const data, uint16_t len)
{
    ARG_UNUSED(conn);

    printk("NUS rx (%u bytes): %.*s\n", len, (int)len, (const char *)data);
}

static void nus_send_enabled(enum bt_nus_send_status status)
{
    nus_ready = (status == BT_NUS_SEND_STATUS_ENABLED);
    printk("NUS notifications %s\n", nus_ready ? "enabled" : "disabled");
}

static struct bt_nus_cb nus_cb = {
    .received     = nus_received,
    .send_enabled = nus_send_enabled,
};

static void nus_report_voltage(int64_t now_ms, int64_t *last_report_ms)
{
    char line[24];
    int  len;
    int  err;

    if (!nus_ready || current_conn == NULL) {
        return;
    }

    if ((now_ms - *last_report_ms) < NUS_REPORT_MS) {
        return;
    }

    *last_report_ms = now_ms;

    /* Max 17 chars, so it fits the default 20-byte ATT notification. */
    len = snprintf(line, sizeof(line), "V=%dmV raw=%d\n",
                   raw_adc_to_mv(adc_buffer), (int)adc_buffer);

    if (len <= 0) {
        return;
    }

    err = bt_nus_send(current_conn, (const uint8_t *)line, (uint16_t)len);
    if (err) {
        /* The link cannot always accept a packet. Count drops and report at
         * most once a second: printing every failure floods RTT and stalls
         * the main loop, which wrecks LED blink and sampling timing.
         */
        static uint32_t dropped;
        static int64_t last_warn_ms;

        dropped++;

        if ((now_ms - last_warn_ms) >= 1000) {
            last_warn_ms = now_ms;
            printk("bt_nus_send: %u drops in last 1s (err %d)\n", dropped, err);
            dropped = 0;
        }
    }
}

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

static const uint8_t led_pins[] = { LED_A_PIN, LED_B_PIN, LED_C_PIN };

static void led_set(uint8_t pin, bool on)
{
    gpio_pin_set(gpio1_dev, pin, on ? 1 : 0);
}

/* All three together. */
static void cal_led_set(bool on)
{
    for (size_t i = 0; i < ARRAY_SIZE(led_pins); i++) {
        led_set(led_pins[i], on);
    }
}

/* Exactly one lit, the rest off - the chase in state 2. */
static void leds_chase_set(uint8_t active)
{
    for (size_t i = 0; i < ARRAY_SIZE(led_pins); i++) {
        led_set(led_pins[i], i == active);
    }
}

static int gpio_init(void)
{
    if (!device_is_ready(gpio1_dev)) {
        printk("GPIO1 not ready\n");
        return -ENODEV;
    }

    for (size_t i = 0; i < ARRAY_SIZE(led_pins); i++) {
        int err = gpio_pin_configure(gpio1_dev, led_pins[i],
                                     GPIO_OUTPUT_INACTIVE);
        if (err) {
            printk("Failed to configure LED on P1.%02u: %d\n",
                   led_pins[i], err);
            return err;
        }
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

    /* Nordic recommends SAADC offset calibration before first use, and again
     * whenever ambient temperature moves more than 10 C. Do it once here
     * rather than on every read: calibration stalls the conversion, and at an
     * 8 ms sample period that would eat the timing budget.
     */
    struct adc_sequence cal_seq = sequence;

    cal_seq.calibrate = true;

    err = adc_read(adc_dev, &cal_seq);
    if (err) {
        /* Non-fatal on purpose: calibration only trims offset error. Losing
         * it costs a little accuracy; treating it as fatal would abort main()
         * and leave the board completely dead.
         */
        printk("SAADC offset calibration failed: %d (continuing)\n", err);
    } else {
        printk("SAADC calibrated (%u-bit, %ux oversampling)\n",
               ADC_RESOLUTION, 1U << sequence.oversampling);
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

    /* Clear thresholds and filters so every session calibrates fresh. */
    hog_connected();

    bt_conn_set_security(conn, BT_SECURITY_L2);

    printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    /* Before dropping current_conn: releases a held key and resets the
     * detector, so threshold_locked clears and the LEDs return to state 1.
     */
    hog_disconnected();

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    is_connected = false;
    nus_ready = false;
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

    err = bt_nus_init(&nus_cb);
    if (err) {
        printk("NUS init failed: %d\n", err);
        return;
    }

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

/*
 * Rev 2 status sequence. Called every loop iteration; owns all three LEDs.
 *
 *   1. !is_connected              yellow flashing, others off
 *   2. connected, not calibrating all three cycling
 *   3. collecting samples         all three blinking together
 *   4. threshold locked           all three steady on
 */
static void update_status_leds(int64_t now_ms)
{
    static int64_t last_step_ms;
    static bool    blink_on;
    static uint8_t chase_index;

    /* 4. Calibration complete - steady on, checked first so it latches. */
    if (hog_threshold_is_locked()) {
        cal_led_set(true);
        return;
    }

    /* 1. Searching for a Bluetooth connection. */
    if (!is_connected) {
        if ((now_ms - last_step_ms) >= LED_BLINK_PERIOD_MS) {
            last_step_ms = now_ms;
            blink_on = !blink_on;
        }

        led_set(LED_B_PIN, false);
        led_set(LED_C_PIN, false);
        led_set(LED_YELLOW_PIN, blink_on);

        chase_index = 0;
        return;
    }

    /* 3. Calibration in progress - all three blink together. */
    if (hog_is_collecting_calibration()) {
        if ((now_ms - last_step_ms) >= LED_BLINK_PERIOD_MS) {
            last_step_ms = now_ms;
            blink_on = !blink_on;
            cal_led_set(blink_on);
        }
        return;
    }

    /* 2. Buffer period before calibration - cycle through the three. */
    if ((now_ms - last_step_ms) >= LED_CHASE_PERIOD_MS) {
        last_step_ms = now_ms;
        leds_chase_set(chase_index);
        chase_index = (chase_index + 1) % ARRAY_SIZE(led_pins);
    }
}

int main(void)
{
    int err;
    int64_t last_nus_ms = 0;

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
            nus_ready = false;
            update_status_leds(now_ms);
            k_sleep(K_MSEC(SAMPLE_PERIOD_MS));
            continue;
        }

        err = adc_read(adc_dev, &sequence);
        if (!err) {
            hog_button_loop();

            /* Independent of hog_button_loop(), which bails out unless
             * HID notifications are subscribed.
             */
            nus_report_voltage(now_ms, &last_nus_ms);
        } else {
            printk("ADC read failed: %d\n", err);
        }

        read_and_print_imu();

        update_status_leds(now_ms);

        k_sleep(K_MSEC(SAMPLE_PERIOD_MS));
    }

    return 0;
}