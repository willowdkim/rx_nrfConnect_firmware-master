/* main.c - Circle Rx application main entry point */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
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

/* ---------------- Rev2 / Rev3 comparison instrumentation ----------------
 *
 * TWO capture paths, and they are NOT equivalent measurements:
 *
 *   BLE (nus_stream_sample)  battery powered, floating ground. This is the
 *                            representative condition and the primary dataset.
 *   RTT (VLOG_RTT_STREAM)    requires J-Link attached, which ties board ground
 *                            to the PC and hence to mains earth. In a
 *                            body-coupled system the body-to-earth return path
 *                            IS the signal path, so the probe changes the
 *                            measurement. Use for debugging, and for
 *                            quantifying the grounding effect by capturing
 *                            both ways - not as the sole dataset.
 *
 * RTT line format:  VLOG,<uptime_ms>,<raw_counts>
 *
 * Raw counts, not millivolts: conversion is lossy and belongs in analysis,
 * where it can use a per-board calibration factor.
 */
#define VLOG_RTT_STREAM 1


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

/* Last detector state announced over NUS; 0xFF forces a re-send. */
uint8_t nus_last_state = 0xFFU;

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

    /* Force the loop to re-announce detector state to the new subscriber. */
    extern uint8_t nus_last_state;
    nus_last_state = 0xFFU;
}

static struct bt_nus_cb nus_cb = {
    .received     = nus_received,
    .send_enabled = nus_send_enabled,
};

/* Batched binary stream: 6 samples per notification, 18 bytes, which fits the
 * default 20-byte ATT payload with no MTU negotiation required. 125 Hz / 6 =
 * ~21 packets/s, comfortably inside what iOS grants at a 30 ms connection
 * interval - so the wireless capture runs at the FULL sample rate.
 *
 * This matters because RTT is not a neutral observer: the J-Link ties board
 * ground to the PC and therefore to mains earth, and in a body-coupled system
 * the body-to-earth return path IS the signal path. Battery-powered BLE is the
 * representative measurement condition; RTT is the perturbed one.
 *
 * Packet layout (little endian):
 *   [0..3] uint32 uptime_ms of the FIRST sample in the batch
 *   [4..5] uint16 packet sequence, wraps at 65536
 *   [6..]  6 x uint16 raw SAADC counts, spaced SAMPLE_PERIOD_MS apart
 */
#define NUS_BATCH_SAMPLES 6
#define NUS_PACKET_BYTES  (6 + 2 * NUS_BATCH_SAMPLES)

static void nus_stream_sample(int64_t now_ms, int16_t raw)
{
    static uint16_t batch[NUS_BATCH_SAMPLES];
    static uint8_t  batch_n;
    static uint32_t batch_t0;
    static uint16_t batch_seq;
    static uint32_t dropped;
    static int64_t  last_warn_ms;

    if (!nus_ready || current_conn == NULL) {
        batch_n = 0;
        return;
    }

    if (batch_n == 0) {
        batch_t0 = (uint32_t)now_ms;
    }

    batch[batch_n++] = (uint16_t)(raw < 0 ? 0 : raw);

    if (batch_n < NUS_BATCH_SAMPLES) {
        return;
    }

    uint8_t pkt[NUS_PACKET_BYTES];

    sys_put_le32(batch_t0, &pkt[0]);
    sys_put_le16(batch_seq++, &pkt[4]);
    for (uint8_t i = 0; i < NUS_BATCH_SAMPLES; i++) {
        sys_put_le16(batch[i], &pkt[6 + 2 * i]);
    }
    batch_n = 0;

    int err = bt_nus_send(current_conn, pkt, sizeof(pkt));
    if (err) {
        /* Count drops, report once a second. Printing every failure floods RTT
         * and stalls the loop, which corrupts the sample timing.
         */
        dropped++;
        if ((now_ms - last_warn_ms) >= 1000) {
            last_warn_ms = now_ms;
            printk("bt_nus_send: %u packets dropped in last 1s (err %d)\n",
                   dropped, err);
            dropped = 0;
        }
    }
}

/* Detector decisions over the same NUS stream, so a battery-powered capture
 * carries them without needing HID pairing on the host.
 *
 * 6 bytes, distinguishable from the 18-byte sample packet by length:
 *   [0..3] uint32 uptime_ms
 *   [4]    0x4B ('K')
 *   [5]    1 = key down, 0 = key up
 */
/* Detector state, so the host knows when calibration is finished instead of
 * relying on someone watching the LEDs:
 *   0 = waiting for the first rising spike
 *   1 = collecting calibration samples
 *   2 = threshold locked, detector armed
 */
#define NUS_TAG_KEY   0x4B
#define NUS_TAG_STATE 0x53

void nus_send_state(uint8_t state)
{
    if (!nus_ready || current_conn == NULL) {
        return;
    }

    /* 12 bytes. The thresholds only exist on the board, so they have to ride
     * along with the state or the host can never display them:
     *   [0..3]  uint32 uptime_ms
     *   [4]     0x53 'S'
     *   [5]     state
     *   [6..7]  int16 baseline mV      (0 until locked)
     *   [8..9]  int16 on-threshold mV
     *   [10..11] int16 off-threshold mV
     */
    uint8_t pkt[12];

    sys_put_le32((uint32_t)k_uptime_get(), &pkt[0]);
    pkt[4] = NUS_TAG_STATE;
    pkt[5] = state;
    sys_put_le16((uint16_t)(int16_t)hog_baseline_mv(), &pkt[6]);
    sys_put_le16((uint16_t)(int16_t)hog_on_threshold_mv(), &pkt[8]);
    sys_put_le16((uint16_t)(int16_t)hog_off_threshold_mv(), &pkt[10]);

    (void)bt_nus_send(current_conn, pkt, sizeof(pkt));
}

void nus_send_key_event(bool down)
{
    if (!nus_ready || current_conn == NULL) {
        return;
    }

    uint8_t pkt[6];

    sys_put_le32((uint32_t)k_uptime_get(), &pkt[0]);
    pkt[4] = NUS_TAG_KEY;
    pkt[5] = down ? 1U : 0U;

    (void)bt_nus_send(current_conn, pkt, sizeof(pkt));
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
#if VLOG_RTT_STREAM
            /* One line per sample, before any filtering. */
            printk("VLOG,%u,%d\n", (uint32_t)now_ms, (int)adc_buffer);
#endif

            hog_button_loop();

            /* Independent of hog_button_loop(), which bails out unless
             * HID notifications are subscribed.
             */
            nus_stream_sample(now_ms, adc_buffer);
        } else {
            printk("ADC read failed: %d\n", err);
        }

        read_and_print_imu();

        update_status_leds(now_ms);

        /* Announce detector state transitions to the host. */
        uint8_t st = hog_threshold_is_locked() ? 2U
                   : hog_is_collecting_calibration() ? 1U : 0U;

        if (st != nus_last_state) {
            nus_last_state = st;
            nus_send_state(st);
            printk("STATE,%u,%u\n", (uint32_t)now_ms, st);
        }

        k_sleep(K_MSEC(SAMPLE_PERIOD_MS));
    }

    return 0;
}