/** @file
 *  @brief Circle Rx manual HID-over-GATT + adaptive touch detector
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "hog.h"

/*
 * ============================================================
 * External state from main.c
 * ============================================================
 */

extern bool is_right_board;
extern bool is_connected;
extern int16_t adc_buffer;
extern struct bt_conn *current_conn;
extern bool nus_ready;   /* set in main.c when the NUS console subscribes */
extern void nus_send_key_event(bool down);   /* main.c: detector event over NUS */

/*
 * ============================================================
 * HID keycodes
 * ============================================================
 *
 * USB HID keycode:
 *   A = 0x04
 *   D = 0x07
 */

#define HID_KEY_A 0x04
#define HID_KEY_D 0x07

/*
 * ============================================================
 * Final causal setup-then-lock detector constants
 * ============================================================
 *
 * Behavior:
 *
 *   1. Wait for first rising spike
 *   2. Start setup collection at that spike
 *   3. Collect 150 filtered samples
 *   4. Compute thresholds once
 *   5. Lock thresholds for the rest of the round
 *
 * This adapts to different player pairs at the start of a round,
 * but does NOT continuously adapt during a held touch.
 */

/*
 * Sample-count constants below are scaled for SAMPLE_PERIOD_MS = 8.
 * They were originally tuned at 30 ms, so each was multiplied by 30/8 = 3.75
 * to preserve the same behaviour in wall-clock time.
 */
#define SETUP_WINDOW          562  /* 562 * 8 ms = 4.5 s, was 150 * 30 ms */
#define MEDIAN_WINDOW         11   /* was 3, same filter time constant */
#define MEAN_WINDOW           11   /* was 3 */

#define ON_FRACTION_NUM       38
#define OFF_FRACTION_NUM      60
#define FRACTION_DEN          100

#define MIN_ON_OFFSET_MV      8
#define MIN_OFF_OFFSET_MV     4

#define ON_DEBOUNCE_COUNT     8    /* 8 * 8 ms = 64 ms, was 2 * 30 ms = 60 ms */
#define OFF_DEBOUNCE_COUNT    8

/*
 * First rising spike detector.
 *
 * If setup does not start reliably, lower this.
 * If setup starts from noise, raise this.
 */
/*
 * First rising spike detector.
 *
 * The delta is measured over a FIXED TIME WINDOW, not between consecutive
 * samples, so the threshold no longer depends on the sampling rate. The
 * original tuning was 8 mV between samples 30 ms apart; SPIKE_LOOKBACK_SAMPLES
 * reproduces that window at any SAMPLE_PERIOD_MS.
 *
 * Comparing adjacent samples instead would make the threshold scale inversely
 * with sample rate, which at 8 ms drove it down to ~2 mV - close enough to the
 * noise floor that setup triggered instantly on noise every time.
 *
 * If setup does not start reliably, lower START_SPIKE_DELTA_MV.
 * If setup starts from noise, raise it.
 */
#define SPIKE_LOOKBACK_SAMPLES 4   /* 4 * 8 ms = 32 ms, was 1 * 30 ms */
#define SPIKE_LOOKBACK_MS      32  /* keep = SPIKE_LOOKBACK_SAMPLES * SAMPLE_PERIOD_MS */
#define START_SPIKE_DELTA_MV   8   /* mV across that window, as originally tuned */

/*
 * Set to 1 if you want algorithm prints over RTT.
 * Keep 0 for normal use because printing every 30 ms is noisy.
 */
#define DEBUG_ALGORITHM_PRINTS 0

/*
 * ADC conversion:
 * nRF SAADC internal reference = 0.6 V
 * gain = 1/6
 * effective full scale = 3.6 V = 3600 mV
 * 10-bit ADC max count = 1023
 */

#define ADC_RESOLUTION_BITS   12
#define ADC_FULL_SCALE_MV     3600
#define ADC_MAX_COUNTS        (1 << ADC_RESOLUTION_BITS)  /* 4096, not 4095 */

/*
 * ============================================================
 * HID service state
 * ============================================================
 */

static uint8_t notifications_enabled;

static const struct bt_gatt_attr *input_report_attr;

static uint8_t empty_report[8] = {0};

enum {
	HIDS_REMOTE_WAKE = BIT(0),
	HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
	uint16_t version;
	uint8_t code;
	uint8_t flags;
} __packed;

struct hids_report {
	uint8_t id;
	uint8_t type;
} __packed;

enum {
	HIDS_INPUT = 0x01,
	HIDS_OUTPUT = 0x02,
	HIDS_FEATURE = 0x03,
};

static struct hids_info info = {
	.version = 0x0111,
	.code = 0x00,
	.flags = HIDS_REMOTE_WAKE | HIDS_NORMALLY_CONNECTABLE,
};

static struct hids_report input = {
	.id = 0x01,
	.type = HIDS_INPUT,
};

static uint8_t ctrl_point;

/*
 * Keyboard HID report map.
 *
 * Report ID = 1
 * Input report payload = 8 bytes:
 *   byte 0: modifier
 *   byte 1: reserved
 *   byte 2-7: up to 6 keycodes
 */
static uint8_t report_map[] = {
	0x05, 0x01,       /* Usage Page (Generic Desktop) */
	0x09, 0x06,       /* Usage (Keyboard) */
	0xA1, 0x01,       /* Collection (Application) */
	0x85, 0x01,       /* Report ID (1) */

	0x05, 0x07,       /* Usage Page (Keyboard) */
	0x19, 0xE0,       /* Usage Minimum (Left Ctrl) */
	0x29, 0xE7,       /* Usage Maximum (Right GUI) */
	0x15, 0x00,
	0x25, 0x01,
	0x75, 0x01,
	0x95, 0x08,
	0x81, 0x02,       /* Input: modifier byte */

	0x75, 0x08,
	0x95, 0x01,
	0x81, 0x03,       /* Input: reserved byte */

	0x75, 0x08,
	0x95, 0x06,
	0x15, 0x00,
	0x25, 0x65,
	0x05, 0x07,
	0x19, 0x00,
	0x29, 0x65,
	0x81, 0x00,       /* Input: 6 keycode bytes */

	0xC0              /* End Collection */
};

/*
 * ============================================================
 * Adaptive detector state
 * ============================================================
 */

/* Causal median filter state */
static int32_t median_buf[MEDIAN_WINDOW];
static uint8_t median_count;
static uint8_t median_index;

/* Causal mean filter state */
static int32_t mean_buf[MEAN_WINDOW];
static uint8_t mean_count;
static uint8_t mean_index;
static int32_t mean_sum;

/*
 * Setup samples.
 *
 * This is temporary per round.
 * It is cleared/reset in hog_reset_detector().
 * No persistent player data is stored.
 */
static int32_t setup_samples[SETUP_WINDOW];
static uint16_t setup_count;

/* Setup/lock state */
static bool spike_detected;
static bool collecting_setup;
static bool threshold_locked;

/* Rising spike detection */
/* Ring of recent filtered values, so the spike delta spans a fixed time. */
static int32_t spike_hist_mv[SPIKE_LOOKBACK_SAMPLES];
static uint8_t spike_hist_index;
static uint8_t spike_hist_count;

/* Detector values */
static int32_t filtered_mv;
static int32_t baseline_mv;
static int32_t on_threshold_mv;
static int32_t off_threshold_mv;

/* Keyboard state machine */
static bool key_is_pressed;
static uint8_t on_counter;
static uint8_t off_counter;

/*
 * ============================================================
 * HID read/write callbacks
 * ============================================================
 */

static ssize_t read_info(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 void *buf,
			 uint16_t len,
			 uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 attr->user_data,
				 sizeof(struct hids_info));
}

static ssize_t read_report_map(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       void *buf,
			       uint16_t len,
			       uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 report_map,
				 sizeof(report_map));
}

static ssize_t read_report_ref(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       void *buf,
			       uint16_t len,
			       uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 attr->user_data,
				 sizeof(struct hids_report));
}

static ssize_t read_input_report(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 void *buf,
				 uint16_t len,
				 uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 empty_report,
				 sizeof(empty_report));
}

static ssize_t write_ctrl_point(struct bt_conn *conn,
				const struct bt_gatt_attr *attr,
				const void *buf,
				uint16_t len,
				uint16_t offset,
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
	ARG_UNUSED(attr);

	notifications_enabled = (value & BT_GATT_CCC_NOTIFY) != 0;

	printk("HID notifications %s\n",
	       notifications_enabled ? "enabled" : "disabled");
}

/*
 * HID Service Declaration
 */
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
		read_report_ref, NULL, &input),

	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT,
		BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		BT_GATT_PERM_WRITE_ENCRYPT,
		NULL, write_ctrl_point, &ctrl_point),
);

/*
 * ============================================================
 * Utility functions
 * ============================================================
 */

static int compare_i32(const void *a, const void *b)
{
	int32_t x = *(const int32_t *)a;
	int32_t y = *(const int32_t *)b;

	return (x > y) - (x < y);
}

static int32_t raw_adc_to_mv(int16_t raw)
{
	int32_t mv;

	if (raw < 0) {
		raw = 0;
	}

	mv = ((int32_t)raw * ADC_FULL_SCALE_MV + (ADC_MAX_COUNTS / 2)) /
	     ADC_MAX_COUNTS;

	return mv;
}

static int32_t median_filter_update(int32_t value)
{
	int32_t temp[MEDIAN_WINDOW];

	median_buf[median_index] = value;
	median_index = (median_index + 1) % MEDIAN_WINDOW;

	if (median_count < MEDIAN_WINDOW) {
		median_count++;
	}

	memcpy(temp, median_buf, median_count * sizeof(int32_t));
	qsort(temp, median_count, sizeof(int32_t), compare_i32);

	return temp[median_count / 2];
}

static int32_t mean_filter_update(int32_t value)
{
	if (mean_count < MEAN_WINDOW) {
		mean_buf[mean_index] = value;
		mean_sum += value;
		mean_count++;
	} else {
		mean_sum -= mean_buf[mean_index];
		mean_buf[mean_index] = value;
		mean_sum += value;
	}

	mean_index = (mean_index + 1) % MEAN_WINDOW;

	if (mean_count == 0) {
		return value;
	}

	if (mean_sum >= 0) {
		return (mean_sum + (mean_count / 2)) / mean_count;
	}

	return (mean_sum - (mean_count / 2)) / mean_count;
}

static int32_t percentile_from_array(const int32_t *data,
				     uint16_t count,
				     uint8_t percent)
{
	/* static, not a local: SETUP_WINDOW * 4 = 2248 bytes would overflow the
	 * main thread stack. Safe because only the main thread calls this.
	 */
	static int32_t temp[SETUP_WINDOW];
	uint16_t idx;

	if (count == 0) {
		return 0;
	}

	if (count > SETUP_WINDOW) {
		count = SETUP_WINDOW;
	}

	memcpy(temp, data, count * sizeof(int32_t));
	qsort(temp, count, sizeof(int32_t), compare_i32);

	idx = ((count - 1) * percent) / 100;

	return temp[idx];
}

static void compute_and_lock_thresholds(void)
{
	int32_t recent_high_mv;
	int32_t signal_range_mv;
	int32_t on_offset_mv;
	int32_t off_offset_mv;

	baseline_mv = percentile_from_array(setup_samples, setup_count, 10);
	recent_high_mv = percentile_from_array(setup_samples, setup_count, 90);

	signal_range_mv = recent_high_mv - baseline_mv;

	if (signal_range_mv < MIN_ON_OFFSET_MV) {
		signal_range_mv = MIN_ON_OFFSET_MV;
	}

	on_offset_mv = (signal_range_mv * ON_FRACTION_NUM) / FRACTION_DEN;
	off_offset_mv = (signal_range_mv * OFF_FRACTION_NUM) / FRACTION_DEN;

	if (on_offset_mv < MIN_ON_OFFSET_MV) {
		on_offset_mv = MIN_ON_OFFSET_MV;
	}

	if (off_offset_mv < MIN_OFF_OFFSET_MV) {
		off_offset_mv = MIN_OFF_OFFSET_MV;
	}

	if (off_offset_mv >= on_offset_mv) {
		off_offset_mv = (on_offset_mv * 9) / 10;
	}

	on_threshold_mv = baseline_mv + on_offset_mv;
	off_threshold_mv = baseline_mv + off_offset_mv;

	threshold_locked = true;
	collecting_setup = false;

	printk("Thresholds locked: count=%u baseline=%d on=%d off=%d\n",
	       setup_count,
	       baseline_mv,
	       on_threshold_mv,
	       off_threshold_mv);
}

static void send_keyboard_report(uint8_t keycode, bool pressed)
{
	uint8_t report[8] = {0};

	if (!is_connected || !notifications_enabled || current_conn == NULL) {
		return;
	}

	if (pressed) {
		report[2] = keycode;
	}

	bt_gatt_notify(current_conn,
		       input_report_attr,
		       report,
		       sizeof(report));
}

/*
 * ============================================================
 * Public functions
 * ============================================================
 */

void hog_reset_detector(void)
{
	memset(median_buf, 0, sizeof(median_buf));
	median_count = 0;
	median_index = 0;

	memset(mean_buf, 0, sizeof(mean_buf));
	mean_count = 0;
	mean_index = 0;
	mean_sum = 0;

	memset(setup_samples, 0, sizeof(setup_samples));
	setup_count = 0;

	spike_detected = false;
	collecting_setup = false;
	threshold_locked = false;

	memset(spike_hist_mv, 0, sizeof(spike_hist_mv));
	spike_hist_index = 0;
	spike_hist_count = 0;

	filtered_mv = 0;
	baseline_mv = 0;
	on_threshold_mv = 0;
	off_threshold_mv = 0;

	key_is_pressed = false;
	on_counter = 0;
	off_counter = 0;
}

void hog_init(void)
{
	/*
	 * hog_svc.attrs[6] is the HID input report value attribute:
	 *
	 * 0 primary service
	 * 1 characteristic declaration - info
	 * 2 characteristic value       - info
	 * 3 characteristic declaration - report map
	 * 4 characteristic value       - report map
	 * 5 characteristic declaration - input report
	 * 6 characteristic value       - input report
	 */
	input_report_attr = &hog_svc.attrs[6];

	notifications_enabled = 0;

	hog_reset_detector();
}

void hog_connected(void)
{
	notifications_enabled = 0;
	hog_reset_detector();
}

void hog_disconnected(void)
{
	if (key_is_pressed) {
		uint8_t keycode = is_right_board ? HID_KEY_D : HID_KEY_A;

		send_keyboard_report(keycode, false);
	}

	notifications_enabled = 0;
	hog_reset_detector();
}

void hog_force_recalibrate(void)
{
	/* Release a held key first. Resetting the detector alone would clear
	 * key_is_pressed without telling the host, leaving the game device
	 * with a key stuck down forever.
	 */
	if (key_is_pressed) {
		uint8_t keycode = is_right_board ? HID_KEY_D : HID_KEY_A;

		send_keyboard_report(keycode, false);
		nus_send_key_event(false);
	}

	/* Note: notifications_enabled is deliberately NOT cleared. The HID
	 * link to the game device stays up; only the detector restarts.
	 */
	hog_reset_detector();
}

bool hog_key_is_pressed(void)
{
	return key_is_pressed;
}

void hog_button_loop(void)
{
	int32_t voltage_mv;
	int32_t median_filtered_mv;
	int32_t delta_mv;
	uint8_t keycode;

	/*
	 * Run the detector when the host has subscribed to EITHER the HID
	 * input report or the NUS console. The console-only case lets the
	 * calibration sequence run while debugging from nRF Connect, where
	 * the phone is not acting as a keyboard.
	 *
	 * send_keyboard_report() still gates on notifications_enabled, so no
	 * HID traffic is generated for a console-only subscriber.
	 */
	if (!is_connected || current_conn == NULL) {
		return;
	}

	if (!notifications_enabled && !nus_ready) {
		return;
	}

	/*
	 * Convert raw SAADC counts to millivolts so the firmware matches
	 * the algorithm constants.
	 */
	voltage_mv = raw_adc_to_mv(adc_buffer);

	/*
	 * Causal filtering:
	 * raw voltage -> rolling median(3) -> rolling mean(3)
	 */
	median_filtered_mv = median_filter_update(voltage_mv);
	filtered_mv = mean_filter_update(median_filtered_mv);

	keycode = is_right_board ? HID_KEY_D : HID_KEY_A;

	/*
	 * ============================================================
	 * SETUP / CALIBRATION WAITING PHASE
	 * ============================================================
	 *
	 * Wait until the first rising spike.
	 * The first collected setup sample is the spike sample itself.
	 */
	if (!threshold_locked) {
		if (!collecting_setup) {
			/* Fill the lookback ring before any comparison is valid. */
			if (spike_hist_count < SPIKE_LOOKBACK_SAMPLES) {
				spike_hist_mv[spike_hist_index] = filtered_mv;
				spike_hist_index = (spike_hist_index + 1) %
						   SPIKE_LOOKBACK_SAMPLES;
				spike_hist_count++;

#if DEBUG_ALGORITHM_PRINTS
				printk("Waiting for first spike: filt=%d\n", filtered_mv);
#endif
				return;
			}

			/* Oldest sample sits at the write index. */
			delta_mv = filtered_mv - spike_hist_mv[spike_hist_index];

			if (delta_mv >= START_SPIKE_DELTA_MV) {
				spike_detected = true;
				collecting_setup = true;
				setup_count = 0;

				setup_samples[setup_count++] = filtered_mv;

				printk("First rising spike detected: was=%d now=%d delta=%d over %u ms\n",
				       spike_hist_mv[spike_hist_index],
				       filtered_mv,
				       delta_mv,
				       SPIKE_LOOKBACK_MS);
			}

			/* Overwrite the oldest entry and advance. */
			spike_hist_mv[spike_hist_index] = filtered_mv;
			spike_hist_index = (spike_hist_index + 1) %
					   SPIKE_LOOKBACK_SAMPLES;

			return;
		}

		/*
		 * Collect setup samples after the first rising spike.
		 * Thresholds are computed once after SETUP_WINDOW samples.
		 */
		if (setup_count < SETUP_WINDOW) {
			setup_samples[setup_count++] = filtered_mv;
		}

#if DEBUG_ALGORITHM_PRINTS
		printk("Setup collecting: %u/%u filt=%d\n",
		       setup_count,
		       SETUP_WINDOW,
		       filtered_mv);
#endif

		if (setup_count >= SETUP_WINDOW) {
			compute_and_lock_thresholds();
		}

		return;
	}

	/*
	 * ============================================================
	 * GAMEPLAY PHASE
	 * ============================================================
	 *
	 * Thresholds are locked.
	 * Do NOT update thresholds here.
	 *
	 * If signal stays above ON threshold, the key remains held forever.
	 * Key is released only when signal falls below OFF threshold.
	 */

#if DEBUG_ALGORITHM_PRINTS
	printk("mv=%d filt=%d base=%d on=%d off=%d state=%d\n",
	       voltage_mv,
	       filtered_mv,
	       baseline_mv,
	       on_threshold_mv,
	       off_threshold_mv,
	       key_is_pressed ? 1 : 0);
#endif

	if (!key_is_pressed) {
		if (filtered_mv >= on_threshold_mv) {
			on_counter++;
		} else {
			on_counter = 0;
		}

		off_counter = 0;

		if (on_counter >= ON_DEBOUNCE_COUNT) {
			key_is_pressed = true;
			on_counter = 0;
			off_counter = 0;

			send_keyboard_report(keycode, true);

			/* Machine-readable, same clock as the VLOG stream, so
			 * detections and signal share one time axis.
			 */
			nus_send_key_event(true);
			printk("KEY,%u,1\n", (uint32_t)k_uptime_get());
			printk("KEY DOWN: %s\n", is_right_board ? "D" : "A");
		}
	} else {
		if (filtered_mv <= off_threshold_mv) {
			off_counter++;
		} else {
			off_counter = 0;
		}

		on_counter = 0;

		if (off_counter >= OFF_DEBOUNCE_COUNT) {
			key_is_pressed = false;
			on_counter = 0;
			off_counter = 0;

			send_keyboard_report(keycode, false);

			nus_send_key_event(false);
			printk("KEY,%u,0\n", (uint32_t)k_uptime_get());
			printk("KEY UP: %s\n", is_right_board ? "D" : "A");
		}
	}
}

bool hog_is_waiting_for_rising_edge(void)
{
	return is_connected &&
	       current_conn != NULL &&
	       !threshold_locked &&
	       !collecting_setup;
}

bool hog_is_collecting_calibration(void)
{
	return is_connected &&
	       current_conn != NULL &&
	       !threshold_locked &&
	       collecting_setup;
}

bool hog_threshold_is_locked(void)
{
	return threshold_locked;
}

int32_t hog_baseline_mv(void)
{
	return baseline_mv;
}

int32_t hog_on_threshold_mv(void)
{
	return on_threshold_mv;
}

int32_t hog_off_threshold_mv(void)
{
	return off_threshold_mv;
}
