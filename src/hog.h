/** @file
 *  @brief HoG Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void hog_init(void);

void hog_button_loop(void);

void hog_reset_detector(void);

void hog_connected(void);
void hog_disconnected(void);

bool hog_is_waiting_for_rising_edge(void);
bool hog_is_collecting_calibration(void);
bool hog_threshold_is_locked(void);

/* Calibrated values, valid once hog_threshold_is_locked() is true. */
int32_t hog_baseline_mv(void);
int32_t hog_on_threshold_mv(void);
int32_t hog_off_threshold_mv(void);

#ifdef __cplusplus
}
#endif


