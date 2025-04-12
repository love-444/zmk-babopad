/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

    struct babopad_data {
        const struct device* dev;
        struct adc_sequence as;

        uint16_t* as_buff;
        int32_t* delta;
        int32_t* prev;
        struct k_work_delayable init_work;
        int async_init_step;
        bool ready;

        struct k_work sampling_work;
        struct k_timer sampling_timer;
        int err;
    };

    struct babopad_config {
        uint32_t sampling_hz;
        size_t adc_channels_size;
        int* adc_channels;
        size_t pwm_channels_size;
        int* pwm_channels;
        int dpi;
        bool x_invert;
        bool y_invert;
        bool xy_swap;
        int evt_type;
        int input_code_x;
        int input_code_y;
    };

#ifdef __cplusplus
}
#endif
