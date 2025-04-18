/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_babopad

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
//#include <zmk/keymap.h>
#include <stdlib.h> //for abs()
#include <zephyr/sys/util.h> // for CLAMP
#include <haly/nrfy_saadc.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(BABOPAD, 3);

#include "babopad.h"

#define ADC_NODE DT_ALIAS(adc0)
static const struct device* adc = DEVICE_DT_GET(ADC_NODE);
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
uint16_t adc_reading[4][8];
uint16_t map[8][8];
static const struct adc_sequence_options options = {
    .extra_samplings = 4 - 1,
    .interval_us = 0,
};
static struct adc_sequence sequence = {
    .buffer = adc_reading,
    .buffer_size = sizeof(adc_reading),
    .resolution = 12,
    .options = &options,
};



static int babopad_report_data(const struct device *dev) {
    struct babopad_data *data = dev->data;
    const struct babopad_config *config = dev->config;
    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    for (size_t c = 0; c < config->pwm_channels_size; c++)
    {
        int err = adc_read(adc, &sequence);        
        for (size_t r = 0; r < config->adc_channels_size; r++)
        {
            map[c][r] = 0;
            for (size_t i = 0; i < 4; i++)
            {
                map[c][r] += adc_reading[i][r];
            }
            LOG_DBG("%d ", map[c][r]);
        }
        LOG_DBG("\n");
    }



    //input_report(dev, config->evt_type, config->input_code_x, 12, true, K_NO_WAIT);
    //gpio_pin_set_dt(&led, map[0][0] > 0 ? 1 : 0);

//    struct adc_sequence* as = &data->as;
//
//    for (uint8_t i = 0; i < config->io_channels_len; i++) {
//        struct analog_input_io_channel ch_cfg = (struct analog_input_io_channel)config->io_channels[i];
//        const struct device* adc = ch_cfg.adc_channel.dev;
//
//        if (i == 0) {
//            int err = adc_read(adc, as);
//            if (err < 0) {
//                LOG_ERR("AIN%u read returned %d", i, err);
//                return err;
//            }
//        }
//
//        int32_t raw = data->as_buff[i];
//        int32_t mv = raw;
//        adc_raw_to_millivolts(adc_ref_internal(adc), ADC_GAIN_1_6, as->resolution, &mv);
//
//        int16_t v = mv - ch_cfg.mv_mid;
//        int16_t dz = ch_cfg.mv_deadzone;
//        if (dz) {
//            if (v > 0) {
//                if (v < dz) v = 0; else v -= dz;
//            }
//            if (v < 0) {
//                if (v > -dz) v = 0; else v += dz;
//            }
//        }
//        uint16_t mm = ch_cfg.mv_min_max;
//        if (mm) {
//            if (v > 0 && v > mm) v = mm;
//            if (v < 0 && v < -mm) v = -mm;
//        }
//
//        if (ch_cfg.invert) v *= -1;
//        v = (int16_t)((v * ch_cfg.scale_multiplier) / ch_cfg.scale_divisor);
//
//        if (ch_cfg.report_on_change_only) {
//            // track raw value to compare until next report interval
//            data->delta[i] = v;
//        }
//        else {
//            // accumulate delta until report in next iteration
//            int32_t delta = data->delta[i];
//            int32_t dv = delta + v;
//            data->delta[i] = dv;
//        }
//    }
//
//    // First read is setup as calibration
//    as->calibrate = false;
//
//    if (!data->actived) {
//        return 0;
//    }
//
//    int8_t idx_to_sync = -1;
//    for (uint8_t i = config->io_channels_len - 1; i >= 0; i--) {
//        int32_t dv = data->delta[i];
//        int32_t pv = data->prev[i];
//        if (dv != pv) {
//            idx_to_sync = i;
//            break;
//        }
//    }
//
//    for (uint8_t i = 0; i < config->io_channels_len; i++) {
//        struct analog_input_io_channel ch_cfg = (struct analog_input_io_channel)config->io_channels[i];
//        // LOG_DBG("AIN%u get delta AGAIN", i);
//        int32_t dv = data->delta[i];
//        int32_t pv = data->prev[i];
//        if (dv != pv) {
//            data->delta[i] = 0;
//            if (ch_cfg.report_on_change_only) {
//                data->prev[i] = dv;
//            }
//
//            
//        }
//    }
    return 0;
}

static struct k_work_q babopad_work_q;
K_THREAD_STACK_DEFINE(babopad_q_stack, 1024);

static void sampling_work_handler(struct k_work *work) {
    struct babopad_data *data = CONTAINER_OF(work, struct babopad_data, sampling_work);
    // LOG_DBG("sampling work triggered");
    babopad_report_data(data->dev);
}

static void sampling_timer_handler(struct k_timer *timer) {
    struct babopad_data *data = CONTAINER_OF(timer, struct babopad_data, sampling_timer);
    // LOG_DBG("sampling timer triggered");
    k_work_submit_to_queue(&babopad_work_q, &data->sampling_work);
    k_work_submit(&data->sampling_work);
}

static void babopad_async_init(struct k_work *work) {
    struct k_work_delayable *work_delayable = (struct k_work_delayable *)work;
    struct babopad_data *data = CONTAINER_OF(work_delayable, 
                                                  struct babopad_data, init_work);
    const struct device *dev = data->dev;
    const struct babopad_config *config = dev->config;

    LOG_DBG("babopad async init");

    if (!device_is_ready(adc)) {
        return;
    };


    input_report(dev, config->evt_type, config->input_code_x, 100, false, K_FOREVER);

    for (size_t i = 0; i < config->adc_channels_size; i++)
    {
        struct adc_channel_cfg _pl = {};
        nrf_saadc_channel_config_t cfg = {
            .resistor_p = NRF_SAADC_RESISTOR_PULLDOWN,
            .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
            .gain = NRF_SAADC_GAIN4,
            .reference = NRF_SAADC_REFERENCE_INTERNAL,
            .acq_time = NRF_SAADC_ACQTIME_10US,
            .mode = NRF_SAADC_MODE_SINGLE_ENDED,
            .burst = NRF_SAADC_BURST_DISABLED,
        };
        sequence.channels |= BIT(config->adc_channels[i]);
        adc_channel_setup(dev, &_pl);
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led, 1);
    }
    // init pwm



    data->ready = true;


    k_work_init(&data->sampling_work, sampling_work_handler);
    k_work_queue_start(&babopad_work_q,
                        babopad_q_stack, K_THREAD_STACK_SIZEOF(babopad_q_stack),
                        10, NULL);

    k_timer_init(&data->sampling_timer, sampling_timer_handler, NULL);
    k_timer_start(&data->sampling_timer, K_NO_WAIT, K_MSEC(1));
}

static int babopad_init(const struct device *dev) {
    struct babopad_data *data = dev->data;
    const struct babopad_config *config = dev->config;
    int err = 0;

    data->dev = dev;
    k_work_init_delayable(&data->init_work, babopad_async_init);
    k_work_schedule(&data->init_work, K_MSEC(1));
    return err;
}

#define BABOPAD_DEFINE(n)                                                                          \
    static struct babopad_data data##n = {                                                         \
    };                                                                                             \
    static int32_t adc_channels##n[] = DT_PROP(DT_DRV_INST(n), adc_channels);                      \
    static int32_t pwm_channels##n[] = DT_PROP(DT_DRV_INST(n), pwm_channels);                      \
    static const struct babopad_config config##n = {                                               \
        .sampling_hz = DT_PROP(DT_DRV_INST(n), sampling_hz),                                       \
        .adc_channels_size = DT_PROP_LEN(DT_DRV_INST(n), adc_channels),                            \
        .adc_channels = adc_channels##n,                                                           \
        .pwm_channels_size = DT_PROP_LEN(DT_DRV_INST(n), pwm_channels),                            \
        .pwm_channels = pwm_channels##n,                                                           \
        .dpi = DT_PROP(DT_DRV_INST(n), dpi),                                                       \
        .x_invert = DT_PROP(DT_DRV_INST(n), x_invert),                                             \
        .y_invert = DT_PROP(DT_DRV_INST(n), y_invert),                                             \
        .xy_swap = DT_PROP(DT_DRV_INST(n), xy_swap),                                               \
        .evt_type = DT_PROP(DT_DRV_INST(n), evt_type),                                             \
        .input_code_x = DT_PROP(DT_DRV_INST(n), input_code_x),                                     \
        .input_code_y = DT_PROP(DT_DRV_INST(n), input_code_y),                                     \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, babopad_init, NULL, &data##n, &config##n, POST_KERNEL,                \
                          CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(BABOPAD_DEFINE)