/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_babopad
#define PWM_PERIOD_4MHZ PWM_KHZ(4000)

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
//#include <zmk/keymap.h>
#include <stdlib.h> //for abs()
#include <zephyr/sys/util.h> // for CLAMP
#include <haly/nrfy_saadc.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
LOG_MODULE_REGISTER(BABOPAD, CONFIG_ZMK_LOG_LEVEL);

#include "babopad.h"

#define ADC_NODE DT_ALIAS(adc0)
static const struct device* adc = DEVICE_DT_GET(ADC_NODE);
#define PWM0_NODE DT_ALIAS(pwm00)
#define PWM1_NODE DT_ALIAS(pwm01)
#define PWM2_NODE DT_ALIAS(pwm02)
static const struct pwm_dt_spec pwm[3] = { PWM_DT_SPEC_GET(PWM0_NODE), PWM_DT_SPEC_GET(PWM1_NODE), PWM_DT_SPEC_GET(PWM2_NODE), };
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
int16_t adc_reading[3][16];
int16_t map[3][3];
static const struct adc_sequence_options options = {
    .extra_samplings = 16 - 1,
    .interval_us = 0,
};
static struct adc_sequence sequence = {
    .buffer = &adc_reading[0][0],
    .buffer_size = sizeof(adc_reading) / 3,
    .resolution = 12,
    .options = &options,
};

static int a = 46;

static int babopad_report_data(const struct device *dev) {
    struct babopad_data *data = dev->data;
    const struct babopad_config *config = dev->config;

    for (size_t c = 0; c < config->pwm_channels_size; c++)
    {
        pwm_set_dt(&pwm[c], PWM_PERIOD_4MHZ, PWM_PERIOD_4MHZ / 2U);
        for (size_t r = 0; r < config->adc_channels_size; r++)
        {
            sequence.buffer = &adc_reading[r][0];
            sequence.channels = BIT(config->adc_channels[r]);
            int err = adc_read(adc, &sequence);
            map[c][r] = 0;
            for (size_t i = 0; i < 16; i++)
            {
                map[c][r] += adc_reading[r][i];

            }
            map[c][r] >>= 4;
        }
        pwm_set_dt(&pwm[c], PWM_PERIOD_4MHZ, 0);
    }

    //analyse coord:
    //1. get x-mean, y-mean, total value
    int x = map[2][0] + map[2][1] + map[2][2] - map[0][0] - map[0][1] - map[0][2];
    int y = map[0][2] + map[1][2] + map[2][2] - map[0][0] - map[1][0] - map[2][0];
    int total = map[0][0] + map[1][0] + map[2][0] + map[0][1] + map[1][1] + map[2][1] + map[0][2] + map[1][2] + map[2][2];
    if (total <= 2000) return 0;
    x = 128 * x / total;
    y = 128 * y / total;
    LOG_DBG("%d %d %d", x, y, total);

    //2. filter fluctuation
    //3. filter with value threshold
    //4. move cursor
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
    for (int i = 0; i < 3; i++)
    if (!pwm_is_ready_dt(&pwm[i])) {
        return;
    }

    //init adc
    for (size_t i = 0; i < config->adc_channels_size; i++)
    {
        struct adc_channel_cfg _pl =
        {
            .gain = ADC_GAIN_1,
            .reference = ADC_REF_INTERNAL,
            .acquisition_time = ADC_ACQ_TIME_DEFAULT,
            .channel_id = config->adc_channels[i],
            .differential = 0,
            .input_positive = config->adc_channels[i] + SAADC_CH_PSELP_PSELP_AnalogInput0,
        };
        adc_channel_setup(adc, &_pl);
    }
    // init pwm
    pwm_set_dt(&pwm[0], PWM_PERIOD_4MHZ, 0);
    pwm_set_dt(&pwm[1], PWM_PERIOD_4MHZ, 0);
    pwm_set_dt(&pwm[2], PWM_PERIOD_4MHZ, 0);

    data->ready = true;

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led, 0);

    k_work_init(&data->sampling_work, sampling_work_handler);
    k_work_queue_start(&babopad_work_q, babopad_q_stack, K_THREAD_STACK_SIZEOF(babopad_q_stack), 50, NULL);
    k_timer_init(&data->sampling_timer, sampling_timer_handler, NULL);
    k_timer_start(&data->sampling_timer, K_MSEC(10), K_MSEC(10));
}

static int babopad_init(const struct device *dev) {
    struct babopad_data *data = dev->data;
    //const struct babopad_config *config = dev->config;
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