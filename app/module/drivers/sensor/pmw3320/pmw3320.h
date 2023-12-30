/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef ZEPHYR_DRIVERS_SENSOR_PIXART_PMW3320_H_
#define ZEPHYR_DRIVERS_SENSOR_PIXART_PMW3320_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/spi.h>

#define PMW3320_WR_MASK 0x80
#define PMW3320_RD_MASK 0x7F

// #define PMW3320_3389_PID 0x47
// #define PMW3320_3360_PID 0x42
#define PMW3320_REV 0x01

/* General Registers */
#define PMW3320_REG_PID 0x00
#define PMW3320_REG_REV_ID 0x01
#define PMW3320_REG_PWR_UP_RST 0x3A

/* Motion Registers */
#define PMW3320_REG_MOTION 0x02
#define PMW3320_REG_DX 0x03
#define PMW3320_REG_DY 0x04
#define PMW3320_REG_BURST 0x63

/* Motion bits */
#define PMW3320_MOTION (1 << 8)
#define PMW3320_OPMODE_RUN (0)
#define PMW3320_OPMODE_REST1 (0b01 << 1)
#define PMW3320_OPMODE_REST2 (0b10 << 1)
#define PMW3320_OPMODE_REST3 (0b11 << 1)

/* CPI Registers */
#define PMW3320_REG_CPI 0x20

/* power up reset cmd */
#define PMW3320_RESET_CMD 0x5A

/* cpi max and min values */
#define PMW3320_CPI_MIN 250
#define PMW3320_CPI_MAX 3500

struct pmw3320_gpio_dt_spec {
    const struct device *port;
    gpio_pin_t pin;
    gpio_dt_flags_t dt_flags;
};

struct pmw3320_spi_cfg {
    struct spi_config spi_conf;
    struct pmw3320_gpio_dt_spec cs_spec;
};

union pmw3320_bus_cfg {
    struct pmw3320_spi_cfg *spi_cfg;
};

struct pmw3320_config {
    char *bus_name;
    int (*bus_init)(const struct device *dev);
    const union pmw3320_bus_cfg bus_cfg;
    bool disable_rest;
    int cpi;
#if CONFIG_PMW3320_TRIGGER
    struct pmw3320_gpio_dt_spec motswk_spec;
#endif // CONFIG_PMW3320_TRIGGER
};

struct pmw3320_data;

struct pmw3320_transfer_function {
    int (*read_data)(const struct device *dev, int16_t *value);
};

struct pmw3320_data {
    const struct device *bus;
    struct spi_cs_control cs_ctrl;

    int16_t dx;
    int16_t dy;

    const struct pmw3320_transfer_function *hw_tf;

#ifdef CONFIG_PMW3320_TRIGGER

    struct gpio_callback motswk_gpio_cb;
    const struct device *dev;

    sensor_trigger_handler_t handler;
    const struct sensor_trigger *trigger;

#if defined(CONFIG_PMW3320_TRIGGER_OWN_THREAD)
    K_THREAD_STACK_MEMBER(thread_stack, CONFIG_PMW3320_THREAD_STACK_SIZE);
    struct k_sem gpio_sem;
    struct k_thread thread;
#elif defined(CONFIG_PMW3320_TRIGGER_GLOBAL_THREAD)
    struct k_work work;
#endif

#endif /* CONFIG_PMW3320_TRIGGER */
};

int pmw3320_spi_init(const struct device *dev);
#ifdef CONFIG_PMW3320_TRIGGER

int pmw3320_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                        sensor_trigger_handler_t handler);

int pmw3320_init_interrupt(const struct device *dev);

void pmw3320_reset_motion(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_PIXART_PMW3320_H_ */
