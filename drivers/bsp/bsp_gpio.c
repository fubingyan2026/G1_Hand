/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bsp_gpio.h"

void bsp_gpio_set_input(GPIO_Type *port, uint32_t port_idx, uint8_t pin_idx)
{
    gpio_set_pin_input(port, port_idx, pin_idx);
}

void bsp_gpio_set_output(GPIO_Type *port, uint32_t port_idx, uint8_t pin_idx)
{
    gpio_set_pin_output(port, port_idx, pin_idx);
}

void bsp_gpio_init(GPIO_Type *port, uint32_t port_idx, uint8_t pin_idx, uint8_t initial_level)
{
    gpio_set_pin_output_with_initial(port, port_idx, pin_idx, initial_level);
}

void bsp_gpio_write(GPIO_Type *port, uint32_t port_idx, uint8_t pin_idx, uint8_t level)
{
    gpio_write_pin(port, port_idx, pin_idx, level);
}

uint8_t bsp_gpio_read(GPIO_Type *port, uint32_t port_idx, uint8_t pin_idx)
{
    return gpio_read_pin(port, port_idx, pin_idx);
}

void bsp_gpio_toggle(GPIO_Type *port, uint32_t port_idx, uint8_t pin_idx)
{
    gpio_toggle_pin(port, port_idx, pin_idx);
}
