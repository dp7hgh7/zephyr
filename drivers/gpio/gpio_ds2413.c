/*
 * Copyright (c) 2022 Caspar Friedrich <c.s.w.friedrich@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/gpio/gpio_ds2413.h>
#include <drivers/w1.h>
#include <logging/log.h>
#include <zephyr.h>

#include "gpio_utils.h"

#define DT_DRV_COMPAT maxim_ds2413

#define DS2413_FAMILY_CODE 0x3a

LOG_MODULE_REGISTER(ds2413, CONFIG_GPIO_LOG_LEVEL);

struct ds2413_config {
	struct gpio_driver_config common;
};

struct ds2413_data {
	struct gpio_driver_data common;
	struct w1_rom rom;
};

static inline const struct ds2413_config *get_config(const struct device *dev)
{
	return dev->config;
}

static inline struct ds2413_data *get_data(const struct device *dev)
{
	return dev->data;
}

#ifdef CONFIG_GPIO_DRV_CMD
static int ds2413_drv_cmd(const struct device *dev, uint32_t cmd, const void *data, size_t len)
{
	if (cmd != DS2413_CMD_SET_ROM) {
		return -EINVAL;
	}

	if (len != sizeof(struct w1_rom)) {
		return -EINVAL;
	}

	const struct w1_rom *rom = data;

	if (rom->family != DS2413_FAMILY_CODE) {
		LOG_ERR("unexpected family code: 0x%x", rom->family);
		return -EINVAL;
	}

	get_data(dev)->rom = *rom;

	return 0;
}
#endif

static int ds2413_init(const struct device *dev)
{
	return 0;
}

static const struct gpio_driver_api ds2413_driver_api = {
#ifdef CONFIG_GPIO_DRV_CMD
	.drv_cmd = ds2413_drv_cmd,
#endif
};

#define DS2413_INIT(n)                                                                             \
	static const struct ds2413_config inst_##n##_config = {                                    \
		.common = { .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n) }                  \
	};                                                                                         \
	static struct ds2413_data inst_##n##_data;                                                 \
	DEVICE_DT_INST_DEFINE(n, ds2413_init, NULL, &inst_##n##_data, &inst_##n##_config,          \
			      POST_KERNEL, CONFIG_W1_INIT_PRIORITY, &ds2413_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DS2413_INIT)
