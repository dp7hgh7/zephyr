/*
 * Copyright (c) 2022 Caspar Friedrich <c.s.w.friedrich@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "w1_ds248x_common.h"

#include <devicetree.h>
#include <drivers/i2c.h>
#include <drivers/w1.h>
#include <logging/log.h>
#include <zephyr.h>

#define DT_DRV_COMPAT maxim_ds2482_800

LOG_MODULE_REGISTER(ds2482_800, CONFIG_W1_LOG_LEVEL);

struct ds2482_800_config {
	struct w1_controller_config w1_config;
	const struct i2c_dt_spec i2c_spec;
	bool apu;
};

struct ds2482_800_data {
	struct w1_controller_data w1_data;
	uint8_t reg_config;
};

static inline const struct ds2482_800_config *get_config(const struct device *dev)
{
	return dev->config;
}

static inline struct ds2482_800_data *get_data(const struct device *dev)
{
	return dev->data;
}

static int ds2482_800_reset_bus(const struct device *dev)
{
	return ds248x_reset_bus(&get_config(dev)->i2c_spec);
}

static int ds2482_800_read_bit(const struct device *dev)
{
	return ds248x_read_bit(&get_config(dev)->i2c_spec);
}

static int ds2482_800_write_bit(const struct device *dev, bool bit)
{
	return ds248x_write_bit(&get_config(dev)->i2c_spec, bit);
}

static int ds2482_800_read_byte(const struct device *dev)
{
	return ds248x_read_byte(&get_config(dev)->i2c_spec);
}

static int ds2482_800_write_byte(const struct device *dev, uint8_t byte)
{
	return ds248x_write_byte(&get_config(dev)->i2c_spec, byte);
}

static int ds2482_800_configure(const struct device *dev, enum w1_driver_settings_type type,
				uint32_t value)
{
	switch (type) {
	case W1_SETTING_SPEED:
		WRITE_BIT(get_data(dev)->reg_config, DEVICE_1WS_pos, value);
		break;
	case W1_SETTING_STRONG_PULLUP:
		WRITE_BIT(get_data(dev)->reg_config, DEVICE_SPU_pos, value);
		break;
	default:
		return -EINVAL;
	}

	return ds248x_write_config(&get_config(dev)->i2c_spec, get_data(dev)->reg_config);
}

static int ds2482_800_init(const struct device *dev)
{
	int ret;

	WRITE_BIT(get_data(dev)->reg_config, DEVICE_APU_pos, get_config(dev)->apu);

	ret = ds248x_reset_device(&get_config(dev)->i2c_spec);
	if (ret) {
		LOG_ERR("Device reset failed: %d", ret);
		return ret;
	}

	ret = ds248x_write_config(&get_config(dev)->i2c_spec, get_data(dev)->reg_config);
	if (ret) {
		LOG_ERR("Device config update failed: %d", ret);
		return ret;
	}

	return 0;
}

static const struct w1_driver_api ds2482_800_driver_api = {
	.reset_bus = ds2482_800_reset_bus,
	.read_bit = ds2482_800_read_bit,
	.write_bit = ds2482_800_write_bit,
	.read_byte = ds2482_800_read_byte,
	.write_byte = ds2482_800_write_byte,
	.configure = ds2482_800_configure,
};

#define DS2482_800_INIT(n)                                                                         \
	static const struct ds2482_800_config inst_##n##_config = {                                \
		.w1_config.client_count = W1_PERIPHERALS_COUNT(DT_DRV_INST(n)),                    \
		.i2c_spec = I2C_DT_SPEC_INST_GET(n),                                               \
		.apu = DT_INST_PROP(n, active_pullup),                                             \
	};                                                                                         \
	static struct ds2482_800_data inst_##n##_data;                                             \
	DEVICE_DT_INST_DEFINE(n, ds2482_800_init, NULL, &inst_##n##_data, &inst_##n##_config,      \
			      POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, &ds2482_800_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DS2482_800_INIT)
