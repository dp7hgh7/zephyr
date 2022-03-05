/*
 * Copyright (c) 2022 Caspar Friedrich <c.s.w.friedrich@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "w1_ds248x_common.h"

#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/w1.h>
#include <logging/log.h>
#include <zephyr.h>

LOG_MODULE_REGISTER(ds248x_common, CONFIG_W1_LOG_LEVEL);

int ds248x_write(const struct i2c_dt_spec *i2c_spec, uint8_t cmd, uint8_t *data)
{
	int ret;

	const uint8_t tmp[] = { cmd, data ? *data : 0 };

	if (!device_is_ready(i2c_spec->bus)) {
		return -EINVAL;
	}

	ret = i2c_write_dt(i2c_spec, tmp, data ? 2 : 1);
	if (ret) {
		return ret;
	}

	return 0;
}

int ds248x_read(const struct i2c_dt_spec *i2c_spec, uint8_t rp, uint8_t *reg)
{
	int ret;

	switch (rp) {
	case REG_NONE:
		break;
	case REG_PORT:
	case REG_DEVICE:
	case REG_CHANNEL:
	case REG_DATA:
	case REG_STATUS:
		ret = ds248x_write(i2c_spec, CMD_SRP, &rp);
		if (ret) {
			return ret;
		}
		break;
	default:
		LOG_ERR("Unknown read pointer");
		return -EINVAL;
	}

	if (!device_is_ready(i2c_spec->bus)) {
		LOG_ERR("Bus not ready");
		return -EINVAL;
	}

	ret = i2c_read_dt(i2c_spec, reg, 1);
	if (ret) {
		return ret;
	}

	return 0;
}

int ds248x_reset_bus(const struct i2c_dt_spec *i2c_spec)
{
	int ret;

	uint8_t reg;

	ret = ds248x_write(i2c_spec, CMD_1WRS, NULL);
	if (ret) {
		return ret;
	}

	do {
		ret = ds248x_read(i2c_spec, REG_NONE, &reg);
		if (ret) {
			return ret;
		}
	} while (reg & STATUS_1WB_msk);

	return reg & STATUS_PPD_msk ? 1 : 0;
}

int ds248x_reset_device(const struct i2c_dt_spec *i2c_spec)
{
	int ret;

	uint8_t reg;

	ret = ds248x_write(i2c_spec, CMD_DRST, NULL);
	if (ret) {
		return ret;
	}

	do {
		ret = ds248x_read(i2c_spec, REG_NONE, &reg);
		if (ret) {
			return ret;
		}
	} while (!(reg & STATUS_RST_msk));

	return 0;
}

int ds248x_write_config(const struct i2c_dt_spec *i2c_spec, uint8_t cfg)
{
	int ret;

	if (cfg & ~(DEVICE_APU_msk | DEVICE_PPD_msk | DEVICE_SPU_msk | DEVICE_1WS_msk)) {
		return -EINVAL;
	}

	uint8_t reg = cfg | ~cfg << 4;

	ret = ds248x_write(i2c_spec, CMD_WCFG, &reg);
	if (ret) {
		return ret;
	}

	ret = ds248x_read(i2c_spec, REG_NONE, &reg);
	if (ret) {
		return ret;
	}

	return (reg == cfg) ? 0 : -EIO;
}

int ds248x_single_bit(const struct i2c_dt_spec *i2c_spec, uint8_t bit_msk)
{
	int ret;

	uint8_t reg;

	ret = ds248x_write(i2c_spec, CMD_1WSB, &bit_msk);
	if (ret) {
		return ret;
	}

	do {
		ret = ds248x_read(i2c_spec, REG_NONE, &reg);
		if (ret) {
			return ret;
		}
	} while (reg & STATUS_1WB_msk);

	return reg & STATUS_SBR_msk;
}

int ds248x_read_bit(const struct i2c_dt_spec *i2c_spec)
{
	int ret;

	ret = ds248x_single_bit(i2c_spec, BIT_SET_msk);

	return ret > 1 ? 1 : ret;
}

int ds248x_write_bit(const struct i2c_dt_spec *i2c_spec, bool bit)
{
	int ret;

	ret = ds248x_single_bit(i2c_spec, bit ? BIT_SET_msk : BIT_CLR_msk);

	return ret > 0 ? 0 : ret;
}

int ds248x_read_byte(const struct i2c_dt_spec *i2c_spec)
{
	int ret;

	uint8_t reg;

	ret = ds248x_write(i2c_spec, CMD_1WRB, NULL);
	if (ret) {
		return ret;
	}

	do {
		ret = ds248x_read(i2c_spec, REG_NONE, &reg);
		if (ret) {
			return ret;
		}
	} while (reg & STATUS_1WB_msk);

	ret = ds248x_read(i2c_spec, REG_DATA, &reg);
	if (ret) {
		return ret;
	}

	return reg;
}

int ds248x_write_byte(const struct i2c_dt_spec *i2c_spec, uint8_t byte)
{
	int ret;

	uint8_t reg;

	ret = ds248x_write(i2c_spec, CMD_1WWB, &byte);
	if (ret) {
		return ret;
	}

	do {
		ret = ds248x_read(i2c_spec, REG_NONE, &reg);
		if (ret) {
			return ret;
		}
	} while (reg & STATUS_1WB_msk);

	return 0;
}
