/*
 * Copyright (c) 2022 Caspar Friedrich <c.s.w.friedrich@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_W1_DS248x_COMMON_H_
#define ZEPHYR_DRIVERS_W1_DS248x_COMMON_H_

#include <drivers/i2c.h>
#include <zephyr.h>

#define CMD_1WT 0x78
#define CMD_1WSB 0x87
#define CMD_1WRB 0x96
#define CMD_1WWB 0xa5
#define CMD_1WRS 0xb4
#define CMD_CHSL 0xc3 /* DS2482-800 only */
#define CMD_ADJP 0xc3 /* DS2484 only */
#define CMD_WCFG 0xd2
#define CMD_SRP 0xe1
#define CMD_DRST 0xf0

#define REG_NONE 0x00 /* special value */
#define REG_PORT 0xb4 /* DS2484 only */
#define REG_DEVICE 0xc3 /* DS2482-800 only */
#define REG_CHANNEL 0xd2
#define REG_DATA 0xe1
#define REG_STATUS 0xf0

/*
 * Device Configuration Register
 */
#define DEVICE_APU_pos 0
#define DEVICE_APU_msk BIT(0)
#define DEVICE_PPD_pos 1 /* DS2484 only */
#define DEVICE_PPD_msk BIT(1) /* DS2484 only */
#define DEVICE_SPU_pos 2
#define DEVICE_SPU_msk BIT(2)
#define DEVICE_1WS_pos 3
#define DEVICE_1WS_msk BIT(3)

/*
 * Status Register
 */
#define STATUS_1WB_pos 0
#define STATUS_1WB_msk BIT(0)
#define STATUS_PPD_pos 1
#define STATUS_PPD_msk BIT(1)
#define STATUS_SD_pos 2
#define STATUS_SD_msk BIT(2)
#define STATUS_LL_pos 3
#define STATUS_LL_msk BIT(3)
#define STATUS_RST_pos 4
#define STATUS_RST_msk BIT(4)
#define STATUS_SBR_pos 5
#define STATUS_SBR_msk BIT(5)
#define STATUS_TSB_pos 6
#define STATUS_TSB_msk BIT(6)
#define STATUS_DIR_pos 7
#define STATUS_DIR_msk BIT(7)

/*
 * Channel Selection Codes, DS2482-800 only
 */
#define CHSL_IO0 0xf0
#define CHSL_IO1 0xe1
#define CHSL_IO2 0xd2
#define CHSL_IO3 0xc3
#define CHSL_IO4 0xb4
#define CHSL_IO5 0xa5
#define CHSL_IO6 0x96
#define CHSL_IO7 0x87

/*
 * Port Configuration Register, DS2484 only
 */
#define PORT_VAL0_pos 0
#define PORT_VAL0_msk BIT(0)
#define PORT_VAL1_pos 1
#define PORT_VAL1_msk BIT(1)
#define PORT_VAL2_pos 2
#define PORT_VAL2_msk BIT(2)
#define PORT_VAL3_pos 3
#define PORT_VAL3_msk BIT(3)

/*
 * Bit Byte
 */
#define BIT_CLR_msk 0
#define BIT_SET_msk BIT(7)

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Generic I2C bus operations
 */
int ds248x_write(const struct i2c_dt_spec *i2c_spec, uint8_t cmd, uint8_t *data);
int ds248x_read(const struct i2c_dt_spec *i2c_spec, uint8_t rp, uint8_t *reg);

/*
 * Generic 1-Wire bus operations
 */
int ds248x_reset_bus(const struct i2c_dt_spec *i2c_spec);
int ds248x_reset_device(const struct i2c_dt_spec *i2c_spec);
int ds248x_write_config(const struct i2c_dt_spec *i2c_spec, uint8_t cfg);

/*
 * 1-Wire bit operations
 */
int ds248x_single_bit(const struct i2c_dt_spec *i2c_spec, uint8_t bit_msk);
int ds248x_read_bit(const struct i2c_dt_spec *i2c_spec);
int ds248x_write_bit(const struct i2c_dt_spec *i2c_spec, bool bit);

/*
 * 1-Wire byte operations
 */
int ds248x_read_byte(const struct i2c_dt_spec *i2c_spec);
int ds248x_write_byte(const struct i2c_dt_spec *i2c_spec, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_W1_DS248x_COMMON_H_ */
