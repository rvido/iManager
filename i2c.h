/*
 * Advantech iManager I2C bus core
 *
 * Copyright (C) 2015 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __I2C_H__
#define __I2C_H__

#include <linux/types.h>

#define I2C_MAX_READ_BYTES	32
#define I2C_MAX_WRITE_BYTES	32

/* Only for setting SMBus frequency */
enum smb_bus_id {
	SMB_OEM_0,
	SMB_OEM_1,
	SMB_OEM_2,
	SMB_EEPROM,
	SMB_TH_0,
	SMB_TH_1,
	SMB_SECURITY_EEPROM,
	I2C_OEM_1,
};

int i2c_core_init(void);

int i2c_core_write_quick(u16 addr);

int i2c_core_read_byte(u16 addr);
int i2c_core_write_byte(u16 addr, u8 cmd);

int i2c_core_write_byte_data(u16 addr, u8 cmd, u8 value);
int i2c_core_read_byte_data(u16 addr, u8 cmd);

int i2c_core_write_word_data(u16 addr, u8 cmd, u16 value);
int i2c_core_read_word_data(u16 addr, u8 cmd);

int i2c_core_write_block_data(u16 addr, u8 cmd, u8 *buf);
int i2c_core_read_block_data(u16 addr, u8 cmd, u8 *buf);

int i2c_core_write_i2c_block_data(u16 addr, u8 cmd, u8 *buf);
int i2c_core_read_i2c_block_data(u16 addr, u8 cmd, u8 *buf);

int i2c_core_smb_get_freq(u32 bus_id);
int i2c_core_smb_set_freq(u32 bus_id, u32 freq);

#endif
