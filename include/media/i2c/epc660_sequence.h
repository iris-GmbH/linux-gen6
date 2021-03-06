/*
 * Init sequence EPC660 ToF image sensor from Espros
 *
 * Copyright (C) 2017, Stefan Haun <stefan.haun@irisgmbh.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _EPC660_SEQUENCE_H
#define _EPC660_SEQUENCE_H

#define EPC660_IC_TYPE				0x00

/*
 * Register definitions
 */
#define EPC660_REG_IC_VERSION		0x01

#define EPC660_EEPROM_ADDRESS		0x11
#define EPC660_EEPROM_DATA			0x12

#define EPC660_REG_MT_0_HI			0x22
#define EPC660_REG_MT_0_LO			0x24
#define EPC660_REG_CFG_MODE_CTRL	0x7D
#define EPC660_REG_CLK_ENABLE		0x80
#define EPC660_REG_TCMI_CLK_DIVIDER	0x89
#define EPC660_REG_LED_DRIVER		0x90
#define EPC660_REG_SEQ_CONTROL		0x91
#define EPC660_REG_MOD_CTRL			0x92
#define EPC660_REG_READOUT_DIR		0x95
#define EPC660_REG_ROI_TL_X_HI		0x96
#define EPC660_REG_ROI_TL_X_LO		0x97
#define EPC660_REG_ROI_TL_Y			0x9A
#define EPC660_REG_ROI_BR_X_HI		0x98
#define EPC660_REG_ROI_BR_X_LO		0x99
#define EPC660_REG_ROI_BR_Y			0x9B
#define EPC660_REG_INTM_HI			0xA0
#define EPC660_REG_SHUTTER_CTRL		0xA4
#define EPC660_REG_POWER_CTRL		0xA5
#define EPC660_REG_TCMI_POLARITY	0xCC

#define EPC660_REG_CUSTOMER_ID		0xF5
#define EPC660_REG_WAFER_ID_MSB		0xF6
#define EPC660_REG_CHIP_ID_MSB		0xF8
#define EPC660_REG_PART_TYPE		0xFA
#define EPC660_REG_PART_VERSION		0xFB


/* Sequences are byte arrays (u8) consisting of sub-sequences.
 *
 * Each sub-sequence begins with the number of sequence bytes N,
 * followed by N bytes to programm on the I2C bus.
 *
 * A squence is terminated by a zero length sub-sequence.
 */
#define EPC660_I2CSEQ_TERMINATE		0

/*
 * Initialization sequence.
 */
static const u8 epc660_init_sequence[] = {
	2, EPC660_REG_CFG_MODE_CTRL,	0x9F,
	// Switch off clocks
	2, EPC660_REG_CLK_ENABLE,		0x00,
	2, EPC660_REG_CFG_MODE_CTRL,	0x94,
	2, EPC660_REG_CFG_MODE_CTRL,	0x84,
	// Switch system clock to PLL
	2, EPC660_REG_CFG_MODE_CTRL,	0x04,
	// Switch on clocks
	2, EPC660_REG_CLK_ENABLE,		0x3F,

	2, EPC660_REG_TCMI_POLARITY,	0x01,
	2, EPC660_REG_TCMI_CLK_DIVIDER,	0x03, /* TCMI Clock: 24 MHz */
	/* These need to go to the respective places in V4L2 control */
	2, EPC660_REG_MOD_CTRL,			0xC4, /* Grayscale mode 1 DCS */
	2, EPC660_REG_READOUT_DIR,		0x03,
	2, EPC660_REG_SEQ_CONTROL, 		0x43,
	3, EPC660_REG_INTM_HI, 			0x00, 0x01,
	EPC660_I2CSEQ_TERMINATE
}; /* epc660_init_sequence */


/* EPC660 Sequencer programm according to Operating_Instruction_epc660-XXX_V1.90
 * page 8/9.
 */
static const u8 epc660_003_Seq_Prog_8MHz_Default_8[] = {
	2, 0xa4, 0x00,
	2, 0x91, 0x00,
	2, 0x47, 0x01,
	9, 0x40, 0x00, 0x43, 0x10, 0x00, 0xC0, 0x00, 0x00, 0x0D,
	9, 0x40, 0x01, 0x43, 0x10, 0x00, 0x00, 0x01, 0x00, 0x0D,
	9, 0x40, 0x02, 0x43, 0x10, 0x00, 0x40, 0x0A, 0x00, 0x0D,
	9, 0x40, 0x03, 0x43, 0x10, 0x10, 0x02, 0x58, 0x00, 0x0D,
	9, 0x40, 0x04, 0x43, 0x10, 0x20, 0x01, 0x80, 0x00, 0x0D,
	9, 0x40, 0x05, 0x43, 0x10, 0xF0, 0x01, 0xB0, 0x00, 0x0D,
	9, 0x40, 0x06, 0x43, 0x10, 0x00, 0x01, 0x60, 0x00, 0x0D,
	9, 0x40, 0x07, 0x43, 0x10, 0xC0, 0x00, 0x78, 0x00, 0x0D,
	9, 0x40, 0x08, 0x43, 0x00, 0x40, 0x00, 0x18, 0x00, 0x0D,
	9, 0x40, 0x09, 0x43, 0x00, 0xD0, 0x02, 0x40, 0x00, 0x0D,
	9, 0x40, 0x0A, 0x43, 0x00, 0x10, 0xC0, 0x1E, 0x00, 0x0D,
	9, 0x40, 0x0B, 0x43, 0x10, 0x00, 0x00, 0x50, 0x00, 0x0D,
	9, 0x40, 0x0C, 0x43, 0x00, 0x20, 0x00, 0x18, 0x00, 0x0D,
	9, 0x40, 0x0D, 0x43, 0x00, 0xD0, 0x02, 0x40, 0x00, 0x0D,
	9, 0x40, 0x0E, 0x43, 0x00, 0x10, 0xC0, 0x1E, 0x00, 0x0D,
	9, 0x40, 0x0F, 0x43, 0x10, 0x00, 0x00, 0x50, 0x00, 0x0D,
	9, 0x40, 0x10, 0x43, 0x00, 0xD0, 0x02, 0x40, 0x00, 0x0D,
	9, 0x40, 0x11, 0x43, 0x10, 0x00, 0x00, 0x50, 0x00, 0x0D,
	9, 0x40, 0x12, 0x43, 0x08, 0x40, 0x40, 0x02, 0x00, 0x0D,
	9, 0x40, 0x13, 0x43, 0x08, 0x02, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x14, 0x43, 0x08, 0x00, 0x00, 0xA8, 0x00, 0x0D,
	9, 0x40, 0x15, 0x43, 0x18, 0x80, 0x07, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x16, 0x43, 0x08, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x17, 0x43, 0x08, 0x01, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x18, 0x43, 0x08, 0x00, 0x00, 0xA8, 0x00, 0x0D,
	9, 0x40, 0x19, 0x03, 0x08, 0x40, 0x03, 0x40, 0x00, 0x0D,
	9, 0x40, 0x1A, 0x03, 0x08, 0xE0, 0x01, 0x60, 0x00, 0x0D,
	9, 0x40, 0x1B, 0x03, 0x08, 0x10, 0xC0, 0x02, 0x00, 0x0D,
	9, 0x40, 0x1C, 0x03, 0x08, 0x40, 0x03, 0x40, 0x00, 0x0D,
	9, 0x40, 0x1D, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x1E, 0x03, 0x00, 0x00, 0x00, 0x50, 0x00, 0x0D,
	9, 0x40, 0x1F, 0x43, 0x10, 0x80, 0x40, 0x02, 0x00, 0x0D,
	9, 0x40, 0x20, 0x43, 0x10, 0x60, 0x00, 0x50, 0x00, 0x0D,
	9, 0x40, 0x21, 0x43, 0x18, 0x60, 0x40, 0x02, 0x00, 0x0D,
	9, 0x40, 0x22, 0x43, 0x18, 0x90, 0x07, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x23, 0x43, 0x08, 0x01, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x24, 0x43, 0x08, 0x00, 0x00, 0xA8, 0x00, 0x0D,
	9, 0x40, 0x25, 0x03, 0x08, 0x40, 0x03, 0x40, 0x00, 0x0D,
	9, 0x40, 0x26, 0x03, 0x00, 0x88, 0x00, 0x10, 0x00, 0x0D,
	9, 0x40, 0x27, 0x03, 0x00, 0x88, 0x3E, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x28, 0x03, 0x00, 0x08, 0x00, 0x14, 0x00, 0x0D,
	9, 0x40, 0x29, 0x03, 0x00, 0x80, 0x00, 0x10, 0x00, 0x0D,
	9, 0x40, 0x2A, 0x03, 0x00, 0x80, 0x3E, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x2B, 0x03, 0x00, 0x00, 0x00, 0x14, 0x00, 0x0D,
	9, 0x40, 0x2C, 0x03, 0x00, 0x00, 0x00, 0x50, 0x00, 0x0D,
	9, 0x40, 0x2D, 0x43, 0x08, 0x02, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x2E, 0x43, 0x08, 0x00, 0x00, 0xA8, 0x00, 0x0D,
	9, 0x40, 0x2F, 0x43, 0x18, 0x80, 0x07, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x30, 0x43, 0x08, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x31, 0x43, 0x08, 0x01, 0x00, 0x3C, 0x00, 0x0D,
	9, 0x40, 0x32, 0x43, 0x08, 0x00, 0x00, 0xA8, 0x00, 0x0D,
	9, 0x40, 0x33, 0x43, 0x08, 0x00, 0x00, 0x14, 0x00, 0x0D,
	9, 0x40, 0x34, 0x43, 0x08, 0x00, 0xC0, 0x00, 0x00, 0x0D,
	9, 0x40, 0x35, 0x43, 0x08, 0x00, 0x00, 0x01, 0x00, 0x0D,
	9, 0x40, 0x36, 0x43, 0x88, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x37, 0x43, 0x08, 0x30, 0x0A, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x38, 0x43, 0x28, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x39, 0x43, 0x08, 0x40, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x3A, 0x43, 0x08, 0xD0, 0x03, 0x88, 0x00, 0x0D,
	9, 0x40, 0x3B, 0x43, 0x08, 0x70, 0x09, 0x48, 0x00, 0x0D,
	9, 0x40, 0x3C, 0x40, 0x08, 0x00, 0x00, 0x44, 0x00, 0x0D,
	9, 0x40, 0x3D, 0x43, 0x08, 0x00, 0x04, 0x8C, 0x00, 0x0D,
	9, 0x40, 0x3E, 0x43, 0x08, 0x60, 0x08, 0x48, 0x00, 0x0D,
	9, 0x40, 0x3F, 0x40, 0x08, 0x00, 0x00, 0x44, 0x00, 0x0D,
	9, 0x40, 0x40, 0x43, 0x08, 0x30, 0x04, 0x90, 0x00, 0x0D,
	9, 0x40, 0x41, 0x41, 0x08, 0x70, 0x07, 0x48, 0x00, 0x0D,
	9, 0x40, 0x42, 0x40, 0x08, 0x00, 0x00, 0x44, 0x00, 0x0D,
	9, 0x40, 0x43, 0x41, 0x08, 0x50, 0x04, 0x48, 0x00, 0x0D,
	9, 0x40, 0x44, 0x40, 0x08, 0x00, 0x00, 0x44, 0x00, 0x0D,
	9, 0x40, 0x45, 0x05, 0x08, 0x00, 0x00, 0x34, 0x00, 0x0D,
	9, 0x40, 0x46, 0x04, 0x08, 0x50, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x47, 0x84, 0x0A, 0xF0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x48, 0x84, 0x0F, 0x00, 0x00, 0x54, 0x00, 0x0D,
	9, 0x40, 0x49, 0x85, 0x0E, 0x10, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x4A, 0x01, 0x0E, 0xD0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x4B, 0x00, 0x0E, 0x00, 0x00, 0xAC, 0x00, 0x0D,
	9, 0x40, 0x4C, 0x40, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x4D, 0x40, 0x08, 0x90, 0x05, 0x9C, 0x00, 0x0D,
	9, 0x40, 0x4E, 0x40, 0x08, 0x60, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x4F, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x50, 0x09, 0x48, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x51, 0x08, 0x08, 0x50, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x52, 0x88, 0x0A, 0xF0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x53, 0x88, 0x0F, 0x00, 0x00, 0x54, 0x00, 0x0D,
	9, 0x40, 0x54, 0x89, 0x0E, 0x10, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x55, 0x01, 0x0E, 0xD0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x56, 0x00, 0x0E, 0x00, 0x00, 0xAC, 0x00, 0x0D,
	9, 0x40, 0x57, 0x40, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x58, 0x40, 0x08, 0x00, 0x07, 0x94, 0x00, 0x0D,
	9, 0x40, 0x59, 0x40, 0x08, 0x00, 0x07, 0x94, 0x00, 0x0D,
	9, 0x40, 0x5A, 0x40, 0x08, 0x50, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x5B, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x5C, 0x11, 0x48, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x5D, 0x10, 0x08, 0x50, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x5E, 0x90, 0x0A, 0xF0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x5F, 0x90, 0x0F, 0x00, 0x00, 0x54, 0x00, 0x0D,
	9, 0x40, 0x60, 0x91, 0x0E, 0x10, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x61, 0x01, 0x0E, 0xD0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x62, 0x00, 0x0E, 0x00, 0x00, 0xAC, 0x00, 0x0D,
	9, 0x40, 0x63, 0x40, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x64, 0x40, 0x08, 0x00, 0x07, 0x9C, 0x00, 0x0D,
	9, 0x40, 0x65, 0x40, 0x08, 0x60, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x66, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x67, 0x21, 0x48, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x68, 0x20, 0x08, 0x50, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x69, 0xA0, 0x0A, 0xF0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x6A, 0xA0, 0x0F, 0x00, 0x00, 0x54, 0x00, 0x0D,
	9, 0x40, 0x6B, 0xA1, 0x0E, 0x10, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x6C, 0x01, 0x0E, 0xD0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x6D, 0x00, 0x0E, 0x00, 0x00, 0xAC, 0x00, 0x0D,
	9, 0x40, 0x6E, 0x40, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x6F, 0x40, 0x08, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x70, 0x40, 0x08, 0x00, 0xC0, 0x03, 0x00, 0x0D,
	9, 0x40, 0x71, 0x40, 0x08, 0x50, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x72, 0x41, 0x48, 0x00, 0x00, 0x14, 0x00, 0x0D,
	9, 0x40, 0x73, 0x00, 0x08, 0x00, 0x00, 0x54, 0x00, 0x0D,
	9, 0x40, 0x74, 0x00, 0x08, 0xC0, 0x02, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x75, 0x00, 0x48, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x76, 0x00, 0x08, 0x00, 0x00, 0x4C, 0x00, 0x0D,
	9, 0x40, 0x77, 0x15, 0x08, 0x00, 0x00, 0x34, 0x00, 0x0D,
	9, 0x40, 0x78, 0x14, 0x08, 0x50, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x79, 0x94, 0x0A, 0xF0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x7A, 0x94, 0x0F, 0x00, 0x00, 0x54, 0x00, 0x0D,
	9, 0x40, 0x7B, 0x95, 0x0E, 0x10, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x7C, 0x01, 0x0E, 0xD0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x7D, 0x00, 0x0E, 0x00, 0x00, 0xAC, 0x00, 0x0D,
	9, 0x40, 0x7E, 0x40, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x7F, 0x40, 0x08, 0x70, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x80, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x81, 0x29, 0x48, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x82, 0x28, 0x08, 0x50, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x83, 0xA8, 0x0A, 0xF0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x84, 0xA8, 0x0F, 0x00, 0x00, 0x54, 0x00, 0x0D,
	9, 0x40, 0x85, 0xA9, 0x0E, 0xC0, 0x06, 0x50, 0x00, 0x0D,
	9, 0x40, 0x86, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x87, 0x0D, 0x08, 0x00, 0x00, 0x34, 0x00, 0x0D,
	9, 0x40, 0x88, 0x0C, 0x08, 0x50, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x89, 0x8C, 0x0A, 0xF0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x8A, 0x8C, 0x0F, 0x00, 0x00, 0x54, 0x00, 0x0D,
	9, 0x40, 0x8B, 0x8D, 0x0E, 0x10, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x8C, 0x01, 0x0E, 0xD0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x8D, 0x00, 0x0E, 0x00, 0x00, 0xAC, 0x00, 0x0D,
	9, 0x40, 0x8E, 0x40, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x8F, 0x40, 0x08, 0x00, 0x07, 0x94, 0x00, 0x0D,
	9, 0x40, 0x90, 0x40, 0x08, 0x60, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x91, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x92, 0x31, 0x48, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x93, 0x30, 0x08, 0x50, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x94, 0xB0, 0x0A, 0xF0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x95, 0xB0, 0x0F, 0x00, 0x00, 0x54, 0x00, 0x0D,
	9, 0x40, 0x96, 0xB1, 0x0E, 0xC0, 0x06, 0x50, 0x00, 0x0D,
	9, 0x40, 0x97, 0x43, 0x08, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x98, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x0D,
	9, 0x40, 0x99, 0x3D, 0x08, 0x00, 0x00, 0x34, 0x00, 0x0D,
	9, 0x40, 0x9A, 0x3C, 0x08, 0x50, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x9B, 0xBC, 0x0A, 0xF0, 0x00, 0x0C, 0x00, 0x0D,
	9, 0x40, 0x9C, 0xBC, 0x0F, 0x00, 0x00, 0x54, 0x00, 0x0D,
	9, 0x40, 0x9D, 0xBD, 0x0E, 0xC0, 0x06, 0x50, 0x00, 0x0D,
	2, 0x47, 0x00,
	2, 0x91, 0x43,	// SEQ_Control: activate HSYNC stretch
	2, 0x90, 0xEC,
	2, 0xAB, 0x04,
	2, 0xAE, 0x01,
	EPC660_I2CSEQ_TERMINATE
}; // epc660_003_Seq_Prog_8MHz_Default_8

#endif 	/* _EPC660_SEQUENCE_H */
