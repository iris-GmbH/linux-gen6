/*
 * sc5xx_sport - Analog Devices SC5XX SPORT driver
 *
 * Copyright (c) 2015 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SC5XX_SPORT_H_
#define _SC5XX_SPORT_H_

#include <linux/platform_device.h>

#define TDM_MAX_SLOTS 8

#define SPORT_CTL_SPENPRI             0x00000001    /* Enable Primary Channel */
#define SPORT_CTL_DTYPE               0x00000006    /* Data type select */
#define SPORT_CTL_RJUSTIFY_ZFILL      0x00000000    /* DTYPE: MCM mode: Right-justify, zero-fill unused MSBs */
#define SPORT_CTL_RJUSTIFY_SFILL      0x00000002    /* DTYPE: MCM mode: Right-justify, sign-extend unused MSBs */
#define SPORT_CTL_USE_U_LAW           0x00000004    /* DTYPE: MCM mode: Compand using u-law */
#define SPORT_CTL_USE_A_LAW           0x00000006    /* DTYPE: MCM mode: Compand using A-law */
#define SPORT_CTL_LSBF                0x00000008    /* Serial bit endian select */
#define SPORT_CTL_SLEN                0x000001F0    /* Serial Word length select */
#define SPORT_CTL_PACK                0x00000200    /* 16-bit to 32-bit packing enable */
#define SPORT_CTL_ICLK                0x00000400    /* Internal Clock Select */
#define SPORT_CTL_OPMODE              0x00000800    /* Operation mode */
#define SPORT_CTL_CKRE                0x00001000    /* Clock rising edge select */
#define SPORT_CTL_FSR                 0x00002000    /* Frame Sync required */
#define SPORT_CTL_IFS                 0x00004000    /* Internal Frame Sync select */
#define SPORT_CTL_DIFS                0x00008000    /* Data-independent frame sync select */
#define SPORT_CTL_LFS                 0x00010000    /* Active low frame sync select */
#define SPORT_CTL_LAFS                0x00020000    /* Late Transmit frame select */
#define SPORT_CTL_RJUST               0x00040000    /* Right Justified mode select */
#define SPORT_CTL_FSED                0x00080000    /* External frame sync edge select */
#define SPORT_CTL_TFIEN               0x00100000    /* Transmit finish interrrupt enable select */
#define SPORT_CTL_GCLKEN              0x00200000    /* Gated clock mode select */
#define SPORT_CTL_SPENSEC             0x01000000    /* Enable secondary channel */
#define SPORT_CTL_SPTRAN              0x02000000    /* Data direction control */
#define SPORT_CTL_DERRSEC             0x04000000    /* Secondary channel error status */
#define SPORT_CTL_DXSSEC              0x18000000    /* Secondary channel data buffer status */
#define SPORT_CTL_SEC_EMPTY           0x00000000    /* DXSSEC: Empty */
#define SPORT_CTL_SEC_PART_FULL       0x10000000    /* DXSSEC: Partially full */
#define SPORT_CTL_SEC_FULL            0x18000000    /* DXSSEC: Full */
#define SPORT_CTL_DERRPRI             0x20000000    /* Primary channel error status */
#define SPORT_CTL_DXSPRI              0xC0000000    /* Primary channel data buffer status */
#define SPORT_CTL_PRM_EMPTY           0x00000000    /* DXSPRI: Empty */
#define SPORT_CTL_PRM_PART_FULL       0x80000000    /* DXSPRI: Partially full */
#define SPORT_CTL_PRM_FULL            0xC0000000    /* DXSPRI: Full */

#define SPORT_DIV_CLKDIV              0x0000FFFF    /* Clock divisor */
#define SPORT_DIV_FSDIV               0xFFFF0000    /* Frame sync divisor */

#define SPORT_MCTL_MCE                0x00000001    /* Multichannel enable */
#define SPORT_MCTL_MCPDE              0x00000004    /* Multichannel data packing select */
#define SPORT_MCTL_MFD                0x000000F0    /* Multichannel frame delay */
#define SPORT_MCTL_WSIZE              0x00007F00    /* Number of multichannel slots */
#define SPORT_MCTL_WOFFSET            0x03FF0000    /* Window offset size */

#define SPORT_CNT_CLKCNT              0x0000FFFF    /* Current state of clk div counter */
#define SPORT_CNT_FSDIVCNT            0xFFFF0000    /* Current state of frame div counter */

#define SPORT_ERR_DERRPMSK            0x00000001    /* Primary channel data error interrupt enable */
#define SPORT_ERR_DERRSMSK            0x00000002    /* Secondary channel data error interrupt enable */
#define SPORT_ERR_FSERRMSK            0x00000004    /* Frame sync error interrupt enable */
#define SPORT_ERR_DERRPSTAT           0x00000010    /* Primary channel data error status */
#define SPORT_ERR_DERRSSTAT           0x00000020    /* Secondary channel data error status */
#define SPORT_ERR_FSERRSTAT           0x00000040    /* Frame sync error status */

#define SPORT_MSTAT_CURCHAN           0x000003FF    /* Channel which is being serviced in the multichannel operation */

#define SPORT_CTL2_FSMUXSEL           0x00000001    /* Frame Sync MUX Select */
#define SPORT_CTL2_CKMUXSEL           0x00000002    /* Clock MUX Select */
#define SPORT_CTL2_LBSEL              0x00000004    /* Loopback Select */

struct sport_register {
	u32 spctl;
	u32 div;
	u32 spmctl;
	u32 spcs0;
	u32 spcs1;
	u32 spcs2;
	u32 spcs3;
	u32 spcnt;
	u32 sperrctl;
	u32 spmstat;
	u32 spctl2;
	u32 txa;
	u32 rxa;
	u32 txb;
	u32 rxb;
	u32 revid;
};

struct sport_device {
	struct platform_device *pdev;
	const unsigned short *pin_req;
	struct sport_register *tx_regs;
	struct sport_register *rx_regs;
	int tx_dma_chan;
	int rx_dma_chan;
	int tx_err_irq;
	int rx_err_irq;

	void (*tx_callback)(void *data);
	void *tx_data;
	void (*rx_callback)(void *data);
	void *rx_data;

	/* cpu address of dma descriptor */
	struct dmasg *tx_desc;
	struct dmasg *rx_desc;
	/* device address of dma descriptor */
	dma_addr_t tx_desc_phy;
	dma_addr_t rx_desc_phy;
	unsigned int tx_desc_size;
	unsigned int rx_desc_size;
	unsigned char *tx_buf;
	unsigned char *rx_buf;
	unsigned int tx_fragsize;
	unsigned int rx_fragsize;
	unsigned int tx_frags;
	unsigned int rx_frags;
	unsigned int wdsize;

	unsigned int tx_map[TDM_MAX_SLOTS];
	unsigned int rx_map[TDM_MAX_SLOTS];
};

struct sport_params {
	u32 spctl;
	u32 div;
	u32 spmctl;
	u32 spcs0;
};

struct sport_device *sport_create(struct platform_device *pdev);
void sport_delete(struct sport_device *sport);
int sport_set_tx_params(struct sport_device *sport,
		struct sport_params *params);
int sport_set_rx_params(struct sport_device *sport,
		struct sport_params *params);
void sport_tx_start(struct sport_device *sport);
void sport_rx_start(struct sport_device *sport);
void sport_tx_stop(struct sport_device *sport);
void sport_rx_stop(struct sport_device *sport);
void sport_set_tx_callback(struct sport_device *sport,
	void (*tx_callback)(void *), void *tx_data);
void sport_set_rx_callback(struct sport_device *sport,
	void (*rx_callback)(void *), void *rx_data);
int sport_config_tx_dma(struct sport_device *sport, void *buf,
	int fragcount, size_t fragsize);
int sport_config_rx_dma(struct sport_device *sport, void *buf,
	int fragcount, size_t fragsize);
unsigned long sport_curr_offset_tx(struct sport_device *sport);
unsigned long sport_curr_offset_rx(struct sport_device *sport);

#endif
