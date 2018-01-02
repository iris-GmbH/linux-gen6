/*
 * Copyright (C) 2014 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */


#define SC57x_SYSTEM_L2_VIRT_BASE 0xFF020000
#define SC57x_SYSTEM_L2_SIZE      0x2C0000

/* ==================================================
        General Purpose Timer Block Registers
   ================================================== */

#define TIMER_GROUP                  0x31018004


/* =========================
        TIMER0
   ========================= */
#define TIMER0_CONFIG	0x31018060

/* =========================
        CGU0
   ========================= */
#define REG_CGU0_CTL                    0x3108D000         /* CGU0 Control Register */
#define REG_CGU0_STAT                   0x3108D008         /* CGU0 Status Register */
#define REG_CGU0_DIV                    0x3108D00C         /* CGU0 Clocks Divisor Register */

/* =========================
        CGU1
   ========================= */
#define REG_CGU1_CTL                    0x3108E000         /* CGU1 Control Register */
#define REG_CGU1_STAT                   0x3108E008         /* CGU1 Status Register */
#define REG_CGU1_DIV                    0x3108E00C         /* CGU1 Clocks Divisor Register */


/* =========================
        UART0
   ========================= */
#define UART0_REVID                 0x31003000         /* UART0 Revision ID Register */

/* =========================
        UART1
   ========================= */
#define UART1_REVID                 0x31003400         /* UART1 Revision ID Register */

/* =========================
        UART2
   ========================= */
#define UART2_REVID                 0x31003800         /* UART2 Revision ID Register */

/* =========================
        WDOG0
   ========================= */
#define REG_WDOG0_CTL                   0x31008000         /* WDOG0 Control Register */

/* =========================
        WDOG1
   ========================= */
#define REG_WDOG1_CTL                   0x31009000         /* WDOG1 Control Register */


/* ==================================================
        DMA Channel Registers
   ================================================== */

#define REG_DMA8_DSCPTR_NXT             0x310A7000         /* DMA8 Pointer to Next Initial Descriptor */
#define REG_DMA8_CFG                    0x310A7008         /* DMA8 Configuration Register */
#define REG_DMA9_DSCPTR_NXT             0x310A7080         /* DMA9 Pointer to Next Initial Descriptor */
#define REG_DMA9_CFG                    0x310A7088         /* DMA9 Configuration Register */
#define REG_DMA9_STAT                   0x310A70B0         /* DMA9 Status Register */
#define REG_DMA18_DSCPTR_NXT            0x310A7100         /* DMA18 Pointer to Next Initial Descriptor */
#define REG_DMA18_CFG                   0x310A7108         /* DMA18 Configuration Register */
#define REG_DMA19_DSCPTR_NXT            0x310A7180         /* DMA19 Pointer to Next Initial Descriptor */
#define REG_DMA19_CFG                   0x310A7188         /* DMA19 Configuration Register */
#define REG_DMA19_STAT                  0x310A71B0         /* DMA19 Status Register */

/* =========================
        L2CTL0
   ========================= */
#define L2CTL0_CTL                  0x31080000         /* L2CTL0 Control Register */
#define L2CTL0_STAT                 0x31080010         /* L2CTL0 Status Register */
#define L2CTL0_ERRADDR0             0x31080040         /* L2CTL0 ECC Error Address 0 Register */
#define L2CTL0_ET0                  0x31080080         /* L2CTL0 Error Type 0 Register */
#define L2CTL0_EADDR0               0x31080084         /* L2CTL0 Error Type 0 Address Register */
#define L2CTL0_ET1                  0x31080088         /* L2CTL0 Error Type 1 Register */
#define L2CTL0_EADDR1               0x3108008C         /* L2CTL0 Error Type 1 Address Register */

/* =========================
        SEC0
   ========================= */

/* --------------------------------------------------------------
       SEC Core Interface (SCI) Register Definitions
   -------------------------------------------------------------- */
#define SEC_COMMON_BASE	0x31089000
#define SEC_SCI_BASE	0x31089440
#define SEC_SSI_BASE	0x31089800

#define SEC_SCI_OFF		0x40
#define SEC_CCTL		0x0         /* SEC Core Control Register n */
#define SEC_CSID		0x1C        /* SEC Core IRQ Source ID Register n */

/* ---------------------------------------------------------------
   SEC Fault Management Interface (SFI) Register Definitions
   --------------------------------------------------------------- */
#define SEC_FCTL					0x10	/* SEC Fault Control Register */

/* ---------------------------------------------------------------
   SEC Global Register Definitions
   --------------------------------------------------------------- */
#define SEC_GCTL					0x0	/* SEC Global Control Register */
#define SEC_RAISE					0x8	/* SEC Global Raise Register */

/* ---------------------------------------------------------------
        SEC_SCTL                        Pos/Masks     Description
   --------------------------------------------------------------- */
#define SEC_SCTL_CTG				0x0F000000    /* Core Target Select */

/* ---------------------------------------------------------------
   SEC Source Interface (SSI) Register Definitions
   --------------------------------------------------------------- */
#define SEC_SCTL0					0x0	/* SEC Source Control Register n */

/* ---------------------------------------------------------------
        SEC_SCTL                             Pos/Masks     Description
   --------------------------------------------------------------- */
#define SEC_SCTL_SRC_EN                 0x00000004    /* SEN: Enable */
#define SEC_SCTL_FAULT_EN               0x00000002    /* FEN: Enable */
#define SEC_SCTL_INT_EN                 0x00000001    /* IEN: Enable */

/* =========================
        TRU0
   ========================= */
#define REG_TRU0_SSR71                  0x3108A118         /* TRU0 Slave Select Register */
#define REG_TRU0_SSR75                  0x3108A128         /* TRU0 Slave Select Register */
#define REG_TRU0_SSR79                  0x3108A138         /* TRU0 Slave Select Register */
#define REG_TRU0_MTR                    0x3108A7E0         /* TRU0 Master Trigger Register */
#define REG_TRU0_GCTL                   0x3108A7F4         /* TRU0 Global Control Register */

/* ===================================
       Trigger Master Definitions
   =================================== */
#define TRGM_SOFT0                            70           /* Software-driven Trigger 0 */
#define TRGM_SOFT1                            71           /* Software-driven Trigger 1 */
#define TRGM_SOFT2                            72           /* Software-driven Trigger 2 */

/* =========================
        RCU0
   ========================= */
#define REG_RCU0_CTL                    0x3108C000         /* RCU0 Control Register */
#define REG_RCU0_STAT                   0x3108C004         /* RCU0 Status Register */
#define REG_RCU0_CRCTL                  0x3108C008         /* RCU0 Core Reset Control Register */
#define REG_RCU0_CRSTAT                 0x3108C00C         /* RCU0 Core Reset Status Register */
#define REG_RCU0_SIDIS                  0x3108C01C         /* RCU0 System Interface Disable Register */
#define REG_RCU0_SISTAT                 0x3108C020         /* RCU0 System Interface Status Register */
#define REG_RCU0_BCODE                  0x3108C028         /* RCU0 Boot Code Register */
#define REG_RCU0_MSG_SET                0x3108C070         /* RCU0 Message Set Bits Register */
#define REG_RCU0_SVECT1                 0x3108C030         /* RCU0 Software Vector Register 1 */
#define REG_RCU0_SVECT2                 0x3108C034         /* RCU0 Software Vector Register 2 */

/* =========================
        SPU0
   ========================= */
#define REG_SPU0_CTL                    0x3108B000         /* SPU0 Control Register */

/* =========================
   PADS0
   ========================= */
#define REG_PADS0_PCFG0                 0x31004404         /* PADS0 Peripheral Configuration0 Register */

#define PADS_PCFG_EMACNORESET		0x4	   	   /* EMAC reset: 0 = asserted */
#define PADS_PCFG_EMACRMII		0x10		   /* Set EMAC to RMII mode */

/* =========================
        LP0
   ========================= */
#define LP0_CTL                     0x30FFE000 /* LP0 Control Register */

/* =========================
        LP1
   ========================= */
#define LP1_CTL                     0x30FFE100 /* LP1 Control Register */

/* =========================
        EPPI
   ========================= */
#define EPPI0_CTL                   0x3102D020 /* EPPI0 Control Register */

