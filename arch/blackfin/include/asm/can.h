/*
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _MACH_CAN_H_
#define _MACH_CAN_H_

#define MAX_CHL_NUMBER	32

/*
 * All Blackfin system MMRs are padded to 32bits even if the register
 * itself is only 16bits.  So use a helper macro to streamline this.
 */

#define __BFP(m) u16 m; u16 __pad_##m

/*
 * bfin can registers layout
 */
struct bfin_can_mask_regs {
	__BFP(aml);
	__BFP(amh);
};

struct bfin_can_channel_regs {
	/* data[0,2,4,6] -> data{0,1,2,3} while data[1,3,5,7] is padding */
	u16 data[8];
	__BFP(dlc);
	__BFP(tsv);
	__BFP(id0);
	__BFP(id1);
};

struct bfin_can_regs {
	/*
	 * global control and status registers
	 */
	__BFP(mc1);           /* offset 0x00 */
	__BFP(md1);           /* offset 0x04 */
	__BFP(trs1);          /* offset 0x08 */
	__BFP(trr1);          /* offset 0x0c */
	__BFP(ta1);           /* offset 0x10 */
	__BFP(aa1);           /* offset 0x14 */
	__BFP(rmp1);          /* offset 0x18 */
	__BFP(rml1);          /* offset 0x1c */
	__BFP(mbtif1);        /* offset 0x20 */
	__BFP(mbrif1);        /* offset 0x24 */
	__BFP(mbim1);         /* offset 0x28 */
	__BFP(rfh1);          /* offset 0x2c */
	__BFP(opss1);         /* offset 0x30 */
	u32 __pad1[3];
	__BFP(mc2);           /* offset 0x40 */
	__BFP(md2);           /* offset 0x44 */
	__BFP(trs2);          /* offset 0x48 */
	__BFP(trr2);          /* offset 0x4c */
	__BFP(ta2);           /* offset 0x50 */
	__BFP(aa2);           /* offset 0x54 */
	__BFP(rmp2);          /* offset 0x58 */
	__BFP(rml2);          /* offset 0x5c */
	__BFP(mbtif2);        /* offset 0x60 */
	__BFP(mbrif2);        /* offset 0x64 */
	__BFP(mbim2);         /* offset 0x68 */
	__BFP(rfh2);          /* offset 0x6c */
	__BFP(opss2);         /* offset 0x70 */
	u32 __pad2[3];
	__BFP(clock);         /* offset 0x80 */
	__BFP(timing);        /* offset 0x84 */
	__BFP(debug);         /* offset 0x88 */
	__BFP(status);        /* offset 0x8c */
	__BFP(cec);           /* offset 0x90 */
	__BFP(gis);           /* offset 0x94 */
	__BFP(gim);           /* offset 0x98 */
	__BFP(gif);           /* offset 0x9c */
	__BFP(control);       /* offset 0xa0 */
	__BFP(intr);          /* offset 0xa4 */
	__BFP(version);       /* offset 0xa8 */
	__BFP(mbtd);          /* offset 0xac */
	__BFP(ewr);           /* offset 0xb0 */
	__BFP(esr);           /* offset 0xb4 */
	u32 __pad3[2];
	__BFP(ucreg);         /* offset 0xc0 */
	__BFP(uccnt);         /* offset 0xc4 */
	__BFP(ucrc);          /* offset 0xc8 */
	__BFP(uccnf);         /* offset 0xcc */
	u32 __pad4[1];
	__BFP(version2);      /* offset 0xd4 */
	u32 __pad5[10];

	/*
	 * channel(mailbox) mask and message registers
	 */
	struct bfin_can_mask_regs msk[MAX_CHL_NUMBER];    /* offset 0x100 */
	struct bfin_can_channel_regs chl[MAX_CHL_NUMBER]; /* offset 0x200 */
};

/* CAN_GIF Masks */
#define EWTIF       0x0001  /* TX Error Count IRQ Flag */
#define EWRIF       0x0002  /* RX Error Count IRQ Flag */
#define EPIF        0x0004  /* Error-Passive Mode IRQ Flag */
#define BOIF        0x0008  /* Bus Off IRQ Flag */
#define WUIF        0x0010  /* Wake-Up IRQ Flag */
#define UIAIF       0x0020  /* Access To Unimplemented Address IRQ Flag */
#define AAIF        0x0040  /* Abort Acknowledge IRQ Flag */
#define RMLIF       0x0080  /* RX Message Lost IRQ Flag */
#define UCEIF       0x0100  /* Universal Counter Overflow IRQ Flag */
#define EXTIF       0x0200  /* External Trigger Output IRQ Flag */
#define ADIF        0x0400  /* Access Denied IRQ Flag */

#endif
