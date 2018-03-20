#ifndef ARCH_ARM_MACH_SC57X_IRIS_TMU_H_
#define ARCH_ARM_MACH_SC57X_IRIS_TMU_H_

/* ============================================================================================================================
        TMU Register BitMasks, Positions & Enumerations
   ============================================================================================================================ */

/* TMU_CTL */
#define TMU0_CTL_MASK		0x1F9
#define TMU0_CTL_TMEN_FORCE	(1<<8)	//Indefinite Enable
#define TMU0_CTL_SCLKDIV	(1<<4)	//System Clock Devide [7:4]; SCLK is divided by (21+4*SCLKDIV).
#define TMU0_CTL_TMEN		(1<<3)	//Periodic Enable
#define TMU0_CTL_TMPU		(1<<0)	//TMU PowerUp(1) PowerDown(0)

/* TMU_AVG */
#define TMU0_AVG_MASK		0x1
#define TMU0_AVG_VALUE		(1<<0)	//Enable averaging the TMU; (7 x previous_avg_value + current_value)/8

/* TMU_STAT */
#define TMU0_STAT_MASK		0xF0
#define TMU0_STAT_ALRTLO	(1<<7)	//Alert Low
#define TMU0_STAT_FLTLO		(1<<6)	//Fault Low
#define TMU0_STAT_ALRTHI	(1<<5)	//Alert High
#define TMU0_STAT_FLTHI		(1<<4)	//Fault High

/* TMU_CNV_BLANK */
#define TMU0_CNV_BLANK_VALUE	(1<<0) //Blanking Period [4:0]; (VALUE+1)*50k SCLK, default 200k, TMUHADC_BUSY!!

/* TMU_GAIN  */
#define TMU0_GAIN_MASK		0x3FF
#define TMU0_GAIN_VALUE		(1<<0)	//Correct Gain Error [9:0]

/* TMU_IMSK */
#define TMU0_IMSK_ALRTLO	(4<<0)
#define TMU0_IMSK_FLTLO		(3<<0)
#define TMU0_IMSK_ALRTHI	(2<<0)
#define TMU0_IMSK_FLTHI		(1<<0)
#define TMU0_IMSK_MASK		0xF

/* TMU_OFFSET */
#define TMU0_OFFSET_MASK		0x1FFF
#define TMU0_OFFSET	(1<<0)	//Offset Value in Q3.8 [12:0]

/* TMU0_TEMP */
#define TMU0_TEMP_MASK		0x1FFF

/* TMU_ALERT_HIGH*/
#define TMU0_ALRT_LIM_HI_MASK		0xFF
#define TMU0_FLT_LIM_HI_MASK		0xFF

#define HIGH_ALERT_LIM				60	/* °C */
#define HIGH_FAULT_LIM				65	/* °C */

#endif /* ARCH_ARM_MACH_SC57X_IRIS_TMU_H_ */
