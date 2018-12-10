/*
 * core timer and machine init for ADI processor on-chip memory
 *
 * Copyright 2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/gfp.h>
#include <linux/bitops.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/phy.h>
#include <linux/sched_clock.h>

#include <asm/irq.h>
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/icst.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <mach/hardware.h>
#include <mach/cpu.h>
#include <mach/dma.h>
#include <mach/portmux-sc58x.h>
#include <mach/sc58x.h>
#include <mach/irqs.h>
#include <mach/clkdev.h>
#include <mach/sec.h>

#include "core.h"

#define TIMER_CLOCKSOURCE 1
#define TIMER_CLOCKEVENT  0

static struct sc58x_gptimer *timer_clock, *timer_event;

void __init sc58x_init_irq(void)
{
	gic_init(0, 32,
		__io_address(SC58X_GIC_PORT0),
		__io_address(SC58X_GIC_PORT1));
}

static struct map_desc sc58x_io_desc[] __initdata __maybe_unused = {
	{
		.virtual	=  IO_ADDRESS(SYS_MMR_BASE),
		.pfn		= __phys_to_pfn(SYS_MMR_BASE),
		.length		= SYS_MMR_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	=  IO_ADDRESS(SYS_L2_START),
		.pfn		= __phys_to_pfn(SYS_L2_START),
		.length		= SZ_16K,
		.type		= MT_MEMORY_RWX_NONCACHED,
	}, {
		.virtual	=  IO_ADDRESS(SYS_SRAM_BASE),
		.pfn		= __phys_to_pfn(SYS_SRAM_BASE),
		.length		= SYS_SRAM_SIZE,
#ifdef CONFIG_ICC
		.type		= MT_MEMORY_RWX_NONCACHED,
#else
		.type		= MT_MEMORY_RWX,
#endif
	},
};

void __init sc58x_map_io(void)
{
	iotable_init(sc58x_io_desc, ARRAY_SIZE(sc58x_io_desc));
}

void sc58x_restart(enum reboot_mode mode, const char *cmd)
{
	writel(1, __io_address(REG_RCU0_CTL));
}


#include <asm/siginfo.h>
#include <asm/signal.h>


static bool first_fault = true;

static int sc58x_abort_handler(unsigned long addr, unsigned int fsr,
		struct pt_regs *regs)
{
	if (fsr == 0x1c06 && first_fault) {
		first_fault = false;

		/*
		 * These faults with code 0x1c06 happens for no good reason,
		 * possibly left over from the CFE boot loader.
		 */
		pr_warn("External imprecise Data abort at addr=%#lx, fsr=%#x ignored.\n",
				addr, fsr);

		/* Returning non-zero causes fault display and panic */
		return 0;
	}

	/* Others should cause a fault */
	return 1;
}

/* Early initializations */
void __init sc58x_init_early(void)
{
	/* Install our hook */
	hook_fault_code(16 + 6, sc58x_abort_handler, SIGBUS, BUS_OBJERR,
			"imprecise external abort");
	sc58x_clock_init();
}

#if IS_ENABLED(CONFIG_VIDEO_BLACKFIN_CAPTURE)
#include <linux/videodev2.h>
#include <media/blackfin/bfin_capture.h>
#include <media/blackfin/ppi.h>

#if IS_ENABLED(CONFIG_VIDEO_ADV7842)
#include <media/adv7842.h>
static struct v4l2_input adv7842_inputs[] = {
	{
		.index = 0,
		.name = "Composite",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = V4L2_STD_ALL,
		.capabilities = V4L2_IN_CAP_STD,
	},
	{
		.index = 1,
		.name = "S-Video",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = V4L2_STD_ALL,
		.capabilities = V4L2_IN_CAP_STD,
	},
	{
		.index = 2,
		.name = "Component",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.capabilities = V4L2_IN_CAP_CUSTOM_TIMINGS,
	},
	{
		.index = 3,
		.name = "VGA",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.capabilities = V4L2_IN_CAP_CUSTOM_TIMINGS,
	},
	{
		.index = 4,
		.name = "HDMI",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.capabilities = V4L2_IN_CAP_CUSTOM_TIMINGS,
	},
};

static struct bcap_route adv7842_routes[] = {
	{
		.input = 3,
		.output = 0,
		.ppi_control = (PACK_EN | DLEN_8 | EPPI_CTL_FLDSEL
				| EPPI_CTL_ACTIVE656),
	},
	{
		.input = 4,
		.output = 0,
	},
	{
		.input = 2,
		.output = 0,
	},
	{
		.input = 1,
		.output = 0,
	},
	{
		.input = 0,
		.output = 1,
		.ppi_control = (EPPI_CTL_SPLTWRD | PACK_EN | DLEN_16
				| EPPI_CTL_FS1HI_FS2HI | EPPI_CTL_POLC2
				| EPPI_CTL_SYNC2 | EPPI_CTL_NON656),
	},
};

static struct adv7842_output_format adv7842_opf[] = {
	{
		.op_ch_sel = ADV7842_OP_CH_SEL_BRG,
		.op_format_sel = ADV7842_OP_FORMAT_SEL_SDR_ITU656_8,
		.op_656_range = 1,
		.blank_data = 1,
		.insert_av_codes = 1,
	},
	{
		.op_ch_sel = ADV7842_OP_CH_SEL_RGB,
		.op_format_sel = ADV7842_OP_FORMAT_SEL_SDR_ITU656_16,
		.op_656_range = 1,
		.blank_data = 1,
	},
};

static struct adv7842_platform_data adv7842_data = {
	.opf = adv7842_opf,
	.num_opf = ARRAY_SIZE(adv7842_opf),
	.ain_sel = ADV7842_AIN10_11_12_NC_SYNC_4_1,
	.prim_mode = ADV7842_PRIM_MODE_SDP,
	.vid_std_select = ADV7842_SDP_VID_STD_CVBS_SD_4x1,
	.inp_color_space = ADV7842_INP_COLOR_SPACE_AUTO,
	.i2c_sdp_io = 0x40,
	.i2c_sdp = 0x41,
	.i2c_cp = 0x42,
	.i2c_vdp = 0x43,
	.i2c_afe = 0x44,
	.i2c_hdmi = 0x45,
	.i2c_repeater = 0x46,
	.i2c_edid = 0x47,
	.i2c_infoframe = 0x48,
	.i2c_cec = 0x49,
	.i2c_avlink = 0x4a,
};

static struct bfin_capture_config bfin_capture_data = {
	.inputs = adv7842_inputs,
	.num_inputs = ARRAY_SIZE(adv7842_inputs),
	.routes = adv7842_routes,
	.board_info = {
		.type = "adv7842",
		.addr = 0x20,
		.platform_data = (void *)&adv7842_data,
	},
	.ppi_control = (PACK_EN | DLEN_8 | EPPI_CTL_FLDSEL
			| EPPI_CTL_ACTIVE656),
};
#endif
#endif

#if IS_ENABLED(CONFIG_VIDEO_BLACKFIN_DISPLAY)
#include <linux/videodev2.h>
#include <media/blackfin/bfin_display.h>
#include <media/blackfin/ppi.h>

#if IS_ENABLED(CONFIG_VIDEO_ADV7511)
#include <media/adv7511.h>

static struct v4l2_output adv7511_outputs[] = {
	{
		.index = 0,
		.name = "HDMI",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.capabilities = V4L2_OUT_CAP_CUSTOM_TIMINGS,
	},
};

static struct disp_route adv7511_routes[] = {
	{
		.output = 0,
	},
};

static struct adv7511_platform_data adv7511_data = {
	.edid_addr = 0x7e,
};

static struct bfin_display_config bfin_display_data = {
	.outputs = adv7511_outputs,
	.num_outputs = ARRAY_SIZE(adv7511_outputs),
	.routes = adv7511_routes,
	.board_info = {
		.type = "adv7511",
		.addr = 0x39,
		.platform_data = (void *)&adv7511_data,
	},
	.ppi_control = (EPPI_CTL_SPLTWRD | PACK_EN | DLEN_16
			| EPPI_CTL_FS1LO_FS2LO | EPPI_CTL_POLC3
			| EPPI_CTL_IFSGEN | EPPI_CTL_SYNC2
			| EPPI_CTL_NON656 | EPPI_CTL_DIR),
};
#endif

#if IS_ENABLED(CONFIG_VIDEO_ADV7343)
#include <media/adv7343.h>

static struct v4l2_output adv7343_outputs[] = {
	{
		.index = 0,
		.name = "Composite",
		.type = V4L2_OUTPUT_TYPE_ANALOG,
		.std = V4L2_STD_ALL,
		.capabilities = V4L2_OUT_CAP_STD,
	},
	{
		.index = 1,
		.name = "S-Video",
		.type = V4L2_OUTPUT_TYPE_ANALOG,
		.std = V4L2_STD_ALL,
		.capabilities = V4L2_OUT_CAP_STD,
	},
	{
		.index = 2,
		.name = "Component",
		.type = V4L2_OUTPUT_TYPE_ANALOG,
		.std = V4L2_STD_ALL,
		.capabilities = V4L2_OUT_CAP_STD,
	},
};

static struct disp_route adv7343_routes[] = {
	{
		.output = ADV7343_COMPOSITE_ID,
	},
	{
		.output = ADV7343_SVIDEO_ID,
	},
	{
		.output = ADV7343_COMPONENT_ID,
	},
};

static struct adv7343_platform_data adv7343_data = {
	.mode_config = {
		.sleep_mode = false,
		.pll_control = false,
		.dac = {1, 1, 1, 1, 1, 1},
	},
	.sd_config = {
		.sd_dac_out = {0},
	},
};

static struct bfin_display_config bfin_display_data = {
	.outputs = adv7343_outputs,
	.num_outputs = ARRAY_SIZE(adv7343_outputs),
	.routes = adv7343_routes,
	.board_info = {
		.type = "adv7343",
		.addr = 0x2b,
		.platform_data = (void *)&adv7343_data,
	},
	.ppi_control = (PACK_EN | DLEN_8 | EPPI_CTL_FS1LO_FS2LO
			| EPPI_CTL_POLC3 | EPPI_CTL_BLANKGEN | EPPI_CTL_SYNC2
		| EPPI_CTL_NON656 | EPPI_CTL_DIR),
};
#endif
#endif

#ifdef CONFIG_OF
static const struct of_dev_auxdata sc58x_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("adi,adi2-pinctrl", 0, "pinctrl-adi2.0", NULL),
	OF_DEV_AUXDATA("arm,adi-uart4", UART0_REVID, "adi-uart4.0", NULL),
	OF_DEV_AUXDATA("arm,adi-uart4", UART1_REVID, "adi-uart4.1", NULL),
	OF_DEV_AUXDATA("arm,adi-uart4", UART2_REVID, "adi-uart4.2", NULL),
	OF_DEV_AUXDATA("arm,adi-watchdog", REG_WDOG0_CTL, "adi-watchdog.0", NULL),
	OF_DEV_AUXDATA("adi,spi3", 0, "adi-spi3.2", NULL),
	OF_DEV_AUXDATA("adi,mmc", 0x31010000, "mmc.0", NULL),
#if IS_ENABLED(CONFIG_VIDEO_BLACKFIN_DISPLAY)
	OF_DEV_AUXDATA("adi,disp", 0x31040000, "bfin_display.0", &bfin_display_data),
#endif
#if IS_ENABLED(CONFIG_VIDEO_BLACKFIN_CAPTURE)
	OF_DEV_AUXDATA("adi,cap", 0x31040000, "bfin_capture.0", &bfin_capture_data),
#endif
	{},
};

static struct of_device_id sc58x_of_bus_ids[] __initdata = {
	{ .compatible = "simple-bus", },
	{},
};
#endif

static int sc58x_phy0_fixup(struct phy_device *phydev)
{
	int  phy_data = 0;

	phy_data = phy_read(phydev, 0x12);

	/* enable 3com mode for RGMII */

	phy_write(phydev, 0x12, (3 << 12) | phy_data);

	return 0;
}

static int sc58x_phy1_fixup(struct phy_device *phydev)
{
	phy_write(phydev, 0x11, 3);

	return 0;
}

#if IS_ENABLED(CONFIG_SND_SC5XX_PCM)
static struct platform_device sc58x_pcm = {
	.name = "sc5xx-pcm-audio",
	.id = -1,
};
#endif

static struct platform_device *ezkit_devices[] __initdata = {
#if IS_ENABLED(CONFIG_SND_SC5XX_PCM)
	&sc58x_pcm,
#endif
};

void __init sc58x_init(void)
{
#ifdef CONFIG_CACHE_L2X0
	l2x0_of_init(0, ~0UL);
#endif

	pr_info("%s: registering device resources\n", __func__);

	sec_init(__io_address(SEC_COMMON_BASE), __io_address(SEC_SCI_BASE),
			__io_address(SEC_SSI_BASE));
#ifdef CONFIG_OF
	of_platform_populate(NULL, sc58x_of_bus_ids,
				sc58x_auxdata_lookup, NULL);
#endif
#if defined(CONFIG_MACH_SC589_MINI) && !defined(CONFIG_DP83867_PHY)
	/* select RMII interface */
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		writel((readl(__io_address(REG_PADS0_PCFG0)) & (~(0x00000008))),
				__io_address(REG_PADS0_PCFG0));
		writel((readl(__io_address(REG_PADS0_PCFG0)) | 0x4),
				__io_address(REG_PADS0_PCFG0));
	}
#else
	/* select RGMII interface */
	if (IS_BUILTIN(CONFIG_PHYLIB))
		writel((readl(__io_address(REG_PADS0_PCFG0)) | 0xc),
				__io_address(REG_PADS0_PCFG0));

	if (IS_BUILTIN(CONFIG_PHYLIB))
		phy_register_fixup_for_uid(0x20005c7a, 0xffffffff,
				sc58x_phy0_fixup);

	if (IS_BUILTIN(CONFIG_PHYLIB))
		phy_register_fixup_for_uid(0x20005c90, 0xffffffff,
				sc58x_phy1_fixup);
#endif
	platform_add_devices(ezkit_devices, ARRAY_SIZE(ezkit_devices));
#ifdef CONFIG_ICC
	platform_ipi_init();
#endif
}

static void __iomem *spu_base;

void set_spu_securep_msec(uint16_t n, bool msec)
{
	void __iomem *p = (void __iomem *)(spu_base + 0xA00 + 4 * n);
	u32 securep = ioread32(p);

	if (msec)
		iowrite32(securep | 0x3, p);
	else
		iowrite32(securep & ~0x3, p);
}
EXPORT_SYMBOL(set_spu_securep_msec);

static int __init spu_init(void)
{
	spu_base = ioremap(REG_SPU0_CTL, 0x1000);

	return 0;
}
arch_initcall(spu_init);

void __init setup_gptimer(struct sc58x_gptimer *timer)
{
	int id = timer->id;

	disable_gptimers(1 << id);
	set_gptimer_config(timer, TIMER_OUT_DIS
			| TIMER_MODE_PWM_CONT | TIMER_PULSE_HI | TIMER_IRQ_PER);
	set_gptimer_period(timer, 0xFFFFFFFF);
	set_gptimer_pwidth(timer, 0xFFFFFFFE);

	enable_gptimers(1 << id);
}

static cycle_t read_gptimer(struct clocksource *cs)
{
	return (cycle_t)get_gptimer_count(timer_clock);
}

static struct clocksource cs_gptimer = {
	.name           = "cs_gptimer",
	.rating         = 350,
	.read           = read_gptimer,
	.mask           = CLOCKSOURCE_MASK(32),
	.flags          = CLOCK_SOURCE_IS_CONTINUOUS,
};

static int __init cs_gptimer_init(void)
{
	setup_gptimer(timer_clock);

	if (clocksource_register_hz(&cs_gptimer, get_sclk()))
		panic("failed to register clocksource");

	return 0;
}

static int gptmr_set_next_event(unsigned long cycles,
		struct clock_event_device *evt)
{
	int id = timer_event->id;

	disable_gptimers(1 << id);

	/* it starts counting three SCLK cycles after the TIMENx bit is set */
	set_gptimer_pwidth(timer_event, cycles - 3);
	enable_gptimers(1 << id);
	return 0;
}

static void gptmr_set_mode(enum clock_event_mode mode,
		struct clock_event_device *evt)
{
	int id = timer_event->id;

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		disable_gptimers(1 << id);
		set_gptimer_config(timer_event, TIMER_OUT_DIS
				| TIMER_MODE_PWM_CONT | TIMER_PULSE_HI |
				TIMER_IRQ_PER);

		set_gptimer_period(timer_event, get_sclk() / HZ);
		set_gptimer_pwidth(timer_event, get_sclk() / HZ - 1);
		enable_gptimers(1 << id);
		break;

	case CLOCK_EVT_MODE_ONESHOT:
		while(1);
		disable_gptimers(1 << id);
		set_gptimer_config(timer_event, TIMER_OUT_DIS | TIMER_MODE_PWM
				| TIMER_PULSE_HI | TIMER_IRQ_WID_DLY);
		set_gptimer_period(timer_event, 0);
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		disable_gptimers(1 << id);
		break;
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static void gptmr_ack(int id)
{
	set_gptimer_status(1 << id);
}

static u64 notrace gptmr_read_sched(void)
{
	return (u32)get_gptimer_count(timer_clock);
}

irqreturn_t gptmr_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	/*
	 * We want to ACK before we handle so that we can handle smaller timer
	 * intervals.  This way if the timer expires again while we're handling
	 * things, we're more likely to see that 2nd int rather than swallowing
	 * it by ACKing the int at the end of this handler.
	 */
	gptmr_ack(TIMER_CLOCKEVENT);
	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static struct irqaction gptmr_irq = {
	.name           = "SC58x GPTimer0",
	.flags          = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler        = gptmr_interrupt,
};

static struct clock_event_device clockevent_gptmr = {
	.name           = "sc58x_gptimer0",
	.rating         = 300,
	.shift          = 32,
	.features       = CLOCK_EVT_FEAT_PERIODIC,
	.set_next_event = gptmr_set_next_event,
	.set_mode       = gptmr_set_mode,
};


static void __init gptmr_clockevent_init(struct clock_event_device *evt)
{
	unsigned long clock_tick;

	clock_tick = get_sclk();
	evt->mult = div_sc(clock_tick, NSEC_PER_SEC, evt->shift);
	evt->max_delta_ns = clockevent_delta2ns(-1, evt);
	evt->min_delta_ns = clockevent_delta2ns(100, evt);

	evt->cpumask = cpumask_of(0);

	clockevents_register_device(evt);
}

static struct sc58x_gptimer *sc58x_timer_of_init(struct device_node *node)
{
	void __iomem *base;
	int irq;
	int id;
	struct sc58x_gptimer *timer = NULL;

	id = of_alias_get_id(node, "timer");
	if (id < 0)
		panic("Can't timer id");

	base = of_iomap(node, 0);
	if (!base)
		panic("Can't remap registers");

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0)
		panic("Can't parse IRQ");

	timer = kzalloc(sizeof(struct sc58x_gptimer), GFP_KERNEL);
	if (!timer) {
		pr_err("%s: no memory.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}
	timer->id = id;
	timer->io_base = base;
	timer->irq = irq;

	return timer;
}

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
void __init sc58x_timer_init(void)
{
	struct device_node *np, *clocksrc_np = NULL, *clockevent_np = NULL;

	for_each_compatible_node(np, NULL, "adi,sc58x-timer-core") {
		if (!clocksrc_np &&
			(of_alias_get_id(np, "timer") == TIMER_CLOCKSOURCE)) {
			clocksrc_np = np;
		}

		if (!clockevent_np &&
			(of_alias_get_id(np, "timer") == TIMER_CLOCKEVENT)) {
			clockevent_np = np;
		}

	}

	timer_clock = sc58x_timer_of_init(clocksrc_np);
	timer_event = sc58x_timer_of_init(clockevent_np);

	clockevent_gptmr.irq = timer_event->irq;

	cs_gptimer_init();

	sched_clock_register(gptmr_read_sched, 32, get_sclk());

	setup_irq(timer_event->irq, &gptmr_irq);
	gptmr_irq.dev_id = &clockevent_gptmr;
	gptmr_clockevent_init(&clockevent_gptmr);

}
