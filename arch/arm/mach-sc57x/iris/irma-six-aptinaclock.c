#if IS_ENABLED(CONFIG_VIDEO_IRIS_GEN6_APTINA_MT9V022_CAPTURE)
#include <linux/dma-mapping.h>
#include <linux/clocksource.h>
#include <mach/hardware.h>
#include <mach/cpu.h>
#include <mach/clkdev.h>

#include "../core.h"

static struct sc57x_gptimer *aptina_clock;

// Aptina Clock Setup
void __init setup_aptina_clock(struct sc57x_gptimer *timer)
{
	int id = timer->id;
	disable_gptimers(1 << id);
	set_gptimer_config(timer, TIMER_MODE_PWM_CONT | TIMER_PULSE_HI | TIMER_IRQ_PER);
	// this will divide the 112MHz of SYS0 by f  */
	// see ADSP-SC57x SHARC Processor Hardware Reference: Continuous PWMOUT Mode
	set_gptimer_period(timer, 0x00000005); // -> 112MHz / 0x5 = ~22,5MHz (EPPI produce errors for higher clockrate)
	set_gptimer_pwidth(timer, 0x00000002);
	enable_gptimers(1 << id);
}

static cycle_t read_aptina_clock(struct clocksource *cs)
{
	return (cycle_t)get_gptimer_count(aptina_clock);
}

static struct clocksource aptina_gptimer = {
	.name		= "aptina_clock",
	// unsure what rating to pick but 350 works for now
	.rating		= 350,
	.read		= read_aptina_clock,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int __init aptina_clock_init(void)
{
	setup_aptina_clock(aptina_clock);
	if (clocksource_register_hz(&aptina_gptimer, get_sclk())){
	  panic("failed to register aptina clocksource");
	}
	return 0;
}
// end Setup Aptina Clock



static int __init irma_six_aptina_init(void)
{	
	struct device_node *np_apt, *clocksrc_np_apt = NULL;

	// lookup the aptina clock in the device tree, init it
	for_each_compatible_node(np_apt, NULL, "adi,sc57x-timer,aptina-clk") {
		if (!clocksrc_np_apt) {
			clocksrc_np_apt = np_apt;
		}
	}

	if(clocksrc_np_apt != NULL){
		aptina_clock = sc57x_timer_of_init(clocksrc_np_apt);
		aptina_clock_init();
	}else{
		pr_info("Could not setup clock for Aptina Imager\n");
	}

#define REG_PORTB_DIR_SET 0x3100409C
#define REG_PORTB_FER_SET 0x31004084
#define REG_PORTB_MUX 0x310040B0

	writel(0x00000002, __io_address(REG_PORTB_FER_SET));
	writel(0x00000002, __io_address(REG_PORTB_DIR_SET));
	writel((readl(__io_address(REG_PORTB_MUX)) | 0x0000000C), __io_address(REG_PORTB_MUX));

	return 0;
}

core_initcall(irma_six_aptina_init);

#endif // CONFIG_VIDEO_IRIS_GEN6_APTINA_MT9V022_CAPTURE


