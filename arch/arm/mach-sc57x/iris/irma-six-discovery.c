#include <linux/of.h>

#if IS_ENABLED(CONFIG_SOC_CAMERA_EPC660)
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/portmux-sc57x.h>
#endif

static int __init irma_six_disco_init(void)
{

#if IS_ENABLED(CONFIG_SOC_CAMERA_EPC660)
	/* enable the EPC660 device */

#define REG_PORTB_FER_CLR 0x31004088
#define REG_PORTB_DIR_SET 0x3100409C
#define REG_PORTB_DATA_SET 0x31004090
#define REG_PORTB_DATA_CLR 0x31004094

	// set pin PB_04 to gpio mode
	writel(0x00000010, __io_address(REG_PORTB_FER_CLR));

	// set pin PB_04 to output
	writel(0x00000010, __io_address(REG_PORTB_DIR_SET));

	// RESET
	writel(0x00000010, __io_address(REG_PORTB_DATA_CLR));
	msleep(7);;
	writel(0x00000010, __io_address(REG_PORTB_DATA_SET));
#endif
	return 0;
}

core_initcall(irma_six_disco_init);
