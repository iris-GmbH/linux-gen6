#include <linux/kernel.h>
#include <mach/hardware.h>
#include <mach/core.h>
#include <linux/io.h>
#include <linux/i2c.h>

/* defines for the imager clock */
#define IMAG_TMR3_ID      3
#define REG_PORTB_DIR_SET 0x3100409C
#define REG_PORTB_FER_SET 0x31004084
#define REG_PORTB_MUX     0x310040B0
#define PB_01	          (1<<1)
#define TMR3_AS_OUTPUT	  0x0000000C
#define IMAGER_EEPROM_ADDR	0x52
#define MAGIC "IRIS"

extern int sc57x_i2c_read_byte(u8 devaddr, u8 regoffset, u8 *value);

struct __attribute__ ((__packed__)) component_identification_block {
	uint8_t magic[4];
	uint8_t version;
	uint8_t flags;
	uint8_t comp_ident;
	uint8_t revision;
	uint8_t sn[6];
	uint8_t reserved[2];
};

struct __attribute__ ((__packed__)) imager_configuration_block {
	uint8_t version;
	uint8_t flags;
	uint8_t clock_freq[4];
	uint8_t reserved[10];
};

static struct __attribute__ ((__packed__)) eeprom {
	struct component_identification_block cib;
	struct imager_configuration_block icb;
} imager_eeprom;

int read_imager_eeprom (void) {
	char * p = (char*) &imager_eeprom;
	int i, length = sizeof(imager_eeprom);
	
	for (i = 0; i<length; i++) {
		if(sc57x_i2c_read_byte(IMAGER_EEPROM_ADDR, i, p+i))
			return -1;
	}
	return 0;
}


int read_clock_freq(unsigned int *freq) {
	if(read_imager_eeprom()) {
		pr_err("%s: Reading from imager eeprom fails\n", __func__);
		return -1;
	}
	
	/* validate magic bytes */
	if (strncmp(MAGIC, imager_eeprom.cib.magic, 4)!=0) {
		pr_err("%s: Imager eeprom is not initalized\n", __func__);
		return -1;
	}
	
	/* validate imager version */
	if (imager_eeprom.icb.version!=1) {
		pr_err("%s: Invalid imager_configuration_block version\n", __func__);
		return -1;
	}
	
	*freq = imager_eeprom.icb.clock_freq[3] |
			(imager_eeprom.icb.clock_freq[2] << 8) | 
			(imager_eeprom.icb.clock_freq[1] << 16) |
			(imager_eeprom.icb.clock_freq[0] << 24);
	
	return 0;
}

/*
 * imager_clock_init - initalize the timer 3 clock source with
 *                     specified frequency
 *
 * Note:
 * - The imager clock source can either be provided by an external
 * source or by timer.
 * - Please make sure, the timer 3 is setup before the imager
 */
static int __init imager_clock_init(void)
{
	struct device_node *np=NULL, *np_tmr3=NULL;
	struct sc57x_gptimer *imager_clock;
	unsigned int freq = 0;

	/* find timer 3 device node in device tree */
	for_each_compatible_node(np, NULL, "adi,sc57x-timer") {
		if(of_alias_get_id(np, "timer") == IMAG_TMR3_ID) {
			np_tmr3 = np;
		}
	}
	
	if (np_tmr3 == NULL) {
		pr_err("%s: Could not setup imager clock\n", __func__);
		return -1;
	}

	if (read_clock_freq(&freq))
		return -1;
	
	/* We can not measure up, if an external clock source is available,
	 * because the voltage level is too low. So we have to trust the
	 * imager eeprom is well programmed.
	 * -> allowed frequency is 0..50MHz
	 */
	if (freq == 0) {
		pr_info("Imager clock is externally provided\n");
		return 0;
	} else if (freq > 50000000) {
		pr_info("Requested imager clock frequency: %dHz is way to high\n", freq);
		return -1;
	}
	
	pr_info("Setup imager clock to %uHz\n", freq);

	/* configure PB01 as output and setup timer 3 */
	imager_clock = sc57x_timer_of_init(np_tmr3);
	setup_clock(imager_clock, freq);
	writel(PB_01, __io_address(REG_PORTB_FER_SET)); /* set peripheral mode */
	writel(PB_01, __io_address(REG_PORTB_DIR_SET)); /* set PB01 as output */
	writel((readl(__io_address(REG_PORTB_MUX)) | TMR3_AS_OUTPUT), __io_address(REG_PORTB_MUX));

	return 0;
}

device_initcall(imager_clock_init);
