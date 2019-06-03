/*
 * core timer and machine init for ADI processor on-chip memory
 *
 * Copyright 2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __ASM_ARCH_SC57X_H
#define __ASM_ARCH_SC57X_H

#include <linux/of_platform.h>
#include <linux/reboot.h>

extern void __init sc57x_init(void);
extern void __init sc57x_init_early(void);
extern void __init sc57x_init_irq(void);
extern void __init sc57x_map_io(void);
extern void sc57x_timer_init(void);
extern struct sc57x_gptimer *sc57x_timer_of_init(struct device_node *node);
extern void sc57x_restart(enum reboot_mode, const char *);
extern void sc57x_clock_init(void);
extern void __init setup_clock(struct sc57x_gptimer *timer, u_long freq);

#endif
