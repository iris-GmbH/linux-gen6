/*
 * Copyright (C) 2014 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#ifndef __MACH_SRAM_H
#define __MACH_SRAM_H

/* ARBITRARY:  SRAM allocations are multiples of this 2^N size */
#define SRAM_GRANULARITY	512

extern void *sram_alloc(size_t size);
extern int sram_free(const void *addr);

#endif /* __MACH_SRAM_H */
