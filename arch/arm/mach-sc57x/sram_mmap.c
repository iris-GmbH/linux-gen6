/*
 * SRAM mmap misc driver for ADI processor on-chip memory
 *
 * Copyright 2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <mach/cpu.h>
#include <mach/hardware.h>
#include <mach/sram.h>
#include <mach/sc57x.h>

void mmap_open(struct vm_area_struct *vma)
{
	void *vaddr;

	vaddr = sram_alloc(vma->vm_end - vma->vm_start);
	if (!vaddr) {
		pr_warn("sram alloc failed\n");
		return;
	}
	vma->vm_private_data = vaddr;
}

void mmap_close(struct vm_area_struct *vma)
{
	sram_free(vma->vm_private_data);
}

struct vm_operations_struct sram_mmap_vm_ops = {
	.open =     mmap_open,
	.close =    mmap_close,
};

static int sram_mmap(struct file *fp, struct vm_area_struct *vma)
{
	void *vaddr, *sram_base;
	unsigned long phy;
	size_t size = vma->vm_end - vma->vm_start;

	vma->vm_private_data = NULL;
	vma->vm_ops = &sram_mmap_vm_ops;
	vma->vm_ops->open(vma);

	vaddr = vma->vm_private_data;
	sram_base = __io_address(SYS_SRAM_BASE + SYS_SRAM_ICC_SIZE);
	phy = vaddr - sram_base + (SYS_SRAM_BASE + SYS_SRAM_ICC_SIZE);

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (!vma->vm_private_data)
		return -EAGAIN;

	if (io_remap_pfn_range(vma, vma->vm_start,
				__phys_to_pfn(phy), size, vma->vm_page_prot)) {
		pr_warn("sram mmap ERROR\n");
		return -EAGAIN;
	}

	return 0;
}

static const struct file_operations sram_fops = {
	.mmap		= sram_mmap,
};

static struct miscdevice sram_if_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "sram_mmap",
	.fops	= &sram_fops
};

static int __init sram_if_init(void)
{
	return misc_register(&sram_if_dev);
}
module_init(sram_if_init);
