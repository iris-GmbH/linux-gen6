/*
 * SRAM allocator for ADI processor on-chip memory
 *
 * Copyright 2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/genalloc.h>
#include <linux/slab.h>
#include <linux/genalloc.h>
#include <linux/interrupt.h>

#include <mach/cpu.h>
#include <mach/sc58x.h>
#include <mach/sram.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#include <asm/io.h>

struct sram_size {
	struct rb_node node;
	void *addr;
	size_t size;
};

static struct rb_root sram_size_tree = RB_ROOT;
static struct kmem_cache *sram_size_cache;
static struct gen_pool *sram_pool;

struct sram_size *sram_size_search(struct rb_root *root, const void *addr)
{
	struct rb_node *node = root->rb_node;

	while (node) {
		struct sram_size *data = container_of(node, struct sram_size, node);

		if (addr < data->addr)
			node = node->rb_left;
		else if (addr >= data->addr + data->size)
			node = node->rb_right;
		else
			return data;
	}
	return NULL;
}

int sram_size_insert(struct rb_root *root, struct sram_size *data)
{
	struct rb_node **new = &(root->rb_node), *parent = NULL;

	/* Figure out where to put new node */
	while (*new) {
		struct sram_size *this = container_of(*new, struct sram_size, node);

		parent = *new;
		if (data->addr < this->addr)
			new = &((*new)->rb_left);
		else if (data->addr >= this->addr + this->size)
			new = &((*new)->rb_right);
		else
			return false;
	}

	/* Add new node and rebalance tree. */
	rb_link_node(&data->node, parent, new);
	rb_insert_color(&data->node, root);

	return true;
}

void *sram_alloc(size_t size)
{
	unsigned long vaddr;
	struct sram_size *data;

	if (!sram_pool)
		return NULL;

	data = kmem_cache_alloc(sram_size_cache, GFP_ATOMIC);
	if (!data)
		return NULL;

	vaddr = gen_pool_alloc(sram_pool, size);
	if (!vaddr) {
		kmem_cache_free(sram_size_cache, data);
		return NULL;
	}

	data->addr = (void *)vaddr;
	data->size = size;
	sram_size_insert(&sram_size_tree, data);

	return (void *)vaddr;
}
EXPORT_SYMBOL(sram_alloc);

int sram_free(const void *addr)
{
	struct sram_size *data = sram_size_search(&sram_size_tree, addr);

	if (data && data->addr == addr) {
		rb_erase(&data->node, &sram_size_tree);
		gen_pool_free(sram_pool, (unsigned long) addr, data->size);
		kmem_cache_free(sram_size_cache, data);

		return 0;
	}

	return -EFAULT;
}
EXPORT_SYMBOL(sram_free);

static irqreturn_t sram_ecc_err(int irq, void *dev_id)
{
	int status;

	pr_err("SRAM ECC error happened\n");
	status = ioread32(__io_address(L2CTL0_STAT));
	pr_err("status 0x%x ctl %x\n", status,
			ioread32(__io_address(L2CTL0_CTL)));

	if (status & 0x1)
		printk(KERN_ERR "Core channel error type:0x%x, addr:0x%x\n",
			ioread32(__io_address(L2CTL0_ET0)),
			ioread32(__io_address(L2CTL0_EADDR0)));
	if (status & 0x2)
		printk(KERN_ERR "System channel error type:0x%x, addr:0x%x\n",
			ioread32(__io_address(L2CTL0_ET1)),
			ioread32(__io_address(L2CTL0_EADDR1)));

	status = status >> 8;
	if (status)
		pr_err("SRAM Bank%d error, addr:0x%x\n", status,
			ioread32(__io_address((L2CTL0_ERRADDR0 + status))));
	panic("Can't recover from the SRAM ECC error.");

	return IRQ_HANDLED;
}

static int __init sram_init(void)
{
	int ret = 0;
	void __iomem *addr;
	int size = SYS_SRAM_SIZE - SYS_SRAM_ICC_SIZE;

	if (size <= 0)
		return -ENOMEM;

	memset(__io_address(SYS_SRAM_BASE), 0, SYS_SRAM_SIZE);
	iowrite32(ioread32(__io_address(L2CTL0_STAT)),
			__io_address(L2CTL0_STAT));

	sram_pool = gen_pool_create(ilog2(SRAM_GRANULARITY), -1);
	if (!sram_pool)
		return -ENOMEM;

	sram_size_cache = kmem_cache_create("sram_piece_cache",
		sizeof(struct sram_size), 0, SLAB_PANIC, NULL);
	if (!sram_size_cache) {
		ret = -ENOMEM;
		goto error_out;
	}

	addr = __io_address(SYS_SRAM_BASE + SYS_SRAM_ICC_SIZE);
	if (!addr) {
		ret = -ENOMEM;
		goto error_out;
	}

	ret = gen_pool_add_virt(sram_pool, (unsigned long) addr,
			SYS_SRAM_BASE + SYS_SRAM_ICC_SIZE, size, -1);
	if (ret < 0)
		goto error_out;

	pr_info("SRAM available: 0x%x - 0x%x\n", (unsigned int)addr,
		(unsigned int)(addr + SYS_SRAM_SIZE));

	ret = request_irq(IRQ_L2CTL0_ECC_ERR, sram_ecc_err, 0,
			"sram-ecc-err", NULL);
	if (unlikely(ret < 0)) {
		pr_err("Fail to request SRAM ECC error interrupt.\n");
	}

	return ret;

error_out:
	if (sram_pool)
		gen_pool_destroy(sram_pool);
	if (sram_size_cache)
		kmem_cache_destroy(sram_size_cache);

	return ret;
}
core_initcall(sram_init);

