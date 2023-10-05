/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <asm/exception.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>
#include <mach/map.h>
#include <mach/irqs.h>

/* Put the bank and irq (32 bits) into the hwirq */
#define MAKE_HWIRQ(b, n)	(bank_pre_irqs[(b)] + (n))
#define HWIRQ_BANK(i)		(((i) < bank_pre_irqs[1]) ? (0):(((i) < bank_pre_irqs[2]) ? 1:2))
#define HWIRQ_BIT(i)		BIT(bank_pre_irqs[HWIRQ_BANK(i)] - (i))

#define BANK0_HWIRQ_MASK	(~((~0UL) << bank_irqs[0]))

#define ZMP_IRQ_MASK				(ZMP_VA_SYSCTRL 	+ 0x24)
#define ZMP_FIQ_MASK				(ZMP_VA_SYSCTRL 	+ 0x28)
#define	ZMP_SYSCTRL_INT_MASK		(ZMP_VA_SYSCTRL 	+ 0x2C)
#define	ZMP_SYSCTRL_INT_STATUS		(ZMP_VA_SYSCTRL 	+ 0x30)
#define ZMP_INT_STATUS				(ZMP_VA_SYSCTRL 	+ 0x4C)
#define ZMP_L2MEM_IRQ_ENABLE		(ZMP_VA_L2CTRL 	+ 0x9C)

#define NR_BANKS		2
static int bank_irqs[NR_BANKS];
static int bank_pre_irqs[NR_BANKS];


#define ZMP_SYS_IRQ_MAX_NUB		20


struct zmp_irqchip_intc {
	struct irq_domain *domain;
	void __iomem *base;
};

static struct zmp_irqchip_intc intc __read_mostly;

/*
 * Disable interrupt number "irq"
 */
static void zmp_mask_irq(struct irq_data *d)
{
	unsigned long regval;
	regval = __raw_readl(ZMP_IRQ_MASK);
	regval &= ~(1UL << d->irq);
	__raw_writel(regval, ZMP_IRQ_MASK);
	
	pr_debug("zmp_mask_irq irq=%u, hwirq=%lu\n", d->irq, d->hwirq);
}

/*
 * Enable interrupt number "irq"
 */
static void zmp_unmask_irq(struct irq_data *d)
{
	unsigned long regval;

	regval = __raw_readl(ZMP_IRQ_MASK);
	regval |= (1UL << d->irq);
	__raw_writel(regval, ZMP_IRQ_MASK);

	pr_debug("zmp_unmask_irq irq=%u, hwirq=%lu\n", d->irq, d->hwirq);
}

static struct irq_chip zmp_irq_chip = {
	.name = "module-irq",
	.irq_mask_ack = zmp_mask_irq,
	.irq_mask = zmp_mask_irq,
	.irq_unmask = zmp_unmask_irq,
};

static void sysctrl_mask_irq(struct irq_data *d)
{
	unsigned long regval;

	regval = __raw_readl(ZMP_SYSCTRL_INT_MASK);
	regval &= ~(1 << (d->irq - (bank_irqs[0]+1)));
	__raw_writel(regval, ZMP_SYSCTRL_INT_MASK);

	pr_debug("sysctrl_mask_irq irq=%u, hwirq=%lu\n", d->irq, d->hwirq);
}

static void sysctrl_unmask_irq(struct irq_data *d)
{
	unsigned long regval;

	regval = __raw_readl(ZMP_SYSCTRL_INT_MASK);
	regval |= (1 << (d->irq - (bank_irqs[0]+1) ));
	__raw_writel(regval, ZMP_SYSCTRL_INT_MASK);

	pr_debug("sysctrl_unmask_irq irq=%u, hwirq=%lu\n", d->irq, d->hwirq);
}

/* enable rtc alarm to wake up systerm */
static int sysctrl_set_wake(struct irq_data *d, unsigned int on)
{
	pr_info("%s,%d\n",__func__,__LINE__);
	// todo
	return 0;
}

static struct irq_chip zmp_sysctrl_chip = {
	.name = "sysctrl-irq",
	.irq_mask_ack = sysctrl_mask_irq,
	.irq_mask = sysctrl_mask_irq,
	.irq_unmask = sysctrl_unmask_irq,
	.irq_set_wake = sysctrl_set_wake,
};

/**
 * @brief: system module irq handler
 * 
 * @author: caolianming
 * @param [in] irq: irq number
 * @param [in] *desc: irq info description
 */
static void zmp_sysctrl_handler(struct irq_desc *desc)
{
	unsigned long regval_mask, regval_sta;
	unsigned long intpnd;
	unsigned int offset;
	u32 irq;
	
	regval_mask = __raw_readl(ZMP_SYSCTRL_INT_MASK);
	regval_sta = __raw_readl(ZMP_SYSCTRL_INT_STATUS);

    /*Beware!  H3D chip System irq number is 20.  */
    intpnd = (regval_mask & 0xFFFFF) & (regval_sta & 0xFFFFF);	

    /*Beware!  H3D chip System irq number is 20, but H3B is 16.  */
	for (offset = 0; intpnd && offset < ZMP_SYS_IRQ_MAX_NUB; offset++) {

		if (intpnd & (1 << offset))
			intpnd &= ~(1 << offset);
		else
			continue;
		irq = bank_irqs[0] + 1 + offset;
		
		generic_handle_irq(irq);
	}
}

static int int_xlate(struct irq_domain *d, struct device_node *ctrlr,
	const u32 *intspec, unsigned int intsize,
	unsigned long *out_hwirq, unsigned int *out_type)
{
	if (WARN_ON(intsize != 2))
		return -EINVAL;

	if (WARN_ON(intspec[1] >= NR_BANKS))
		return -EINVAL;

	if (WARN_ON(intspec[0] > bank_irqs[intspec[1]]))  
		return -EINVAL;

	*out_hwirq = MAKE_HWIRQ(intspec[1], intspec[0]);
	*out_type = IRQ_TYPE_NONE;
	
	return 0;
}

static const struct irq_domain_ops int_ops = {
	.xlate = int_xlate
};

static int __init zmp_int_of_init(struct device_node *node,
					  struct device_node *parent)
{
	int b, i;
	int irq;

	intc.base = ZMP_VA_SYSCTRL;
	/* 1st, clear all interrupts */
	__raw_readl(ZMP_INT_STATUS);
	__raw_readl(ZMP_SYSCTRL_INT_STATUS);

	/* 2nd, mask all interrutps */
	__raw_writel(0x0, ZMP_IRQ_MASK);
	__raw_writel(0x0, ZMP_FIQ_MASK);
	__raw_writel(0x0, ZMP_SYSCTRL_INT_MASK);

	/* mask all l2 interrupts */
	__raw_writel(0x0, ZMP_L2MEM_IRQ_ENABLE);

	b = 0;
	for (i = 0; i < NR_BANKS; i++) {
		bank_pre_irqs[i] = b;
		if(of_property_read_u32_index(node, "bank_irqs", i,&bank_irqs[i]) < 0)
		{
			pr_err("%s: unable to get bank_irqs\n", node->full_name);
			return -1;
		}
		//pr_err("bank_irqs[%d]=%d\n", i, bank_irqs[i]);	
		b += bank_irqs[i];
	}
	
	intc.domain = irq_domain_add_linear(node, bank_irqs[0]+bank_irqs[1]+1,
			&int_ops, NULL);
	if (!intc.domain)
		pr_err("%s: unable to create IRQ domain\n", node->full_name);

	for (b = 0; b < NR_BANKS; b++) {
		for (i = 1; i <= bank_irqs[b]; i++) {
			irq = irq_create_mapping(intc.domain, MAKE_HWIRQ(b, i));
			pr_debug("i = %d irq = %d\n", i, irq);
			BUG_ON(irq <= 0);
			if (b == 0)
				irq_set_chip_and_handler(irq, &zmp_irq_chip, handle_level_irq);
			else if (b == 1)
				irq_set_chip_and_handler(irq, &zmp_sysctrl_chip, handle_level_irq);
			
			irq_set_probe(irq);
		}
	}
	
	//pr_err("zmp_int_of_init success\n");
	
	return 0;
}

static int __init zmp_sysint_of_init(struct device_node *node,
		struct device_node *parent)
{
 	int parent_irq = irq_of_parse_and_map(node, 0);
	if (!parent_irq) {
		panic("%s: unable to get parent interrupt.\n",
			    node->full_name);
	}
	//pr_err("get parent interrupt = %d\n", parent_irq);

	irq_set_chained_handler(parent_irq, zmp_sysctrl_handler);
	
	//pr_err("zmp_sysint_of_init success\n");

	return 0;
}

IRQCHIP_DECLARE(zmp_irqchip, "zlgmcu,zmp110x-ic",
				zmp_int_of_init);
IRQCHIP_DECLARE(zmp_sysirqchip, "zlgmcu,zmp110x-system-ic",
				zmp_sysint_of_init); 	
