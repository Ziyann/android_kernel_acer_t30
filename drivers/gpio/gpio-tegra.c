/*
 * arch/arm/mach-tegra/gpio.c
 *
 * Copyright (c) 2010 Google, Inc
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
 *
 * Copyright (c) 2011-2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscore_ops.h>
#include <linux/irqdomain.h>

#include <asm/mach/irq.h>

#include <mach/gpio-tegra.h>
#include <mach/iomap.h>
#include <mach/legacy_irq.h>
#include <mach/pinmux.h>

#include "../../arch/arm/mach-tegra/pm-irq.h"
#if defined(CONFIG_ARCH_ACER_T30)
#include "../../../arch/arm/mach-tegra/gpio-names.h"
#include "../../../arch/arm/mach-tegra/board-acer-t30.h"
#endif

#define GPIO_BANK(x)		((x) >> 5)
#define GPIO_PORT(x)		(((x) >> 3) & 0x3)
#define GPIO_BIT(x)		((x) & 0x7)

#define GPIO_REG(x)		(GPIO_BANK(x) * tegra_gpio_bank_stride + \
					GPIO_PORT(x) * 4)

#define GPIO_CNF(x)		(GPIO_REG(x) + 0x00)
#define GPIO_OE(x)		(GPIO_REG(x) + 0x10)
#define GPIO_OUT(x)		(GPIO_REG(x) + 0X20)
#define GPIO_IN(x)		(GPIO_REG(x) + 0x30)
#define GPIO_INT_STA(x)		(GPIO_REG(x) + 0x40)
#define GPIO_INT_ENB(x)		(GPIO_REG(x) + 0x50)
#define GPIO_INT_LVL(x)		(GPIO_REG(x) + 0x60)
#define GPIO_INT_CLR(x)		(GPIO_REG(x) + 0x70)

#if defined(CONFIG_ARCH_ACER_T30)
#define GPIO_MSK_CNF(x)		(GPIO_REG(x) + 0x80)
#define GPIO_MSK_OE(x)		(GPIO_REG(x) + 0x90)
#define GPIO_MSK_OUT(x)		(GPIO_REG(x) + 0XA0)
#define GPIO_MSK_INT_STA(x)	(GPIO_REG(x) + 0xC0)
#define GPIO_MSK_INT_ENB(x)	(GPIO_REG(x) + 0xD0)
#define GPIO_MSK_INT_LVL(x)	(GPIO_REG(x) + 0xE0)
#else
#define GPIO_MSK_CNF(x)		(GPIO_REG(x) + tegra_gpio_upper_offset + 0x00)
#define GPIO_MSK_OE(x)		(GPIO_REG(x) + tegra_gpio_upper_offset + 0x10)
#define GPIO_MSK_OUT(x)		(GPIO_REG(x) + tegra_gpio_upper_offset + 0X20)
#define GPIO_MSK_INT_STA(x)	(GPIO_REG(x) + tegra_gpio_upper_offset + 0x40)
#define GPIO_MSK_INT_ENB(x)	(GPIO_REG(x) + tegra_gpio_upper_offset + 0x50)
#define GPIO_MSK_INT_LVL(x)	(GPIO_REG(x) + tegra_gpio_upper_offset + 0x60)
#endif

#define GPIO_INT_LVL_MASK		0x010101
#define GPIO_INT_LVL_EDGE_RISING	0x000101
#define GPIO_INT_LVL_EDGE_FALLING	0x000100
#define GPIO_INT_LVL_EDGE_BOTH		0x010100
#define GPIO_INT_LVL_LEVEL_HIGH		0x000001
#define GPIO_INT_LVL_LEVEL_LOW		0x000000

#define	BANK_MASK	0b111
#define	OE_MASK		0b11
#define	GPIOBIT_MASK	0b111

extern int acer_sku;

struct tegra_gpio_bank {
	int bank;
	int irq;
	spinlock_t lvl_lock[4];
#ifdef CONFIG_PM_SLEEP
	u32 cnf[4];
	u32 out[4];
	u32 oe[4];
	u32 int_enb[4];
	u32 int_lvl[4];
	u32 wake_enb[4];
#if defined(CONFIG_ARCH_ACER_T30)
	u32 in[4];
#endif
#endif
};

#if defined(CONFIG_ARCH_ACER_T30)
static struct tegra_gpio_bank tegra_gpio_sleep_banks[] = {
	{.bank = 0, .irq = INT_GPIO1},
	{.bank = 1, .irq = INT_GPIO2},
	{.bank = 2, .irq = INT_GPIO3},
	{.bank = 3, .irq = INT_GPIO4},
	{.bank = 4, .irq = INT_GPIO5},
	{.bank = 5, .irq = INT_GPIO6},
	{.bank = 6, .irq = INT_GPIO7},
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	{.bank = 7, .irq = INT_GPIO8},
#endif
};
#endif

static struct irq_domain *irq_domain;
static void __iomem *regs;

static u32 tegra_gpio_bank_count;
static u32 tegra_gpio_bank_stride;
static u32 tegra_gpio_upper_offset;
static struct tegra_gpio_bank *tegra_gpio_banks;

static inline void tegra_gpio_writel(u32 val, u32 reg)
{
	__raw_writel(val, regs + reg);
}

static inline u32 tegra_gpio_readl(u32 reg)
{
	return __raw_readl(regs + reg);
}

static int tegra_gpio_compose(int bank, int port, int bit)
{
	return (bank << 5) | ((port & 0x3) << 3) | (bit & 0x7);
}

void tegra_gpio_set_tristate(int gpio_nr, enum tegra_tristate ts)
{
	int pin_group  =  tegra_pinmux_get_pingroup(gpio_nr);
	tegra_pinmux_set_tristate(pin_group, ts);
}

static void tegra_gpio_mask_write(u32 reg, int gpio, int value)
{
	u32 val;

	val = 0x100 << GPIO_BIT(gpio);
	if (value)
		val |= 1 << GPIO_BIT(gpio);
	tegra_gpio_writel(val, reg);
}

int tegra_gpio_get_bank_int_nr(int gpio)
{
	int bank;
	int irq;
	if (gpio >= TEGRA_NR_GPIOS) {
		pr_warn("%s : Invalid gpio ID - %d\n", __func__, gpio);
		return -EINVAL;
	}
	bank = gpio >> 5;
	irq = tegra_gpio_banks[bank].irq;
	return irq;
}

void tegra_gpio_enable(int gpio)
{
	if (gpio >= TEGRA_NR_GPIOS) {
		pr_warn("%s : Invalid gpio ID - %d\n", __func__, gpio);
		return;
	}
	tegra_gpio_mask_write(GPIO_MSK_CNF(gpio), gpio, 1);
}
EXPORT_SYMBOL_GPL(tegra_gpio_enable);

int tegra_is_gpio(int gpio)
{
	if (gpio >= TEGRA_NR_GPIOS) {
		pr_warn("%s : Invalid gpio ID - %d\n", __func__, gpio);
		return 0;
	}
	return (tegra_gpio_readl(GPIO_CNF(gpio)) >> GPIO_BIT(gpio)) & 0x1;
}
EXPORT_SYMBOL(tegra_is_gpio);


void tegra_gpio_disable(int gpio)
{
	if (gpio >= TEGRA_NR_GPIOS) {
		pr_warn("%s : Invalid gpio ID - %d\n", __func__, gpio);
		return;
	}
	tegra_gpio_mask_write(GPIO_MSK_CNF(gpio), gpio, 0);
}
EXPORT_SYMBOL_GPL(tegra_gpio_disable);

void tegra_gpio_init_configure(unsigned gpio, bool is_input, int value)
{
	if (gpio >= TEGRA_NR_GPIOS) {
		pr_warn("%s : Invalid gpio ID - %d\n", __func__, gpio);
		return;
	}
	if (is_input) {
		tegra_gpio_mask_write(GPIO_MSK_OE(gpio), gpio, 0);
	} else {
		tegra_gpio_mask_write(GPIO_MSK_OUT(gpio), gpio, value);
		tegra_gpio_mask_write(GPIO_MSK_OE(gpio), gpio, 1);
	}
	tegra_gpio_mask_write(GPIO_MSK_CNF(gpio), gpio, 1);
}

static void tegra_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	tegra_gpio_mask_write(GPIO_MSK_OUT(offset), offset, value);
}

static int tegra_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	if ((tegra_gpio_readl(GPIO_OE(offset)) >> GPIO_BIT(offset)) & 0x1)
		return (tegra_gpio_readl(GPIO_OUT(offset)) >>
			GPIO_BIT(offset)) & 0x1;
	return (tegra_gpio_readl(GPIO_IN(offset)) >> GPIO_BIT(offset)) & 0x1;
}

static int tegra_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	tegra_gpio_mask_write(GPIO_MSK_OE(offset), offset, 0);
	tegra_gpio_enable(offset);
	return 0;
}

static int tegra_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					int value)
{
	tegra_gpio_set(chip, offset, value);
	tegra_gpio_mask_write(GPIO_MSK_OE(offset), offset, 1);
	tegra_gpio_enable(offset);
	return 0;
}

static int tegra_gpio_set_debounce(struct gpio_chip *chip, unsigned offset,
				unsigned debounce)
{
	return -ENOSYS;
}

static int tegra_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return irq_find_mapping(irq_domain, offset);
}

static int tegra_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

static void tegra_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	tegra_gpio_disable(offset);
}

static struct gpio_chip tegra_gpio_chip = {
	.label			= "tegra-gpio",
	.request		= tegra_gpio_request,
	.free			= tegra_gpio_free,
	.direction_input	= tegra_gpio_direction_input,
	.get			= tegra_gpio_get,
	.direction_output	= tegra_gpio_direction_output,
	.set			= tegra_gpio_set,
	.set_debounce		= tegra_gpio_set_debounce,
	.to_irq			= tegra_gpio_to_irq,
	.base			= 0,
	.ngpio			= TEGRA_NR_GPIOS,
};

static void tegra_gpio_irq_ack(struct irq_data *d)
{
	int gpio = d->hwirq;

	tegra_gpio_writel(1 << GPIO_BIT(gpio), GPIO_INT_CLR(gpio));

#ifdef CONFIG_TEGRA_FPGA_PLATFORM
	/* FPGA platforms have a serializer between the GPIO
	   block and interrupt controller. Allow time for
	   clearing of the GPIO interrupt to propagate to the
	   interrupt controller before re-enabling the IRQ
	   to prevent double interrupts. */
	udelay(15);
#endif
}

static void tegra_gpio_irq_mask(struct irq_data *d)
{
	int gpio = d->hwirq;

	tegra_gpio_mask_write(GPIO_MSK_INT_ENB(gpio), gpio, 0);
}

static void tegra_gpio_irq_unmask(struct irq_data *d)
{
	int gpio = d->hwirq;

	tegra_gpio_mask_write(GPIO_MSK_INT_ENB(gpio), gpio, 1);
}

static int tegra_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	int gpio = d->hwirq;
	struct tegra_gpio_bank *bank = irq_data_get_irq_chip_data(d);
	int port = GPIO_PORT(gpio);
	int lvl_type;
	int val;
	unsigned long flags;
	int wake = tegra_gpio_to_wake(d->hwirq);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		lvl_type = GPIO_INT_LVL_EDGE_RISING;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		lvl_type = GPIO_INT_LVL_EDGE_FALLING;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		lvl_type = GPIO_INT_LVL_EDGE_BOTH;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		lvl_type = GPIO_INT_LVL_LEVEL_HIGH;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		lvl_type = GPIO_INT_LVL_LEVEL_LOW;
		break;

	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&bank->lvl_lock[port], flags);

	val = tegra_gpio_readl(GPIO_INT_LVL(gpio));
	val &= ~(GPIO_INT_LVL_MASK << GPIO_BIT(gpio));
	val |= lvl_type << GPIO_BIT(gpio);
	tegra_gpio_writel(val, GPIO_INT_LVL(gpio));

	spin_unlock_irqrestore(&bank->lvl_lock[port], flags);

	tegra_gpio_mask_write(GPIO_MSK_OE(gpio), gpio, 0);
	tegra_gpio_enable(gpio);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		__irq_set_handler_locked(d->irq, handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		__irq_set_handler_locked(d->irq, handle_edge_irq);

	tegra_pm_irq_set_wake_type(wake, type);

	return 0;
}

static void tegra_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct tegra_gpio_bank *bank;
	int port;
	int pin;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	bank = irq_get_handler_data(irq);

	for (port = 0; port < 4; port++) {
		int gpio = tegra_gpio_compose(bank->bank, port, 0);
		unsigned long sta = tegra_gpio_readl(GPIO_INT_STA(gpio)) &
			tegra_gpio_readl(GPIO_INT_ENB(gpio));

		for_each_set_bit(pin, &sta, 8)
			generic_handle_irq(gpio_to_irq(gpio + pin));
	}

	chained_irq_exit(chip, desc);

}

#ifdef CONFIG_PM_SLEEP
void tegra_gpio_resume(void)
{
	unsigned long flags;
	int b;
	int p;

	local_irq_save(flags);

	for (b = 0; b < tegra_gpio_bank_count; b++) {
		struct tegra_gpio_bank *bank = &tegra_gpio_banks[b];

		for (p = 0; p < ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			tegra_gpio_writel(bank->cnf[p], GPIO_CNF(gpio));
			tegra_gpio_writel(bank->out[p], GPIO_OUT(gpio));
#if defined(CONFIG_ARCH_ACER_T30)
			tegra_gpio_writel(bank->in[p], GPIO_IN(gpio));
#endif
			tegra_gpio_writel(bank->oe[p], GPIO_OE(gpio));
			tegra_gpio_writel(bank->int_lvl[p], GPIO_INT_LVL(gpio));
			tegra_gpio_writel(bank->int_enb[p], GPIO_INT_ENB(gpio));
		}
	}

	local_irq_restore(flags);
}

int tegra_gpio_suspend(void)
{
	unsigned long flags;
	int b;
	int p;

	local_irq_save(flags);
	for (b = 0; b < tegra_gpio_bank_count; b++) {
		struct tegra_gpio_bank *bank = &tegra_gpio_banks[b];

		for (p = 0; p < ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			bank->cnf[p] = tegra_gpio_readl(GPIO_CNF(gpio));
			bank->out[p] = tegra_gpio_readl(GPIO_OUT(gpio));
#if defined(CONFIG_ARCH_ACER_T30)
			bank->in[p] = tegra_gpio_readl(GPIO_IN(gpio));
#endif
			bank->oe[p] = tegra_gpio_readl(GPIO_OE(gpio));
			bank->int_enb[p] = tegra_gpio_readl(GPIO_INT_ENB(gpio));
			bank->int_lvl[p] = tegra_gpio_readl(GPIO_INT_LVL(gpio));

			/* disable gpio interrupts that are not wake sources */
			tegra_gpio_writel(bank->wake_enb[p], GPIO_INT_ENB(gpio));
		}
	}
	local_irq_restore(flags);

	return 0;
}

static int tegra_update_lp1_gpio_wake(struct irq_data *d, bool enable)
{
#ifdef CONFIG_PM_SLEEP
	struct tegra_gpio_bank *bank = irq_data_get_irq_chip_data(d);
	u8 mask;
	u8 port_index;
	u8 pin_index_in_bank;
	u8 pin_in_port;
	int gpio = d->hwirq;

	if (gpio < 0)
		return -EIO;
	pin_index_in_bank = (gpio & 0x1F);
	port_index = pin_index_in_bank >> 3;
	pin_in_port = (pin_index_in_bank & 0x7);
	mask = BIT(pin_in_port);
	if (enable)
		bank->wake_enb[port_index] |= mask;
	else
		bank->wake_enb[port_index] &= ~mask;
#endif

	return 0;
}

static int tegra_gpio_irq_set_wake(struct irq_data *d, unsigned int enable)
{
	struct tegra_gpio_bank *bank = irq_data_get_irq_chip_data(d);
	int ret = 0;
	int wake = tegra_gpio_to_wake(d->hwirq);

	/*
	 * update LP1 mask for gpio port/pin interrupt
	 * LP1 enable independent of LP0 wake support
	 */
	ret = tegra_update_lp1_gpio_wake(d, enable);
	if (ret) {
		pr_err("Failed gpio lp1 %s for irq=%d, error=%d\n",
			(enable ? "enable" : "disable"), d->irq, ret);
		goto fail;
	}

	/* LP1 enable for bank interrupt */
	ret = tegra_update_lp1_irq_wake(bank->irq, enable);
	if (ret)
		pr_err("Failed gpio lp1 %s for irq=%d, error=%d\n",
			(enable ? "enable" : "disable"), bank->irq, ret);

	if (wake < 0)
		pr_err("Warning: enabling a non-LP0 wake source %lu\n",
			d->hwirq);
	else {
		ret = tegra_pm_irq_set_wake(wake, enable);
		if (ret)
			pr_err("Failed gpio lp0 %s for irq=%d, error=%d\n",
				(enable ? "enable" : "disable"), d->irq, ret);
	}

fail:
	return ret;
}
#else
#define tegra_gpio_irq_set_wake NULL
#define tegra_update_lp1_gpio_wake NULL
#define tegra_gpio_suspend NULL
#define tegra_gpio_resume NULL
#endif

static struct syscore_ops tegra_gpio_syscore_ops = {
	.suspend = tegra_gpio_suspend,
	.resume = tegra_gpio_resume,
};

int tegra_gpio_resume_init(void)
{
	register_syscore_ops(&tegra_gpio_syscore_ops);
	return 0;
}

static struct irq_chip tegra_gpio_irq_chip = {
	.name		= "GPIO",
	.irq_ack	= tegra_gpio_irq_ack,
	.irq_mask	= tegra_gpio_irq_mask,
	.irq_unmask	= tegra_gpio_irq_unmask,
	.irq_set_type	= tegra_gpio_irq_set_type,
	.irq_set_wake	= tegra_gpio_irq_set_wake,
	.flags		= IRQCHIP_MASK_ON_SUSPEND,
};

struct tegra_gpio_soc_config {
	u32 bank_stride;
	u32 upper_offset;
};

static struct tegra_gpio_soc_config tegra20_gpio_config = {
	.bank_stride = 0x80,
	.upper_offset = 0x800,
};

static struct tegra_gpio_soc_config tegra30_gpio_config = {
	.bank_stride = 0x100,
	.upper_offset = 0x80,
};

static struct of_device_id tegra_gpio_of_match[] __devinitdata = {
	{ .compatible = "nvidia,tegra30-gpio", .data = &tegra30_gpio_config },
	{ .compatible = "nvidia,tegra20-gpio", .data = &tegra20_gpio_config },
	{ },
};

/* This lock class tells lockdep that GPIO irqs are in a different
 * category than their parents, so it won't report false recursion.
 */
static struct lock_class_key gpio_lock_class;

static int __devinit tegra_gpio_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct tegra_gpio_soc_config *config;
	int irq_base;
	struct resource *res;
	struct tegra_gpio_bank *bank;
	int gpio;
	int i;
	int j;

	match = of_match_device(tegra_gpio_of_match, &pdev->dev);
	if (match)
		config = (struct tegra_gpio_soc_config *)match->data;
	else
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
		config = &tegra20_gpio_config;
#else
		config = &tegra30_gpio_config;
#endif

	tegra_gpio_bank_stride = config->bank_stride;
	tegra_gpio_upper_offset = config->upper_offset;

	for (;;) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, tegra_gpio_bank_count);
		if (!res)
			break;
		tegra_gpio_bank_count++;
	}
	if (!tegra_gpio_bank_count) {
		dev_err(&pdev->dev, "Missing IRQ resource\n");
		return -ENODEV;
	}

	tegra_gpio_chip.ngpio = tegra_gpio_bank_count * 32;

	tegra_gpio_banks = devm_kzalloc(&pdev->dev,
			tegra_gpio_bank_count * sizeof(*tegra_gpio_banks),
			GFP_KERNEL);
	if (!tegra_gpio_banks) {
		dev_err(&pdev->dev, "Couldn't allocate bank structure\n");
		return -ENODEV;
	}

	irq_base = irq_alloc_descs(-1, 0, tegra_gpio_chip.ngpio, 0);
	if (irq_base < 0) {
		dev_err(&pdev->dev, "Couldn't allocate IRQ numbers\n");
		return -ENODEV;
	}

	irq_domain = irq_domain_add_legacy(pdev->dev.of_node,
					   tegra_gpio_chip.ngpio, irq_base, 0,
					   &irq_domain_simple_ops, NULL);

	for (i = 0; i < tegra_gpio_bank_count; i++) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, i);
		if (!res) {
			dev_err(&pdev->dev, "Missing IRQ resource\n");
			return -ENODEV;
		}

		bank = &tegra_gpio_banks[i];
		bank->bank = i;
		bank->irq = res->start;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing MEM resource\n");
		return -ENODEV;
	}

	regs = devm_request_and_ioremap(&pdev->dev, res);
	if (!regs) {
		dev_err(&pdev->dev, "Couldn't ioremap regs\n");
		return -ENODEV;
	}

	for (i = 0; i < tegra_gpio_bank_count; i++) {
		for (j = 0; j < 4; j++) {
			int gpio = tegra_gpio_compose(i, j, 0);
			tegra_gpio_writel(0x00, GPIO_INT_ENB(gpio));
			tegra_gpio_writel(0x00, GPIO_INT_STA(gpio));
		}
	}

#ifdef CONFIG_OF_GPIO
	tegra_gpio_chip.of_node = pdev->dev.of_node;
#endif

	gpiochip_add(&tegra_gpio_chip);

	for (gpio = 0; gpio < tegra_gpio_chip.ngpio; gpio++) {
		int irq = irq_find_mapping(irq_domain, gpio);
		/* No validity check; all Tegra GPIOs are valid IRQs */

		bank = &tegra_gpio_banks[GPIO_BANK(gpio)];

		irq_set_lockdep_class(irq, &gpio_lock_class);
		irq_set_chip_data(irq, bank);
		irq_set_chip_and_handler(irq, &tegra_gpio_irq_chip,
					 handle_simple_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	for (i = 0; i < tegra_gpio_bank_count; i++) {
		bank = &tegra_gpio_banks[i];

		for (j = 0; j < 4; j++)
			spin_lock_init(&bank->lvl_lock[j]);

		irq_set_handler_data(bank->irq, bank);
		irq_set_chained_handler(bank->irq, tegra_gpio_irq_handler);

	}

	return 0;
}

static struct platform_driver tegra_gpio_driver = {
	.driver		= {
		.name	= "tegra-gpio",
		.owner	= THIS_MODULE,
		.of_match_table = tegra_gpio_of_match,
	},
	.probe		= tegra_gpio_probe,
};

static int __init tegra_gpio_init(void)
{
	return platform_driver_register(&tegra_gpio_driver);
}
postcore_initcall(tegra_gpio_init);

void tegra_gpio_config(struct tegra_gpio_table *table, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		int gpio = table[i].gpio;

		if (table[i].enable)
			tegra_gpio_enable(gpio);
		else
			tegra_gpio_disable(gpio);
	}
}

#ifdef	CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int dbg_gpio_show(struct seq_file *s, void *unused)
{
	int i;
	int j;

#if defined(CONFIG_ARCH_ACER_T30)
	int k;

	seq_printf(s, "GPIO Current State:\n------------------------------------------\n");
	seq_printf(s, "GPIO CNF OE OUT IN INT_STA INT_ENB INT_LVL\n");
	for (i = 0; i < tegra_gpio_bank_count; i++) {
		struct tegra_gpio_bank *bank = &tegra_gpio_banks[i];

		for (j = 0; j < ARRAY_SIZE(bank->oe); j++) {
			for (k = 0; k < 8; k++) {
				unsigned int gpio = (i<<5) | (j<<3) | k;
				seq_printf(s,
					" %3d   %1x  %1x   %1x  %1x       %1x       %1x       %1x\n",
					gpio,
					(tegra_gpio_readl(GPIO_CNF(gpio)) >> GPIO_BIT(gpio)) & 0x1,
					(tegra_gpio_readl(GPIO_OE(gpio)) >> GPIO_BIT(gpio)) & 0x1,
					(tegra_gpio_readl(GPIO_OUT(gpio)) >> GPIO_BIT(gpio)) & 0x1,
					(tegra_gpio_readl(GPIO_IN(gpio)) >> GPIO_BIT(gpio)) & 0x1,
					(tegra_gpio_readl(GPIO_INT_STA(gpio)) >> GPIO_BIT(gpio)) & 0x1,
					(tegra_gpio_readl(GPIO_INT_ENB(gpio)) >> GPIO_BIT(gpio)) & 0x1,
					(tegra_gpio_readl(GPIO_INT_LVL(gpio)) >> GPIO_BIT(gpio)) & 0x1);
			}
		}
	}
#else
	seq_printf(s, "Bank:Port CNF OE OUT IN INT_STA INT_ENB INT_LVL\n");
	for (i = 0; i < tegra_gpio_bank_count; i++) {
		for (j = 0; j < 4; j++) {
			int gpio = tegra_gpio_compose(i, j, 0);
			seq_printf(s,
				"%d:%d %02x %02x %02x %02x %02x %02x %06x\n",
				i, j,
				tegra_gpio_readl(GPIO_CNF(gpio)),
				tegra_gpio_readl(GPIO_OE(gpio)),
				tegra_gpio_readl(GPIO_OUT(gpio)),
				tegra_gpio_readl(GPIO_IN(gpio)),
				tegra_gpio_readl(GPIO_INT_STA(gpio)),
				tegra_gpio_readl(GPIO_INT_ENB(gpio)),
				tegra_gpio_readl(GPIO_INT_LVL(gpio)));
		}
	}
#endif

	return 0;
}

static int dbg_gpio_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_gpio_show, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_gpio_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#if defined(CONFIG_ARCH_ACER_T30)
static int dbg_gpio_sleep_show(struct seq_file *s, void *unused)
{
	int i, j, k;

	seq_printf(s, "GPIO Sleep State:\n------------------------------------------\n");
	seq_printf(s, "GPIO CNF OE OUT IN INT_ENB INT_LVL\n");
	for (i = 0; i < ARRAY_SIZE(tegra_gpio_sleep_banks); i++) {
		struct tegra_gpio_bank *bank = &tegra_gpio_sleep_banks[i];

		for (j = 0; j < ARRAY_SIZE(bank->oe); j++) {
			for (k = 0; k < 8; k++) {
				unsigned int gpio = (i<<5) | (j<<3) | k;
				seq_printf(s,
					" %3d   %1x  %1x   %1x  %1x       %1x       %1x\n",
					gpio,
					(bank->cnf[j] >> GPIO_BIT(gpio)) & 0x1,
					(bank->oe[j] >> GPIO_BIT(gpio)) & 0x1,
					(bank->out[j] >> GPIO_BIT(gpio)) & 0x1,
					(bank->in[j] >> GPIO_BIT(gpio)) & 0x1,
					(bank->int_enb[j] >> GPIO_BIT(gpio)) & 0x1,
					(bank->int_lvl[j] >> GPIO_BIT(gpio)) & 0x1);
			}
		}
	}

	return 0;
}

int acer_gpio_sleep_table_store(void)
{
	unsigned long flags;
	int b;
	int p;

	local_irq_save(flags);
	for (b = 0; b < ARRAY_SIZE(tegra_gpio_sleep_banks); b++) {
		struct tegra_gpio_bank *bank = &tegra_gpio_sleep_banks[b];

		for (p = 0; p < ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			bank->cnf[p] = __raw_readl(GPIO_CNF(gpio));
			bank->out[p] = __raw_readl(GPIO_OUT(gpio));
			bank->in[p] = __raw_readl(GPIO_IN(gpio));
			bank->oe[p] = __raw_readl(GPIO_OE(gpio));
			bank->int_enb[p] = __raw_readl(GPIO_INT_ENB(gpio));
			bank->int_lvl[p] = __raw_readl(GPIO_INT_LVL(gpio));
		}
	}

	local_irq_restore(flags);
        return 0;
}


static int dbg_gpio_sleep_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_gpio_sleep_show, &inode->i_private);
}

static const struct file_operations debug_sleep_fops = {
	.open		= dbg_gpio_sleep_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

// GPIO configuration
#if defined(CONFIG_ARCH_ACER_T30)
static struct gpio_table gpio_unused_init_table[] = {
	GPIO_CONFIG(NULL, TEGRA_GPIO_PK6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SPDIF_IN
	GPIO_CONFIG(NULL, TEGRA_GPIO_PK5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SPDIF_OUT
	GPIO_CONFIG(NULL, TEGRA_GPIO_PX6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SPI1_CS0_N
	GPIO_CONFIG(NULL, TEGRA_GPIO_PX4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SPI1_MOSI
	GPIO_CONFIG(NULL, TEGRA_GPIO_PX5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SPI1_SCK
	GPIO_CONFIG(NULL, TEGRA_GPIO_PX2, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SPI2_SCK
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PO1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // ULPI_DATA0 (DEBUG_UART1_TX)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PO3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // ULPI_DATA2
	GPIO_CONFIG(NULL, TEGRA_GPIO_PO6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // ULPI_DATA5
	GPIO_CONFIG(NULL, TEGRA_GPIO_PO7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // ULPI_DATA6
	GPIO_CONFIG(NULL, TEGRA_GPIO_PO0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // ULPI_DATA7
	GPIO_CONFIG(NULL, TEGRA_GPIO_PCC1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GPIO_PCC1
	GPIO_CONFIG(NULL, TEGRA_GPIO_PCC2, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GPIO_PCC2
	GPIO_CONFIG(NULL, TEGRA_GPIO_PV6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // CRT_HSYNC
	GPIO_CONFIG(NULL, TEGRA_GPIO_PV7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // CRT_VSYNC
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PN4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_CS0_N
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PW0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_CS1_N
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PN6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_DC0
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PD2, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_DC1
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PW1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_M1
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PB2, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_PWR0
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PC1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_PWR1
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PZ4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_SCK
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PZ2, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_SDIN
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PN5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_SDOUT
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PZ3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_WR_N
	GPIO_CONFIG(NULL, TEGRA_GPIO_PCC7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // PEX_L2_CLKREQ_N
	GPIO_CONFIG(NULL, TEGRA_GPIO_PDD7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // PEX_L2_PRSNT_N
	GPIO_CONFIG(NULL, TEGRA_GPIO_PCC6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // PEX_L2_RST_N
	GPIO_CONFIG(NULL, TEGRA_GPIO_PDD3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // PEX_WAKE_N
	GPIO_CONFIG(NULL, TEGRA_GPIO_PCC5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // CLK2_REQ
	GPIO_CONFIG(NULL, TEGRA_GPIO_PV2, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GPIO_PV2
	GPIO_CONFIG(NULL, TEGRA_GPIO_PV3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GPIO_PV3
	GPIO_CONFIG(NULL, TEGRA_GPIO_PD1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC3_DAT4
	GPIO_CONFIG(NULL, TEGRA_GPIO_PD0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC3_DAT5
	GPIO_CONFIG(NULL, TEGRA_GPIO_PD3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC3_DAT6
	GPIO_CONFIG(NULL, TEGRA_GPIO_PD4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC3_DAT7
	GPIO_CONFIG(NULL, TEGRA_GPIO_PQ5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // KB_COL5
	GPIO_CONFIG(NULL, TEGRA_GPIO_PQ6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // KB_COL6
	GPIO_CONFIG(NULL, TEGRA_GPIO_PQ7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // KB_COL7
	GPIO_CONFIG(NULL, TEGRA_GPIO_PR3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // KB_ROW3
	GPIO_CONFIG(NULL, TEGRA_GPIO_PR4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // KB_ROW4
	GPIO_CONFIG(NULL, TEGRA_GPIO_PR6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // KB_ROW6
#if !defined(CONFIG_ARCH_ACER_T30)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PR7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // KB_ROW7
#endif
	GPIO_CONFIG(NULL, TEGRA_GPIO_PS1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // KB_ROW9
	GPIO_CONFIG(NULL, TEGRA_GPIO_PS3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // KB_ROW11
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PK5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // OWR
	GPIO_CONFIG(NULL, TEGRA_GPIO_PZ5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SYS_CLK_REQ
	GPIO_CONFIG(NULL, TEGRA_GPIO_PEE1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // CLK3_REQ
	GPIO_CONFIG(NULL, TEGRA_GPIO_PU4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GPIO_PU4
	GPIO_CONFIG(NULL, TEGRA_GPIO_PK2, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), //GMI_CS4_N (POUT_3G_1)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PI6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), //GMI_CS7_N (POUT_WIFI)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PI5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), //GMI_IORDY (POUT_3G)
};

static struct gpio_table gpio_wifi_sku_table[] = {
	GPIO_CONFIG(NULL, TEGRA_GPIO_PI7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GMI_WAIT (3G_DISABLE#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PBB7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GPIO_PBB7 (3G_DISABLE2#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PO5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), //ULPI_DATA4 (SIM_DET)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PC7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GMI_WP_N
	GPIO_CONFIG(NULL, TEGRA_GPIO_PU1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GPIO_PU1
};

static struct gpio_table gpio_3G_sku_table[] = {
	GPIO_CONFIG(NULL, TEGRA_GPIO_PI7,  GPIO_ENABLE, GPIO_HIGH, GPIO_OUTPUT), // GMI_WAIT  (3G_DISABLE#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PBB7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GPIO_PBB7 (3G_DISABLE2#)
};

static struct gpio_table gpio_sleep_init_table[] = {
	GPIO_CONFIG(NULL, TEGRA_GPIO_PW4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // CLK1_OUT (AUDIO_CLK)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PN1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // DAP1_DIN (ENABLE_USB_HOST)
#if !defined(CONFIG_ACER_ES305)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PN0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // DAP1_FS (AUDIO_RST#)
#endif
	GPIO_CONFIG(NULL, TEGRA_GPIO_PN3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // DAP1_SCLK (AUDIO_SEL)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PA4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // DAP2_DIN (AUDIO_DIN2)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PA5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // DAP2_DOUT (AUDIO_DOUT2)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PA2, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // DAP2_FS (AUDIO_FS2)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PA3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // DAP2_SCLK (AUDIO_SCLK2)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PX0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SPI2_MOSI (EN_ES305_OSC)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PP1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // DAP3_DIN (CAM_LED_EN_NV)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PY0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // ULPI_CLK (AUDIO_UART4_TX)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PO1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // ULPI_DATA0 (DEBUG_UART1_TX)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PO2, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), // ULPI_DATA1 (DEBUG_UART1_RX)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PY1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // ULPI_DIR (AUDIO_UART4_RX)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PCC0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // CAM_MCLK (CAM_MCLK)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PJ7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GMI_A16 (VIB_EN_T30S)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PB1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GMI_A18 (EN_VDDLCD_T30S)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PH2, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GMI_AD10 (EN_T30S_FUSE_3V3)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PH3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GMI_AD11 (LCD_DCR)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PH0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GMI_AD8 (LCD_PWM_OUT)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PH1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GMI_AD9 (DISPOFF#)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PI4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // GMI_RST_N (EN_HDMI_5V0)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PI7, GPIO_ENABLE, GPIO_HIGH, GPIO_OUTPUT), // GMI_WAIT (3G_DISABLE#)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PE0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D0 (LCD_D00)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PE1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D1 (LCD_D01)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PF2, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D10 (LCD_D10)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PF3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D11 (LCD_D11)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PF4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D12 (LCD_D12)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PF5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D13 (LCD_D13)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PF6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D14 (LCD_D14)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PF7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D15 (LCD_D15)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PM0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D16 (LCD_D16)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PM1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D17 (LCD_D17)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PM2, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D18 (LCD_D18)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PM3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D19 (LCD_D19)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PE2, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D2 (LCD_D02)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PM4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D20 (LCD_D20)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PM5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D21 (LCD_D21)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PM6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D22 (LCD_D22)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PM7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D23 (LCD_D23)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PE3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D3 (LCD_D03)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PE4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D4 (LCD_D04)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PE5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D5 (LCD_D05)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PE6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D6 (LCD_D06)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PE7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D7 (LCD_D07)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PF0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D8 (LCD_D08)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PF1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_D9 (LCD_D09)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PJ1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_DE (LCD_DE)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PJ3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_HSYNC (LCD_HSYNC)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PB3, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_PCLK (NV_LCD_PCLK)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PJ4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // LCD_VSYNC (LCD_VSYNC)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PW5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // CLK2_OUT (CP_GPIO)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PZ0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC1_CLK (SDMMC_CLK)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PZ1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC1_CMD (SDMMC_CMD)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PY7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC1_DAT0 (SDMMC_DAT0)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PY6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC1_DAT1 (SDMMC_DAT1)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PY5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC1_DAT2 (SDMMC_DAT2)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PY4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC1_DAT3 (SDMMC_DAT3)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PA6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC3_CLK (WFMMC_CLK)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PA7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC3_CMD (WFMMC_CMD)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PB7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC3_DAT0 (WFMMC_DAT0)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PB6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC3_DAT1 (WFMMC_DAT1)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PB5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC3_DAT2 (WFMMC_DAT2)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PB4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC3_DAT3 (WFMMC_DAT3)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PCC4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // SDMMC4_CLK (EMMC_CLK)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PT7, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), // SDMMC4_CMD (EMMC_CMD)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PAA0, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), // SDMMC4_DAT0 (EMMC_DA0)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PAA1, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), // SDMMC4_DAT1 (EMMC_DA1)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PAA2, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), // SDMMC4_DAT2 (EMMC_DA2)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PAA3, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), // SDMMC4_DAT3 (EMMC_DA3)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PAA4, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), // SDMMC4_DAT4 (EMMC_DA4)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PAA5, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), // SDMMC4_DAT5 (EMMC_DA5)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PAA6, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), // SDMMC4_DAT6 (EMMC_DA6)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PAA7, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), // SDMMC4_DAT7 (EMMC_DA7)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PCC3, GPIO_ENABLE, GPIO_HIGH, GPIO_OUTPUT), // SDMMC4_RST_N (EMMC_RST#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PA0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // CLK_32K_OUT (CLK_32K_OUT)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PW4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // HDMI_CEC (HDMI_CEC)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PQ4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // KB_COL4 (BOARD_ID_WP)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PR5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // KB_ROW5 (UART_SW)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PEE0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // CLK3_OUT (CLK_12M_ES305)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PP5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // DAP4_DIN (BT_PCM_IN)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PP6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // DAP4_DOUT (BT_PCM_OUT)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PP4, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // DAP4_FS (BT_PCM_SYNC)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PP7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // DAP4_SCLK (BT_PCM_CLK)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PJ5, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // UART2_CTS_N (GPS_UART_CTS#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PJ6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // UART2_RTS_N (GPS_UART_RTS#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PC3, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), // UART2_RXD (GPS_UART_RXD )
	GPIO_CONFIG(NULL, TEGRA_GPIO_PC2, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // UART2_TXD (GPS_UART_RXD)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PA1, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // UART3_CTS_N (BT_UART_CTS#)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PC0, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // UART3_RTS_N (BT_UART_RTS#)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PW7, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // UART3_RXD (BT_UART_RXD)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PW6, GPIO_ENABLE, GPIO_LOW, GPIO_OUTPUT), // UART3_TXD (BT_UART_TXD)

	// Input pins
	GPIO_CONFIG(NULL, TEGRA_GPIO_PN2, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //DAP1_DOUT (ES305_INT_R)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PX7, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //SPI1_MISO (COMPASS_DRDY)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PX3, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //SPI2_CS0_N (LIGHT_INT)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PW2, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //SPI2_CS1_N (HP_DET#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PW3, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //SPI2_CS2_N (CDC_IRQ#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PX1, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //SPI2_MISO (GYRO_INT_R)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PV0, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GPIO_PV0 (ON_KEY#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PV1, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GPIO_PV1 (WAKE_UP_VBUS)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PO4, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //ULPI_DATA3 (ULPI_DATA3)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PO5, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //ULPI_DATA4 (SIM_DET)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PBB6, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GPIO_PBB6 (DOCK_DET#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PG0, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_AD0 (NAND_D0)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PG1, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_AD1 (BOOT_PD)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PG2, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_AD2 (BOOT_PD)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PG3, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_AD3 (BOOT_PD)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PG4, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_AD4 (NAND_D4)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PG5, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_AD5 (NAND_D5)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PG6, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_AD6 (NAND_D6)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PG7, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_AD7 (NAND_D7)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PH4, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_AD12 (PCB_ID0)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PH5, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_AD13 (PCB_ID1)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PH6, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_AD14 (PCB_ID2)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PK0, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_ADV_N (BOOT_PD)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PK1, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_CLK (BOOT_PD)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PJ0, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_CS0_N (TS_INT#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PJ2, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_CS1_N (CHARGER_STAT)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PJ3, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_CS2_N (BOARD_ID0)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PJ4, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_CS3_N (BOARD_ID1)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PI3, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_CS6_N (TEMP_ALERT#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PI1, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_OE_N (FORCE_RECOVERY#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PC7, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_WP_N (GMI_WP_N)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PI0, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GMI_WR_N (NOR_BOOT)
	//GPIO_CONFIG(NULL, TEGRA_GPIO_PN7, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //HDMI_INT (HDMI_DET_T30S)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PQ0, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //KB_COL0 (SC_LOCK#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PQ1, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //KB_COL1 (VOL_DOWN#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PQ2, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //KB_COL2 (VOL_UP#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PR1, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //KB_ROW1 (BOARD_ID3)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PS0, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //KB_ROW8 (SHORT_DET)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PS2, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //KB_ROW10 (WF_WAKE#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PS4, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //KB_ROW12 (SD_DET#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PS5, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //KB_ROW13 (G_ACC_INT)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PS6, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //KB_ROW14 (LINE_OUT_DET#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PS7, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //KB_ROW15 (BT_IRQ#)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PU5, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GPIO_PU5 (DEBUG_UART1_RX_R)

	// I2C pins
	GPIO_CONFIG(NULL, TEGRA_GPIO_PBB1, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //CAM_I2C_SCL (CAM_I2C_SCL)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PBB2, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //CAM_I2C_SDA (CAM_I2C_SDA)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PT5, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GEN2_I2C_SCL (GEN2_I2C_SCL)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PT6, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GEN2_I2C_SDA (GEN2_I2C_SDA)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PV4, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //DDC_SCL (DDC_SCL_R)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PV5, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //DDC_SDA (DDC_SDA_R)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PZ6, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //PWR_I2C_SCL (PWR_I2C_SCL)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PZ7, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //PWR_I2C_SDA (PWR_I2C_SDA)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PC4, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GEN1_I2C_SCL (GEN1_I2C_SCL)
	GPIO_CONFIG(NULL, TEGRA_GPIO_PC5, GPIO_ENABLE, GPIO_LOW, GPIO_INPUT), //GEN1_I2C_SCL (GEN1_I2C_SCL)
};

void gpio_unused_init(void)
{
	int i;
	int ret;
	unsigned long flags;

#if defined(CONFIG_ARCH_ACER_T30)
	printk("%s\n", __func__);
#endif

	local_irq_save(flags);
		for (i = 0; i < ARRAY_SIZE(gpio_unused_init_table); i++) {
			if (gpio_unused_init_table[i].enabled) {
				gpio_request(gpio_unused_init_table[i].gpio, gpio_unused_init_table[i].name);
				if (gpio_unused_init_table[i].direction == GPIO_OUTPUT) {
					ret = gpio_direction_output(gpio_unused_init_table[i].gpio, gpio_unused_init_table[i].value);
					if(ret)
						pr_err("---%s:  GPIO(%d)  init error\n", __func__, gpio_unused_init_table[i].gpio);
					else
						tegra_gpio_enable(gpio_unused_init_table[i].gpio);
				}
				else if (gpio_unused_init_table[i].direction == GPIO_INPUT) {
					gpio_direction_input(gpio_unused_init_table[i].gpio);
					tegra_gpio_enable(gpio_unused_init_table[i].gpio);
				}
				else
					pr_err("%s : GPIO neither output nor input \n", __func__);
			}
		}

	if (acer_sku == BOARD_SKU_WIFI) { // Wifi
		for (i = 0; i < ARRAY_SIZE(gpio_wifi_sku_table); i++) {
			if (gpio_wifi_sku_table[i].enabled) {
				gpio_request(gpio_wifi_sku_table[i].gpio, gpio_wifi_sku_table[i].name);
				if (gpio_wifi_sku_table[i].direction == GPIO_OUTPUT) {
					ret = gpio_direction_output(gpio_wifi_sku_table[i].gpio, gpio_wifi_sku_table[i].value);
					if(ret)
						pr_err("---%s:  GPIO(%d)  init error\n", __func__, gpio_wifi_sku_table[i].gpio);
					else
						tegra_gpio_enable(gpio_wifi_sku_table[i].gpio);
				}
				else if (gpio_wifi_sku_table[i].direction == GPIO_INPUT) {
					gpio_direction_input(gpio_wifi_sku_table[i].gpio);
					tegra_gpio_enable(gpio_wifi_sku_table[i].gpio);
				}
				else
					pr_err("%s : GPIO neither output nor input \n", __func__);
			}
		}
	}
	else {  // 3G/LTE
		for (i = 0; i < ARRAY_SIZE(gpio_3G_sku_table); i++) {
			if (gpio_3G_sku_table[i].enabled) {
				gpio_request(gpio_3G_sku_table[i].gpio, gpio_3G_sku_table[i].name);
				if (gpio_3G_sku_table[i].direction == GPIO_OUTPUT) {
					ret = gpio_direction_output(gpio_3G_sku_table[i].gpio, gpio_3G_sku_table[i].value);
					if(ret)
						pr_err("---%s:  GPIO(%d)  init error\n", __func__, gpio_3G_sku_table[i].gpio);
					else
						tegra_gpio_enable(gpio_3G_sku_table[i].gpio);
				}
				else if (gpio_3G_sku_table[i].direction == GPIO_INPUT) {
					gpio_direction_input(gpio_3G_sku_table[i].gpio);
					tegra_gpio_enable(gpio_3G_sku_table[i].gpio);
				}
				else
					pr_err("%s : GPIO neither output nor input \n", __func__);
			}
		}
	}
	local_irq_restore(flags);
}

void gpio_sleep_init(void)
{
	int i;
	int value;
	unsigned int banks_idx, banks_oe_idx, bit;
	unsigned long flags;

#if defined(CONFIG_ARCH_ACER_T30)
	printk("%s\n", __func__);
#endif

	local_irq_save(flags);
		for (i = 0 ; i < ARRAY_SIZE(gpio_sleep_init_table); i++) {
			if (gpio_sleep_init_table[i].enabled) {

				// get gpio index
				banks_idx = (gpio_sleep_init_table[i].gpio >> 5) & BANK_MASK;
				banks_oe_idx = (gpio_sleep_init_table[i].gpio >> 3) & OE_MASK;
				bit = gpio_sleep_init_table[i].gpio & GPIOBIT_MASK;

				// cnf
				value = __raw_readl(GPIO_CNF(gpio_sleep_init_table[i].gpio));
				value |= (0x1 << bit);
				__raw_writel(value, GPIO_CNF(gpio_sleep_init_table[i].gpio));

				// oe
				value = __raw_readl(GPIO_OE(gpio_sleep_init_table[i].gpio));
				if (gpio_sleep_init_table[i].direction == GPIO_OUTPUT)
					value |= (0x1 << bit);
				else
					value &= ~(0x1 << bit);
				__raw_writel(value, GPIO_OE(gpio_sleep_init_table[i].gpio));

				// out
				value = __raw_readl(GPIO_OUT(gpio_sleep_init_table[i].gpio));
				if (gpio_sleep_init_table[i].value == GPIO_HIGH)
					value |= (0x1 << bit);
				else
					value &= ~(0x1 << bit);
				__raw_writel(value, GPIO_OUT(gpio_sleep_init_table[i].gpio));
			}
		}
	local_irq_restore(flags);
}
#endif // GPIO configuration

static int __init tegra_gpio_debuginit(void)
{
	(void) debugfs_create_file("tegra_gpio", S_IRUGO,
					NULL, NULL, &debug_fops);
#if defined(CONFIG_ARCH_ACER_T30)
	(void) debugfs_create_file("acer_gpio_sleep_table", S_IRUGO,
					NULL, NULL, &debug_sleep_fops);
#endif
	return 0;
}
late_initcall(tegra_gpio_debuginit);
#endif
