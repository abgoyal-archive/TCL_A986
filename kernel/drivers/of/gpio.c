

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <asm/prom.h>

int of_get_gpio_flags(struct device_node *np, int index,
		      enum of_gpio_flags *flags)
{
	int ret;
	struct device_node *gc;
	struct of_gpio_chip *of_gc = NULL;
	int size;
	const void *gpio_spec;
	const __be32 *gpio_cells;

	ret = of_parse_phandles_with_args(np, "gpios", "#gpio-cells", index,
					  &gc, &gpio_spec);
	if (ret) {
		pr_debug("%s: can't parse gpios property\n", __func__);
		goto err0;
	}

	of_gc = gc->data;
	if (!of_gc) {
		pr_debug("%s: gpio controller %s isn't registered\n",
			 np->full_name, gc->full_name);
		ret = -ENODEV;
		goto err1;
	}

	gpio_cells = of_get_property(gc, "#gpio-cells", &size);
	if (!gpio_cells || size != sizeof(*gpio_cells) ||
			be32_to_cpup(gpio_cells) != of_gc->gpio_cells) {
		pr_debug("%s: wrong #gpio-cells for %s\n",
			 np->full_name, gc->full_name);
		ret = -EINVAL;
		goto err1;
	}

	/* .xlate might decide to not fill in the flags, so clear it. */
	if (flags)
		*flags = 0;

	ret = of_gc->xlate(of_gc, np, gpio_spec, flags);
	if (ret < 0)
		goto err1;

	ret += of_gc->gc.base;
err1:
	of_node_put(gc);
err0:
	pr_debug("%s exited with status %d\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL(of_get_gpio_flags);

unsigned int of_gpio_count(struct device_node *np)
{
	unsigned int cnt = 0;

	do {
		int ret;

		ret = of_parse_phandles_with_args(np, "gpios", "#gpio-cells",
						  cnt, NULL, NULL);
		/* A hole in the gpios = <> counts anyway. */
		if (ret < 0 && ret != -EEXIST)
			break;
	} while (++cnt);

	return cnt;
}
EXPORT_SYMBOL(of_gpio_count);

int of_gpio_simple_xlate(struct of_gpio_chip *of_gc, struct device_node *np,
			 const void *gpio_spec, enum of_gpio_flags *flags)
{
	const __be32 *gpio = gpio_spec;
	const u32 n = be32_to_cpup(gpio);

	/*
	 * We're discouraging gpio_cells < 2, since that way you'll have to
	 * write your own xlate function (that will have to retrive the GPIO
	 * number and the flags from a single gpio cell -- this is possible,
	 * but not recommended).
	 */
	if (of_gc->gpio_cells < 2) {
		WARN_ON(1);
		return -EINVAL;
	}

	if (n > of_gc->gc.ngpio)
		return -EINVAL;

	if (flags)
		*flags = be32_to_cpu(gpio[1]);

	return n;
}
EXPORT_SYMBOL(of_gpio_simple_xlate);

int of_mm_gpiochip_add(struct device_node *np,
		       struct of_mm_gpio_chip *mm_gc)
{
	int ret = -ENOMEM;
	struct of_gpio_chip *of_gc = &mm_gc->of_gc;
	struct gpio_chip *gc = &of_gc->gc;

	gc->label = kstrdup(np->full_name, GFP_KERNEL);
	if (!gc->label)
		goto err0;

	mm_gc->regs = of_iomap(np, 0);
	if (!mm_gc->regs)
		goto err1;

	gc->base = -1;

	if (!of_gc->xlate)
		of_gc->xlate = of_gpio_simple_xlate;

	if (mm_gc->save_regs)
		mm_gc->save_regs(mm_gc);

	np->data = of_gc;

	ret = gpiochip_add(gc);
	if (ret)
		goto err2;

	/* We don't want to lose the node and its ->data */
	of_node_get(np);

	pr_debug("%s: registered as generic GPIO chip, base is %d\n",
		 np->full_name, gc->base);
	return 0;
err2:
	np->data = NULL;
	iounmap(mm_gc->regs);
err1:
	kfree(gc->label);
err0:
	pr_err("%s: GPIO chip registration failed with status %d\n",
	       np->full_name, ret);
	return ret;
}
EXPORT_SYMBOL(of_mm_gpiochip_add);
