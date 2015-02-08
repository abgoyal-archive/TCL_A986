
#include <linux/atmel_tc.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* Number of bytes to reserve for the iomem resource */
#define ATMEL_TC_IOMEM_SIZE	256



#if defined(CONFIG_AVR32)
/* AVR32 has these divide PBB */
const u8 atmel_tc_divisors[5] = { 0, 4, 8, 16, 32, };
EXPORT_SYMBOL(atmel_tc_divisors);

#elif defined(CONFIG_ARCH_AT91)
/* AT91 has these divide MCK */
const u8 atmel_tc_divisors[5] = { 2, 8, 32, 128, 0, };
EXPORT_SYMBOL(atmel_tc_divisors);

#endif

static DEFINE_SPINLOCK(tc_list_lock);
static LIST_HEAD(tc_list);

struct atmel_tc *atmel_tc_alloc(unsigned block, const char *name)
{
	struct atmel_tc		*tc;
	struct platform_device	*pdev = NULL;
	struct resource		*r;

	spin_lock(&tc_list_lock);
	list_for_each_entry(tc, &tc_list, node) {
		if (tc->pdev->id == block) {
			pdev = tc->pdev;
			break;
		}
	}

	if (!pdev || tc->iomem)
		goto fail;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	r = request_mem_region(r->start, ATMEL_TC_IOMEM_SIZE, name);
	if (!r)
		goto fail;

	tc->regs = ioremap(r->start, ATMEL_TC_IOMEM_SIZE);
	if (!tc->regs)
		goto fail_ioremap;

	tc->iomem = r;

out:
	spin_unlock(&tc_list_lock);
	return tc;

fail_ioremap:
	release_resource(r);
fail:
	tc = NULL;
	goto out;
}
EXPORT_SYMBOL_GPL(atmel_tc_alloc);

void atmel_tc_free(struct atmel_tc *tc)
{
	spin_lock(&tc_list_lock);
	if (tc->regs) {
		iounmap(tc->regs);
		release_resource(tc->iomem);
		tc->regs = NULL;
		tc->iomem = NULL;
	}
	spin_unlock(&tc_list_lock);
}
EXPORT_SYMBOL_GPL(atmel_tc_free);

static int __init tc_probe(struct platform_device *pdev)
{
	struct atmel_tc *tc;
	struct clk	*clk;
	int		irq;

	if (!platform_get_resource(pdev, IORESOURCE_MEM, 0))
		return -EINVAL;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -EINVAL;

	tc = kzalloc(sizeof(struct atmel_tc), GFP_KERNEL);
	if (!tc)
		return -ENOMEM;

	tc->pdev = pdev;

	clk = clk_get(&pdev->dev, "t0_clk");
	if (IS_ERR(clk)) {
		kfree(tc);
		return -EINVAL;
	}

	tc->clk[0] = clk;
	tc->clk[1] = clk_get(&pdev->dev, "t1_clk");
	if (IS_ERR(tc->clk[1]))
		tc->clk[1] = clk;
	tc->clk[2] = clk_get(&pdev->dev, "t2_clk");
	if (IS_ERR(tc->clk[2]))
		tc->clk[2] = clk;

	tc->irq[0] = irq;
	tc->irq[1] = platform_get_irq(pdev, 1);
	if (tc->irq[1] < 0)
		tc->irq[1] = irq;
	tc->irq[2] = platform_get_irq(pdev, 2);
	if (tc->irq[2] < 0)
		tc->irq[2] = irq;

	spin_lock(&tc_list_lock);
	list_add_tail(&tc->node, &tc_list);
	spin_unlock(&tc_list_lock);

	return 0;
}

static struct platform_driver tc_driver = {
	.driver.name	= "atmel_tcb",
};

static int __init tc_init(void)
{
	return platform_driver_probe(&tc_driver, tc_probe);
}
arch_initcall(tc_init);
