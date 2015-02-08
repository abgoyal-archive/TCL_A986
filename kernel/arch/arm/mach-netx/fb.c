

#include <linux/device.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/err.h>
#include <linux/gfp.h>

#include <asm/irq.h>

#include <mach/netx-regs.h>
#include <mach/hardware.h>

static struct clcd_panel *netx_panel;

void netx_clcd_enable(struct clcd_fb *fb)
{
}

int netx_clcd_setup(struct clcd_fb *fb)
{
	dma_addr_t dma;

	fb->panel = netx_panel;

	fb->fb.screen_base = dma_alloc_writecombine(&fb->dev->dev, 1024*1024,
						    &dma, GFP_KERNEL);
	if (!fb->fb.screen_base) {
		printk(KERN_ERR "CLCD: unable to map framebuffer\n");
		return -ENOMEM;
	}

	fb->fb.fix.smem_start	= dma;
	fb->fb.fix.smem_len	= 1024*1024;

	return 0;
}

int netx_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
	return dma_mmap_writecombine(&fb->dev->dev, vma,
				     fb->fb.screen_base,
				     fb->fb.fix.smem_start,
				     fb->fb.fix.smem_len);
}

void netx_clcd_remove(struct clcd_fb *fb)
{
	dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
			      fb->fb.screen_base, fb->fb.fix.smem_start);
}

void clk_disable(struct clk *clk)
{
}

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	return 0;
}

int clk_enable(struct clk *clk)
{
	return 0;
}

struct clk *clk_get(struct device *dev, const char *id)
{
	return dev && strcmp(dev_name(dev), "fb") == 0 ? NULL : ERR_PTR(-ENOENT);
}

void clk_put(struct clk *clk)
{
}

static struct amba_device fb_device = {
	.dev		= {
		.init_name = "fb",
		.coherent_dma_mask = ~0,
	},
	.res		= {
		.start	= 0x00104000,
		.end	= 0x00104fff,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { NETX_IRQ_LCD, NO_IRQ },
	.periphid	= 0x10112400,
};

int netx_fb_init(struct clcd_board *board, struct clcd_panel *panel)
{
	netx_panel = panel;
	fb_device.dev.platform_data = board;
	return amba_device_register(&fb_device, &iomem_resource);
}
