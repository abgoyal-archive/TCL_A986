

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/vga.h>

#if 0
#define DPRINTK(args...) printk(KERN_DEBUG __FILE__": " ##args)
#else
#define DPRINTK(args...)
#endif

#if 0
#define CHKINFO(ret) if (info != &fb_info) { printk(KERN_DEBUG __FILE__": This should never happen, line:%d \n", __LINE__); return ret; }
#else
#define CHKINFO(ret)
#endif

/* Description of the hardware layout */

static void __iomem *hga_vram;			/* Base of video memory */
static unsigned long hga_vram_len;		/* Size of video memory */

#define HGA_ROWADDR(row) ((row%4)*8192 + (row>>2)*90)
#define HGA_TXT			0
#define HGA_GFX			1

static inline u8 __iomem * rowaddr(struct fb_info *info, u_int row)
{
	return info->screen_base + HGA_ROWADDR(row);
}

static int hga_mode = -1;			/* 0 = txt, 1 = gfx mode */

static enum { TYPE_HERC, TYPE_HERCPLUS, TYPE_HERCCOLOR } hga_type;
static char *hga_type_name;

#define HGA_INDEX_PORT		0x3b4		/* Register select port */
#define HGA_VALUE_PORT		0x3b5		/* Register value port */
#define HGA_MODE_PORT		0x3b8		/* Mode control port */
#define HGA_STATUS_PORT		0x3ba		/* Status and Config port */
#define HGA_GFX_PORT		0x3bf		/* Graphics control port */

/* HGA register values */

#define HGA_CURSOR_BLINKING	0x00
#define HGA_CURSOR_OFF		0x20
#define HGA_CURSOR_SLOWBLINK	0x60

#define HGA_MODE_GRAPHICS	0x02
#define HGA_MODE_VIDEO_EN	0x08
#define HGA_MODE_BLINK_EN	0x20
#define HGA_MODE_GFX_PAGE1	0x80

#define HGA_STATUS_HSYNC	0x01
#define HGA_STATUS_VSYNC	0x80
#define HGA_STATUS_VIDEO	0x08

#define HGA_CONFIG_COL132	0x08
#define HGA_GFX_MODE_EN		0x01
#define HGA_GFX_PAGE_EN		0x02

/* Global locks */

static DEFINE_SPINLOCK(hga_reg_lock);

/* Framebuffer driver structures */

static struct fb_var_screeninfo hga_default_var __devinitdata = {
	.xres		= 720,
	.yres 		= 348,
	.xres_virtual 	= 720,
	.yres_virtual	= 348,
	.bits_per_pixel = 1,
	.red 		= {0, 1, 0},
	.green 		= {0, 1, 0},
	.blue 		= {0, 1, 0},
	.transp 	= {0, 0, 0},
	.height 	= -1,
	.width 		= -1,
};

static struct fb_fix_screeninfo hga_fix __devinitdata = {
	.id 		= "HGA",
	.type 		= FB_TYPE_PACKED_PIXELS,	/* (not sure) */
	.visual 	= FB_VISUAL_MONO10,
	.xpanstep 	= 8,
	.ypanstep 	= 8,
	.line_length 	= 90,
	.accel 		= FB_ACCEL_NONE
};

/* Don't assume that tty1 will be the initial current console. */
static int release_io_port = 0;
static int release_io_ports = 0;
static int nologo = 0;


static void write_hga_b(unsigned int val, unsigned char reg)
{
	outb_p(reg, HGA_INDEX_PORT); 
	outb_p(val, HGA_VALUE_PORT);
}

static void write_hga_w(unsigned int val, unsigned char reg)
{
	outb_p(reg,   HGA_INDEX_PORT); outb_p(val >> 8,   HGA_VALUE_PORT);
	outb_p(reg+1, HGA_INDEX_PORT); outb_p(val & 0xff, HGA_VALUE_PORT);
}

static int test_hga_b(unsigned char val, unsigned char reg)
{
	outb_p(reg, HGA_INDEX_PORT); 
	outb  (val, HGA_VALUE_PORT);
	udelay(20); val = (inb_p(HGA_VALUE_PORT) == val);
	return val;
}

static void hga_clear_screen(void)
{
	unsigned char fillchar = 0xbf; /* magic */
	unsigned long flags;

	spin_lock_irqsave(&hga_reg_lock, flags);
	if (hga_mode == HGA_TXT)
		fillchar = ' ';
	else if (hga_mode == HGA_GFX)
		fillchar = 0x00;
	spin_unlock_irqrestore(&hga_reg_lock, flags);
	if (fillchar != 0xbf)
		memset_io(hga_vram, fillchar, hga_vram_len);
}

static void hga_txt_mode(void)
{
	unsigned long flags;

	spin_lock_irqsave(&hga_reg_lock, flags);
	outb_p(HGA_MODE_VIDEO_EN | HGA_MODE_BLINK_EN, HGA_MODE_PORT);
	outb_p(0x00, HGA_GFX_PORT);
	outb_p(0x00, HGA_STATUS_PORT);

	write_hga_b(0x61, 0x00);	/* horizontal total */
	write_hga_b(0x50, 0x01);	/* horizontal displayed */
	write_hga_b(0x52, 0x02);	/* horizontal sync pos */
	write_hga_b(0x0f, 0x03);	/* horizontal sync width */

	write_hga_b(0x19, 0x04);	/* vertical total */
	write_hga_b(0x06, 0x05);	/* vertical total adjust */
	write_hga_b(0x19, 0x06);	/* vertical displayed */
	write_hga_b(0x19, 0x07);	/* vertical sync pos */

	write_hga_b(0x02, 0x08);	/* interlace mode */
	write_hga_b(0x0d, 0x09);	/* maximum scanline */
	write_hga_b(0x0c, 0x0a);	/* cursor start */
	write_hga_b(0x0d, 0x0b);	/* cursor end */

	write_hga_w(0x0000, 0x0c);	/* start address */
	write_hga_w(0x0000, 0x0e);	/* cursor location */

	hga_mode = HGA_TXT;
	spin_unlock_irqrestore(&hga_reg_lock, flags);
}

static void hga_gfx_mode(void)
{
	unsigned long flags;

	spin_lock_irqsave(&hga_reg_lock, flags);
	outb_p(0x00, HGA_STATUS_PORT);
	outb_p(HGA_GFX_MODE_EN, HGA_GFX_PORT);
	outb_p(HGA_MODE_VIDEO_EN | HGA_MODE_GRAPHICS, HGA_MODE_PORT);

	write_hga_b(0x35, 0x00);	/* horizontal total */
	write_hga_b(0x2d, 0x01);	/* horizontal displayed */
	write_hga_b(0x2e, 0x02);	/* horizontal sync pos */
	write_hga_b(0x07, 0x03);	/* horizontal sync width */

	write_hga_b(0x5b, 0x04);	/* vertical total */
	write_hga_b(0x02, 0x05);	/* vertical total adjust */
	write_hga_b(0x57, 0x06);	/* vertical displayed */
	write_hga_b(0x57, 0x07);	/* vertical sync pos */

	write_hga_b(0x02, 0x08);	/* interlace mode */
	write_hga_b(0x03, 0x09);	/* maximum scanline */
	write_hga_b(0x00, 0x0a);	/* cursor start */
	write_hga_b(0x00, 0x0b);	/* cursor end */

	write_hga_w(0x0000, 0x0c);	/* start address */
	write_hga_w(0x0000, 0x0e);	/* cursor location */

	hga_mode = HGA_GFX;
	spin_unlock_irqrestore(&hga_reg_lock, flags);
}

static void hga_show_logo(struct fb_info *info)
{
}

static void hga_pan(unsigned int xoffset, unsigned int yoffset)
{
	unsigned int base;
	unsigned long flags;
	
	base = (yoffset / 8) * 90 + xoffset;
	spin_lock_irqsave(&hga_reg_lock, flags);
	write_hga_w(base, 0x0c);	/* start address */
	spin_unlock_irqrestore(&hga_reg_lock, flags);
	DPRINTK("hga_pan: base:%d\n", base);
}

static void hga_blank(int blank_mode)
{
	unsigned long flags;

	spin_lock_irqsave(&hga_reg_lock, flags);
	if (blank_mode) {
		outb_p(0x00, HGA_MODE_PORT);	/* disable video */
	} else {
		outb_p(HGA_MODE_VIDEO_EN | HGA_MODE_GRAPHICS, HGA_MODE_PORT);
	}
	spin_unlock_irqrestore(&hga_reg_lock, flags);
}

static int __devinit hga_card_detect(void)
{
	int count = 0;
	void __iomem *p, *q;
	unsigned short p_save, q_save;

	hga_vram_len  = 0x08000;

	hga_vram = ioremap(0xb0000, hga_vram_len);

	if (request_region(0x3b0, 12, "hgafb"))
		release_io_ports = 1;
	if (request_region(0x3bf, 1, "hgafb"))
		release_io_port = 1;

	/* do a memory check */

	p = hga_vram;
	q = hga_vram + 0x01000;

	p_save = readw(p); q_save = readw(q);

	writew(0xaa55, p); if (readw(p) == 0xaa55) count++;
	writew(0x55aa, p); if (readw(p) == 0x55aa) count++;
	writew(p_save, p);

	if (count != 2)
		goto error;

	/* Ok, there is definitely a card registering at the correct
	 * memory location, so now we do an I/O port test.
	 */
	
	if (!test_hga_b(0x66, 0x0f))	    /* cursor low register */
		goto error;

	if (!test_hga_b(0x99, 0x0f))     /* cursor low register */
		goto error;

	/* See if the card is a Hercules, by checking whether the vsync
	 * bit of the status register is changing.  This test lasts for
	 * approximately 1/10th of a second.
	 */
	
	p_save = q_save = inb_p(HGA_STATUS_PORT) & HGA_STATUS_VSYNC;

	for (count=0; count < 50000 && p_save == q_save; count++) {
		q_save = inb(HGA_STATUS_PORT) & HGA_STATUS_VSYNC;
		udelay(2);
	}

	if (p_save == q_save) 
		goto error;

	switch (inb_p(HGA_STATUS_PORT) & 0x70) {
		case 0x10:
			hga_type = TYPE_HERCPLUS;
			hga_type_name = "HerculesPlus";
			break;
		case 0x50:
			hga_type = TYPE_HERCCOLOR;
			hga_type_name = "HerculesColor";
			break;
		default:
			hga_type = TYPE_HERC;
			hga_type_name = "Hercules";
			break;
	}
	return 1;
error:
	if (release_io_ports)
		release_region(0x3b0, 12);
	if (release_io_port)
		release_region(0x3bf, 1);
	return 0;
}


static int hgafb_open(struct fb_info *info, int init)
{
	hga_gfx_mode();
	hga_clear_screen();
	if (!nologo) hga_show_logo(info);
	return 0;
}


static int hgafb_release(struct fb_info *info, int init)
{
	hga_txt_mode();
	hga_clear_screen();
	return 0;
}


static int hgafb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			   u_int transp, struct fb_info *info)
{
	if (regno > 1)
		return 1;
	return 0;
}


static int hgafb_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	if (var->vmode & FB_VMODE_YWRAP) {
		if (var->yoffset < 0 || 
		    var->yoffset >= info->var.yres_virtual ||
		    var->xoffset)
			return -EINVAL;
	} else {
		if (var->xoffset + var->xres > info->var.xres_virtual
		 || var->yoffset + var->yres > info->var.yres_virtual
		 || var->yoffset % 8)
			return -EINVAL;
	}

	hga_pan(var->xoffset, var->yoffset);
	return 0;
}


static int hgafb_blank(int blank_mode, struct fb_info *info)
{
	hga_blank(blank_mode);
	return 0;
}

#ifdef CONFIG_FB_HGA_ACCEL
static void hgafb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	u_int rows, y;
	u8 __iomem *dest;

	y = rect->dy;

	for (rows = rect->height; rows--; y++) {
		dest = rowaddr(info, y) + (rect->dx >> 3);
		switch (rect->rop) {
		case ROP_COPY:
			//fb_memset(dest, rect->color, (rect->width >> 3));
			break;
		case ROP_XOR:
			fb_writeb(~(fb_readb(dest)), dest);
			break;
		}
	}
}

static void hgafb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	u_int rows, y1, y2;
	u8 __iomem *src;
	u8 __iomem *dest;

	if (area->dy <= area->sy) {
		y1 = area->sy;
		y2 = area->dy;

		for (rows = area->height; rows--; ) {
			src = rowaddr(info, y1) + (area->sx >> 3);
			dest = rowaddr(info, y2) + (area->dx >> 3);
			//fb_memmove(dest, src, (area->width >> 3));
			y1++;
			y2++;
		}
	} else {
		y1 = area->sy + area->height - 1;
		y2 = area->dy + area->height - 1;

		for (rows = area->height; rows--;) {
			src = rowaddr(info, y1) + (area->sx >> 3);
			dest = rowaddr(info, y2) + (area->dx >> 3);
			//fb_memmove(dest, src, (area->width >> 3));
			y1--;
			y2--;
		}
	}
}

static void hgafb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	u8 __iomem *dest;
	u8 *cdat = (u8 *) image->data;
	u_int rows, y = image->dy;
	u8 d;

	for (rows = image->height; rows--; y++) {
		d = *cdat++;
		dest = rowaddr(info, y) + (image->dx >> 3);
		fb_writeb(d, dest);
	}
}
#else /* !CONFIG_FB_HGA_ACCEL */
#define hgafb_fillrect cfb_fillrect
#define hgafb_copyarea cfb_copyarea
#define hgafb_imageblit cfb_imageblit
#endif /* CONFIG_FB_HGA_ACCEL */


static struct fb_ops hgafb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= hgafb_open,
	.fb_release	= hgafb_release,
	.fb_setcolreg	= hgafb_setcolreg,
	.fb_pan_display	= hgafb_pan_display,
	.fb_blank	= hgafb_blank,
	.fb_fillrect	= hgafb_fillrect,
	.fb_copyarea	= hgafb_copyarea,
	.fb_imageblit	= hgafb_imageblit,
};
		

/* ------------------------------------------------------------------------- */
    
	/*
	 *  Initialization
	 */

static int __devinit hgafb_probe(struct platform_device *pdev)
{
	struct fb_info *info;

	if (! hga_card_detect()) {
		printk(KERN_INFO "hgafb: HGA card not detected.\n");
		if (hga_vram)
			iounmap(hga_vram);
		return -EINVAL;
	}

	printk(KERN_INFO "hgafb: %s with %ldK of memory detected.\n",
		hga_type_name, hga_vram_len/1024);

	info = framebuffer_alloc(0, &pdev->dev);
	if (!info) {
		iounmap(hga_vram);
		return -ENOMEM;
	}

	hga_fix.smem_start = (unsigned long)hga_vram;
	hga_fix.smem_len = hga_vram_len;

	info->flags = FBINFO_DEFAULT | FBINFO_HWACCEL_YPAN;
	info->var = hga_default_var;
	info->fix = hga_fix;
	info->monspecs.hfmin = 0;
	info->monspecs.hfmax = 0;
	info->monspecs.vfmin = 10000;
	info->monspecs.vfmax = 10000;
	info->monspecs.dpms = 0;
	info->fbops = &hgafb_ops;
	info->screen_base = hga_vram;

        if (register_framebuffer(info) < 0) {
		framebuffer_release(info);
		iounmap(hga_vram);
		return -EINVAL;
	}

        printk(KERN_INFO "fb%d: %s frame buffer device\n",
               info->node, info->fix.id);
	platform_set_drvdata(pdev, info);
	return 0;
}

static int __devexit hgafb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	hga_txt_mode();
	hga_clear_screen();

	if (info) {
		unregister_framebuffer(info);
		framebuffer_release(info);
	}

	iounmap(hga_vram);

	if (release_io_ports)
		release_region(0x3b0, 12);

	if (release_io_port)
		release_region(0x3bf, 1);

	return 0;
}

static struct platform_driver hgafb_driver = {
	.probe = hgafb_probe,
	.remove = __devexit_p(hgafb_remove),
	.driver = {
		.name = "hgafb",
	},
};

static struct platform_device *hgafb_device;

static int __init hgafb_init(void)
{
	int ret;

	if (fb_get_options("hgafb", NULL))
		return -ENODEV;

	ret = platform_driver_register(&hgafb_driver);

	if (!ret) {
		hgafb_device = platform_device_register_simple("hgafb", 0, NULL, 0);

		if (IS_ERR(hgafb_device)) {
			platform_driver_unregister(&hgafb_driver);
			ret = PTR_ERR(hgafb_device);
		}
	}

	return ret;
}

static void __exit hgafb_exit(void)
{
	platform_device_unregister(hgafb_device);
	platform_driver_unregister(&hgafb_driver);
}


MODULE_AUTHOR("Ferenc Bakonyi (fero@drama.obuda.kando.hu)");
MODULE_DESCRIPTION("FBDev driver for Hercules Graphics Adaptor");
MODULE_LICENSE("GPL");

module_param(nologo, bool, 0);
MODULE_PARM_DESC(nologo, "Disables startup logo if != 0 (default=0)");
module_init(hgafb_init);
module_exit(hgafb_exit);
