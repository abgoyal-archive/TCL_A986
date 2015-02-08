

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/pci.h>

    /*
     *  This is just simple sample code.
     *
     *  No warranty that it actually compiles.
     *  Even less warranty that it actually works :-)
     */

static char *mode_option __devinitdata;


struct xxx_par;

static struct fb_fix_screeninfo xxxfb_fix __devinitdata = {
	.id =		"FB's name", 
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_PSEUDOCOLOR,
	.xpanstep =	1,
	.ypanstep =	1,
	.ywrapstep =	1, 
	.accel =	FB_ACCEL_NONE,
};

    /*
     * 	Modern graphical hardware not only supports pipelines but some 
     *  also support multiple monitors where each display can have its  
     *  its own unique data. In this case each display could be  
     *  represented by a separate framebuffer device thus a separate 
     *  struct fb_info. Now the struct xxx_par represents the graphics
     *  hardware state thus only one exist per card. In this case the 
     *  struct xxx_par for each graphics card would be shared between 
     *  every struct fb_info that represents a framebuffer on that card. 
     *  This allows when one display changes it video resolution (info->var) 
     *  the other displays know instantly. Each display can always be
     *  aware of the entire hardware state that affects it because they share
     *  the same xxx_par struct. The other side of the coin is multiple
     *  graphics cards that pass data around until it is finally displayed
     *  on one monitor. Such examples are the voodoo 1 cards and high end
     *  NUMA graphics servers. For this case we have a bunch of pars, each
     *  one that represents a graphics state, that belong to one struct 
     *  fb_info. Their you would want to have *par point to a array of device
     *  states and have each struct fb_ops function deal with all those 
     *  states. I hope this covers every possible hardware design. If not
     *  feel free to send your ideas at jsimmons@users.sf.net 
     */

    /*
     *  If your driver supports multiple boards or it supports multiple 
     *  framebuffers, you should make these arrays, or allocate them 
     *  dynamically using framebuffer_alloc() and free them with
     *  framebuffer_release().
     */ 
static struct fb_info info;

    /* 
     * Each one represents the state of the hardware. Most hardware have
     * just one hardware state. These here represent the default state(s). 
     */
static struct xxx_par __initdata current_par;

int xxxfb_init(void);

static int xxxfb_open(struct fb_info *info, int user)
{
    return 0;
}

static int xxxfb_release(struct fb_info *info, int user)
{
    return 0;
}

static int xxxfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
    /* ... */
    return 0;	   	
}

static int xxxfb_set_par(struct fb_info *info)
{
    struct xxx_par *par = info->par;
    /* ... */
    return 0;	
}

static int xxxfb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp,
			   struct fb_info *info)
{
    if (regno >= 256)  /* no. of hw registers */
       return -EINVAL;
    /*
     * Program hardware... do anything you want with transp
     */

    /* grayscale works only partially under directcolor */
    if (info->var.grayscale) {
       /* grayscale = 0.30*R + 0.59*G + 0.11*B */
       red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
    }

    /* Directcolor:
     *   var->{color}.offset contains start of bitfield
     *   var->{color}.length contains length of bitfield
     *   {hardwarespecific} contains width of DAC
     *   pseudo_palette[X] is programmed to (X << red.offset) |
     *                                      (X << green.offset) |
     *                                      (X << blue.offset)
     *   RAMDAC[X] is programmed to (red, green, blue)
     *   color depth = SUM(var->{color}.length)
     *
     * Pseudocolor:
     *    var->{color}.offset is 0 unless the palette index takes less than
     *                        bits_per_pixel bits and is stored in the upper
     *                        bits of the pixel value
     *    var->{color}.length is set so that 1 << length is the number of
     *                        available palette entries
     *    pseudo_palette is not used
     *    RAMDAC[X] is programmed to (red, green, blue)
     *    color depth = var->{color}.length
     *
     * Static pseudocolor:
     *    same as Pseudocolor, but the RAMDAC is not programmed (read-only)
     *
     * Mono01/Mono10:
     *    Has only 2 values, black on white or white on black (fg on bg),
     *    var->{color}.offset is 0
     *    white = (1 << var->{color}.length) - 1, black = 0
     *    pseudo_palette is not used
     *    RAMDAC does not exist
     *    color depth is always 2
     *
     * Truecolor:
     *    does not use RAMDAC (usually has 3 of them).
     *    var->{color}.offset contains start of bitfield
     *    var->{color}.length contains length of bitfield
     *    pseudo_palette is programmed to (red << red.offset) |
     *                                    (green << green.offset) |
     *                                    (blue << blue.offset) |
     *                                    (transp << transp.offset)
     *    RAMDAC does not exist
     *    color depth = SUM(var->{color}.length})
     *
     *  The color depth is used by fbcon for choosing the logo and also
     *  for color palette transformation if color depth < 4
     *
     *  As can be seen from the above, the field bits_per_pixel is _NOT_
     *  a criteria for describing the color visual.
     *
     *  A common mistake is assuming that bits_per_pixel <= 8 is pseudocolor,
     *  and higher than that, true/directcolor.  This is incorrect, one needs
     *  to look at the fix->visual.
     *
     *  Another common mistake is using bits_per_pixel to calculate the color
     *  depth.  The bits_per_pixel field does not directly translate to color
     *  depth. You have to compute for the color depth (using the color
     *  bitfields) and fix->visual as seen above.
     */

    /*
     * This is the point where the color is converted to something that
     * is acceptable by the hardware.
     */
#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
    red = CNVT_TOHW(red, info->var.red.length);
    green = CNVT_TOHW(green, info->var.green.length);
    blue = CNVT_TOHW(blue, info->var.blue.length);
    transp = CNVT_TOHW(transp, info->var.transp.length);
#undef CNVT_TOHW
    /*
     * This is the point where the function feeds the color to the hardware
     * palette after converting the colors to something acceptable by
     * the hardware. Note, only FB_VISUAL_DIRECTCOLOR and
     * FB_VISUAL_PSEUDOCOLOR visuals need to write to the hardware palette.
     * If you have code that writes to the hardware CLUT, and it's not
     * any of the above visuals, then you are doing something wrong.
     */
    if (info->fix.visual == FB_VISUAL_DIRECTCOLOR ||
	info->fix.visual == FB_VISUAL_TRUECOLOR)
	    write_{red|green|blue|transp}_to_clut();

    /* This is the point were you need to fill up the contents of
     * info->pseudo_palette. This structure is used _only_ by fbcon, thus
     * it only contains 16 entries to match the number of colors supported
     * by the console. The pseudo_palette is used only if the visual is
     * in directcolor or truecolor mode.  With other visuals, the
     * pseudo_palette is not used. (This might change in the future.)
     *
     * The contents of the pseudo_palette is in raw pixel format.  Ie, each
     * entry can be written directly to the framebuffer without any conversion.
     * The pseudo_palette is (void *).  However, if using the generic
     * drawing functions (cfb_imageblit, cfb_fillrect), the pseudo_palette
     * must be casted to (u32 *) _regardless_ of the bits per pixel. If the
     * driver is using its own drawing functions, then it can use whatever
     * size it wants.
     */
    if (info->fix.visual == FB_VISUAL_TRUECOLOR ||
	info->fix.visual == FB_VISUAL_DIRECTCOLOR) {
	    u32 v;

	    if (regno >= 16)
		    return -EINVAL;

	    v = (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset) |
		    (transp << info->var.transp.offset);

	    ((u32*)(info->pseudo_palette))[regno] = v;
    }

    /* ... */
    return 0;
}

static int xxxfb_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
    /*
     * If your hardware does not support panning, _do_ _not_ implement this
     * function. Creating a dummy function will just confuse user apps.
     */

    /*
     * Note that even if this function is fully functional, a setting of
     * 0 in both xpanstep and ypanstep means that this function will never
     * get called.
     */

    /* ... */
    return 0;
}

static int xxxfb_blank(int blank_mode, struct fb_info *info)
{
    /* ... */
    return 0;
}

/* ------------ Accelerated Functions --------------------- */


void xxxfb_fillrect(struct fb_info *p, const struct fb_fillrect *region)
{
}

void xxxfb_copyarea(struct fb_info *p, const struct fb_copyarea *area) 
{
}


void xxxfb_imageblit(struct fb_info *p, const struct fb_image *image) 
{

}

int xxxfb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
}

void xxxfb_rotate(struct fb_info *info, int angle)
{
/* Will be deprecated */
}

int xxxfb_sync(struct fb_info *info)
{
	return 0;
}

    /*
     *  Frame buffer operations
     */

static struct fb_ops xxxfb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= xxxfb_open,
	.fb_read	= xxxfb_read,
	.fb_write	= xxxfb_write,
	.fb_release	= xxxfb_release,
	.fb_check_var	= xxxfb_check_var,
	.fb_set_par	= xxxfb_set_par,
	.fb_setcolreg	= xxxfb_setcolreg,
	.fb_blank	= xxxfb_blank,
	.fb_pan_display	= xxxfb_pan_display,
	.fb_fillrect	= xxxfb_fillrect, 	/* Needed !!! */
	.fb_copyarea	= xxxfb_copyarea,	/* Needed !!! */
	.fb_imageblit	= xxxfb_imageblit,	/* Needed !!! */
	.fb_cursor	= xxxfb_cursor,		/* Optional !!! */
	.fb_rotate	= xxxfb_rotate,
	.fb_sync	= xxxfb_sync,
	.fb_ioctl	= xxxfb_ioctl,
	.fb_mmap	= xxxfb_mmap,
};

/* ------------------------------------------------------------------------- */

    /*
     *  Initialization
     */

/* static int __init xxfb_probe (struct platform_device *pdev) -- for platform devs */
static int __devinit xxxfb_probe(struct pci_dev *dev,
			      const struct pci_device_id *ent)
{
    struct fb_info *info;
    struct xxx_par *par;
    struct device *device = &dev->dev; /* or &pdev->dev */
    int cmap_len, retval;	
   
    /*
     * Dynamically allocate info and par
     */
    info = framebuffer_alloc(sizeof(struct xxx_par), device);

    if (!info) {
	    /* goto error path */
    }

    par = info->par;

    /* 
     * Here we set the screen_base to the virtual memory address
     * for the framebuffer. Usually we obtain the resource address
     * from the bus layer and then translate it to virtual memory
     * space via ioremap. Consult ioport.h. 
     */
    info->screen_base = framebuffer_virtual_memory;
    info->fbops = &xxxfb_ops;
    info->fix = xxxfb_fix; /* this will be the only time xxxfb_fix will be
			    * used, so mark it as __devinitdata
			    */
    info->pseudo_palette = pseudo_palette; /* The pseudopalette is an
					    * 16-member array
					    */
    /*
     * Set up flags to indicate what sort of acceleration your
     * driver can provide (pan/wrap/copyarea/etc.) and whether it
     * is a module -- see FBINFO_* in include/linux/fb.h
     *
     * If your hardware can support any of the hardware accelerated functions
     * fbcon performance will improve if info->flags is set properly.
     *
     * FBINFO_HWACCEL_COPYAREA - hardware moves
     * FBINFO_HWACCEL_FILLRECT - hardware fills
     * FBINFO_HWACCEL_IMAGEBLIT - hardware mono->color expansion
     * FBINFO_HWACCEL_YPAN - hardware can pan display in y-axis
     * FBINFO_HWACCEL_YWRAP - hardware can wrap display in y-axis
     * FBINFO_HWACCEL_DISABLED - supports hardware accels, but disabled
     * FBINFO_READS_FAST - if set, prefer moves over mono->color expansion
     * FBINFO_MISC_TILEBLITTING - hardware can do tile blits
     *
     * NOTE: These are for fbcon use only.
     */
    info->flags = FBINFO_DEFAULT;

/********************* This stage is optional ******************************/
     /*
     * The struct pixmap is a scratch pad for the drawing functions. This
     * is where the monochrome bitmap is constructed by the higher layers
     * and then passed to the accelerator.  For drivers that uses
     * cfb_imageblit, you can skip this part.  For those that have a more
     * rigorous requirement, this stage is needed
     */

    /* PIXMAP_SIZE should be small enough to optimize drawing, but not
     * large enough that memory is wasted.  A safe size is
     * (max_xres * max_font_height/8). max_xres is driver dependent,
     * max_font_height is 32.
     */
    info->pixmap.addr = kmalloc(PIXMAP_SIZE, GFP_KERNEL);
    if (!info->pixmap.addr) {
	    /* goto error */
    }

    info->pixmap.size = PIXMAP_SIZE;

    /*
     * FB_PIXMAP_SYSTEM - memory is in system ram
     * FB_PIXMAP_IO     - memory is iomapped
     * FB_PIXMAP_SYNC   - if set, will call fb_sync() per access to pixmap,
     *                    usually if FB_PIXMAP_IO is set.
     *
     * Currently, FB_PIXMAP_IO is unimplemented.
     */
    info->pixmap.flags = FB_PIXMAP_SYSTEM;

    /*
     * scan_align is the number of padding for each scanline.  It is in bytes.
     * Thus for accelerators that need padding to the next u32, put 4 here.
     */
    info->pixmap.scan_align = 4;

    /*
     * buf_align is the amount to be padded for the buffer. For example,
     * the i810fb needs a scan_align of 2 but expects it to be fed with
     * dwords, so a buf_align = 4 is required.
     */
    info->pixmap.buf_align = 4;

    /* access_align is how many bits can be accessed from the framebuffer
     * ie. some epson cards allow 16-bit access only.  Most drivers will
     * be safe with u32 here.
     *
     * NOTE: This field is currently unused.
     */
    info->pixmap.access_align = 32;
/***************************** End optional stage ***************************/

    /*
     * This should give a reasonable default video mode. The following is
     * done when we can set a video mode. 
     */
    if (!mode_option)
	mode_option = "640x480@60";	 	

    retval = fb_find_mode(&info->var, info, mode_option, NULL, 0, NULL, 8);
  
    if (!retval || retval == 4)
	return -EINVAL;			

    /* This has to be done! */
    if (fb_alloc_cmap(&info->cmap, cmap_len, 0))
	return -ENOMEM;
	
    /* 
     * The following is done in the case of having hardware with a static 
     * mode. If we are setting the mode ourselves we don't call this. 
     */	
    info->var = xxxfb_var;

    /*
     * For drivers that can...
     */
    xxxfb_check_var(&info->var, info);

    /*
     * Does a call to fb_set_par() before register_framebuffer needed?  This
     * will depend on you and the hardware.  If you are sure that your driver
     * is the only device in the system, a call to fb_set_par() is safe.
     *
     * Hardware in x86 systems has a VGA core.  Calling set_par() at this
     * point will corrupt the VGA console, so it might be safer to skip a
     * call to set_par here and just allow fbcon to do it for you.
     */
    /* xxxfb_set_par(info); */

    if (register_framebuffer(info) < 0) {
	fb_dealloc_cmap(&info->cmap);
	return -EINVAL;
    }
    printk(KERN_INFO "fb%d: %s frame buffer device\n", info->node,
	   info->fix.id);
    pci_set_drvdata(dev, info); /* or platform_set_drvdata(pdev, info) */
    return 0;
}

    /*
     *  Cleanup
     */
/* static void __devexit xxxfb_remove(struct platform_device *pdev) */
static void __devexit xxxfb_remove(struct pci_dev *dev)
{
	struct fb_info *info = pci_get_drvdata(dev);
	/* or platform_get_drvdata(pdev); */

	if (info) {
		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);
		/* ... */
		framebuffer_release(info);
	}
}

#ifdef CONFIG_PCI
#ifdef CONFIG_PM
static int xxxfb_suspend(struct pci_dev *dev, pm_message_t msg)
{
	struct fb_info *info = pci_get_drvdata(dev);
	struct xxxfb_par *par = info->par;

	/* suspend here */
	return 0;
}

static int xxxfb_resume(struct pci_dev *dev)
{
	struct fb_info *info = pci_get_drvdata(dev);
	struct xxxfb_par *par = info->par;

	/* resume here */
	return 0;
}
#else
#define xxxfb_suspend NULL
#define xxxfb_resume NULL
#endif /* CONFIG_PM */

static struct pci_device_id xxxfb_id_table[] = {
	{ PCI_VENDOR_ID_XXX, PCI_DEVICE_ID_XXX,
	  PCI_ANY_ID, PCI_ANY_ID, PCI_BASE_CLASS_DISPLAY << 16,
	  PCI_CLASS_MASK, 0 },
	{ 0, }
};

/* For PCI drivers */
static struct pci_driver xxxfb_driver = {
	.name =		"xxxfb",
	.id_table =	xxxfb_id_table,
	.probe =	xxxfb_probe,
	.remove =	__devexit_p(xxxfb_remove),
	.suspend =      xxxfb_suspend, /* optional but recommended */
	.resume =       xxxfb_resume,  /* optional but recommended */
};

MODULE_DEVICE_TABLE(pci, xxxfb_id_table);

int __init xxxfb_init(void)
{
	/*
	 *  For kernel boot options (in 'video=xxxfb:<options>' format)
	 */
#ifndef MODULE
	char *option = NULL;

	if (fb_get_options("xxxfb", &option))
		return -ENODEV;
	xxxfb_setup(option);
#endif

	return pci_register_driver(&xxxfb_driver);
}

static void __exit xxxfb_exit(void)
{
	pci_unregister_driver(&xxxfb_driver);
}
#else /* non PCI, platform drivers */
#include <linux/platform_device.h>
/* for platform devices */

#ifdef CONFIG_PM
static int xxxfb_suspend(struct platform_device *dev, pm_message_t msg)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct xxxfb_par *par = info->par;

	/* suspend here */
	return 0;
}

static int xxxfb_resume(struct platform_dev *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct xxxfb_par *par = info->par;

	/* resume here */
	return 0;
}
#else
#define xxxfb_suspend NULL
#define xxxfb_resume NULL
#endif /* CONFIG_PM */

static struct platform_device_driver xxxfb_driver = {
	.probe = xxxfb_probe,
	.remove = xxxfb_remove,
	.suspend = xxxfb_suspend, /* optional but recommended */
	.resume = xxxfb_resume,   /* optional but recommended */
	.driver = {
		.name = "xxxfb",
	},
};

static struct platform_device *xxxfb_device;

#ifndef MODULE
    /*
     *  Setup
     */

int __init xxxfb_setup(char *options)
{
    /* Parse user speficied options (`video=xxxfb:') */
}
#endif /* MODULE */

static int __init xxxfb_init(void)
{
	int ret;
	/*
	 *  For kernel boot options (in 'video=xxxfb:<options>' format)
	 */
#ifndef MODULE
	char *option = NULL;

	if (fb_get_options("xxxfb", &option))
		return -ENODEV;
	xxxfb_setup(option);
#endif
	ret = platform_driver_register(&xxxfb_driver);

	if (!ret) {
		xxxfb_device = platform_device_register_simple("xxxfb", 0,
								NULL, 0);

		if (IS_ERR(xxxfb_device)) {
			platform_driver_unregister(&xxxfb_driver);
			ret = PTR_ERR(xxxfb_device);
		}
	}

	return ret;
}

static void __exit xxxfb_exit(void)
{
	platform_device_unregister(xxxfb_device);
	platform_driver_unregister(&xxxfb_driver);
}
#endif /* CONFIG_PCI */

/* ------------------------------------------------------------------------- */


    /*
     *  Modularization
     */

module_init(xxxfb_init);
module_exit(xxxfb_remove);

MODULE_LICENSE("GPL");
