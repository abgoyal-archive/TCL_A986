


#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>

#include <mach/gpio.h>
#include "omapfb.h"

static int palmtt_panel_init(struct lcd_panel *panel,
	struct omapfb_device *fbdev)
{
	return 0;
}

static void palmtt_panel_cleanup(struct lcd_panel *panel)
{
}

static int palmtt_panel_enable(struct lcd_panel *panel)
{
	return 0;
}

static void palmtt_panel_disable(struct lcd_panel *panel)
{
}

static unsigned long palmtt_panel_get_caps(struct lcd_panel *panel)
{
	return OMAPFB_CAPS_SET_BACKLIGHT;
}

struct lcd_panel palmtt_panel = {
	.name		= "palmtt",
	.config		= OMAP_LCDC_PANEL_TFT | OMAP_LCDC_INV_VSYNC |
			OMAP_LCDC_INV_HSYNC | OMAP_LCDC_HSVS_RISING_EDGE |
			OMAP_LCDC_HSVS_OPPOSITE,
	.bpp		= 16,
	.data_lines	= 16,
	.x_res		= 320,
	.y_res		= 320,
	.pixel_clock	= 10000,
	.hsw		= 4,
	.hfp		= 8,
	.hbp		= 28,
	.vsw		= 1,
	.vfp		= 8,
	.vbp		= 7,
	.pcd		= 0,

	.init		= palmtt_panel_init,
	.cleanup	= palmtt_panel_cleanup,
	.enable		= palmtt_panel_enable,
	.disable	= palmtt_panel_disable,
	.get_caps	= palmtt_panel_get_caps,
};

static int palmtt_panel_probe(struct platform_device *pdev)
{
	omapfb_register_panel(&palmtt_panel);
	return 0;
}

static int palmtt_panel_remove(struct platform_device *pdev)
{
	return 0;
}

static int palmtt_panel_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int palmtt_panel_resume(struct platform_device *pdev)
{
	return 0;
}

struct platform_driver palmtt_panel_driver = {
	.probe		= palmtt_panel_probe,
	.remove		= palmtt_panel_remove,
	.suspend	= palmtt_panel_suspend,
	.resume		= palmtt_panel_resume,
	.driver		= {
		.name	= "lcd_palmtt",
		.owner	= THIS_MODULE,
	},
};

static int __init palmtt_panel_drv_init(void)
{
	return platform_driver_register(&palmtt_panel_driver);
}

static void __exit palmtt_panel_drv_cleanup(void)
{
	platform_driver_unregister(&palmtt_panel_driver);
}

module_init(palmtt_panel_drv_init);
module_exit(palmtt_panel_drv_cleanup);
