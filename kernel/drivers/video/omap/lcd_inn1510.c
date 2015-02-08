

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <plat/fpga.h>
#include "omapfb.h"

static int innovator1510_panel_init(struct lcd_panel *panel,
				    struct omapfb_device *fbdev)
{
	return 0;
}

static void innovator1510_panel_cleanup(struct lcd_panel *panel)
{
}

static int innovator1510_panel_enable(struct lcd_panel *panel)
{
	fpga_write(0x7, OMAP1510_FPGA_LCD_PANEL_CONTROL);
	return 0;
}

static void innovator1510_panel_disable(struct lcd_panel *panel)
{
	fpga_write(0x0, OMAP1510_FPGA_LCD_PANEL_CONTROL);
}

static unsigned long innovator1510_panel_get_caps(struct lcd_panel *panel)
{
	return 0;
}

struct lcd_panel innovator1510_panel = {
	.name		= "inn1510",
	.config		= OMAP_LCDC_PANEL_TFT,

	.bpp		= 16,
	.data_lines	= 16,
	.x_res		= 240,
	.y_res		= 320,
	.pixel_clock	= 12500,
	.hsw		= 40,
	.hfp		= 40,
	.hbp		= 72,
	.vsw		= 1,
	.vfp		= 1,
	.vbp		= 0,
	.pcd		= 12,

	.init		= innovator1510_panel_init,
	.cleanup	= innovator1510_panel_cleanup,
	.enable		= innovator1510_panel_enable,
	.disable	= innovator1510_panel_disable,
	.get_caps	= innovator1510_panel_get_caps,
};

static int innovator1510_panel_probe(struct platform_device *pdev)
{
	omapfb_register_panel(&innovator1510_panel);
	return 0;
}

static int innovator1510_panel_remove(struct platform_device *pdev)
{
	return 0;
}

static int innovator1510_panel_suspend(struct platform_device *pdev,
				       pm_message_t mesg)
{
	return 0;
}

static int innovator1510_panel_resume(struct platform_device *pdev)
{
	return 0;
}

struct platform_driver innovator1510_panel_driver = {
	.probe		= innovator1510_panel_probe,
	.remove		= innovator1510_panel_remove,
	.suspend	= innovator1510_panel_suspend,
	.resume		= innovator1510_panel_resume,
	.driver		= {
		.name	= "lcd_inn1510",
		.owner	= THIS_MODULE,
	},
};

static int __init innovator1510_panel_drv_init(void)
{
	return platform_driver_register(&innovator1510_panel_driver);
}

static void __exit innovator1510_panel_drv_cleanup(void)
{
	platform_driver_unregister(&innovator1510_panel_driver);
}

module_init(innovator1510_panel_drv_init);
module_exit(innovator1510_panel_drv_cleanup);

