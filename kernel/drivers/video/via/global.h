

#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/console.h>
#include <linux/timer.h>

#include "debug.h"

#include "viafbdev.h"
#include "chip.h"
#include "accel.h"
#include "share.h"
#include "dvi.h"
#include "viamode.h"
#include "hw.h"

#include "lcd.h"
#include "ioctl.h"
#include "via_utility.h"
#include "vt1636.h"
#include "tblDPASetting.h"
#include "tbl1636.h"

/* External struct*/

extern int viafb_platform_epia_dvi;
extern int viafb_device_lcd_dualedge;
extern int viafb_bus_width;
extern int viafb_display_hardware_layout;
extern struct offset offset_reg;
extern struct viafb_par *viaparinfo;
extern struct viafb_par *viaparinfo1;
extern struct fb_info *viafbinfo;
extern struct fb_info *viafbinfo1;
extern int viafb_DeviceStatus;
extern int viafb_refresh;
extern int viafb_refresh1;
extern int viafb_lcd_dsp_method;
extern int viafb_lcd_mode;

extern int viafb_CRT_ON;
extern int viafb_hotplug_Xres;
extern int viafb_hotplug_Yres;
extern int viafb_hotplug_bpp;
extern int viafb_hotplug_refresh;
extern int viafb_primary_dev;

extern unsigned int viafb_second_xres;
extern unsigned int viafb_second_yres;
extern int viafb_lcd_panel_id;

#endif /* __GLOBAL_H__ */
