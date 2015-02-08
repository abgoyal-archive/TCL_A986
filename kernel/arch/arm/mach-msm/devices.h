

#ifndef __ARCH_ARM_MACH_MSM_DEVICES_H
#define __ARCH_ARM_MACH_MSM_DEVICES_H

#include "clock.h"

extern struct platform_device msm_device_uart1;
extern struct platform_device msm_device_uart2;
extern struct platform_device msm_device_uart3;

extern struct platform_device msm_device_sdc1;
extern struct platform_device msm_device_sdc2;
extern struct platform_device msm_device_sdc3;
extern struct platform_device msm_device_sdc4;

extern struct platform_device msm_device_hsusb;

extern struct platform_device msm_device_i2c;

extern struct platform_device msm_device_smd;

extern struct platform_device msm_device_nand;

extern struct clk msm_clocks_7x01a[];
extern unsigned msm_num_clocks_7x01a;

extern struct clk msm_clocks_7x30[];
extern unsigned msm_num_clocks_7x30;

extern struct clk msm_clocks_8x50[];
extern unsigned msm_num_clocks_8x50;

#endif
