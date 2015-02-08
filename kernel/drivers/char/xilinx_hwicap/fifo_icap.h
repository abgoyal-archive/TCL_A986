

#ifndef XILINX_FIFO_ICAP_H_	/* prevent circular inclusions */
#define XILINX_FIFO_ICAP_H_	/* by using protection macros */

#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include "xilinx_hwicap.h"

/* Reads integers from the device into the storage buffer. */
int fifo_icap_get_configuration(
		struct hwicap_drvdata *drvdata,
		u32 *FrameBuffer,
		u32 NumWords);

/* Writes integers to the device from the storage buffer. */
int fifo_icap_set_configuration(
		struct hwicap_drvdata *drvdata,
		u32 *FrameBuffer,
		u32 NumWords);

u32 fifo_icap_get_status(struct hwicap_drvdata *drvdata);
void fifo_icap_reset(struct hwicap_drvdata *drvdata);
void fifo_icap_flush_fifo(struct hwicap_drvdata *drvdata);

#endif
