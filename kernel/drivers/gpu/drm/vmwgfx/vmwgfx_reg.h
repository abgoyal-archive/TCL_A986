


#ifndef _VMWGFX_REG_H_
#define _VMWGFX_REG_H_

#include <linux/types.h>

#define VMWGFX_INDEX_PORT     0x0
#define VMWGFX_VALUE_PORT     0x1
#define VMWGFX_IRQSTATUS_PORT 0x8

struct svga_guest_mem_descriptor {
	__le32 ppn;
	__le32 num_pages;
};

struct svga_fifo_cmd_fence {
	__le32 fence;
};

#define SVGA_SYNC_GENERIC         1
#define SVGA_SYNC_FIFOFULL        2

#include "svga_types.h"

#include "svga3d_reg.h"

#endif
