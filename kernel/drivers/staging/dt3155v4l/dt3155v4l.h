

/*    DT3155 header file    */
#ifndef _DT3155_H_
#define _DT3155_H_

#ifdef __KERNEL__

#include <linux/pci.h>
#include <linux/interrupt.h>

#define DT3155_NAME "dt3155"
#define DT3155_VER_MAJ 1
#define DT3155_VER_MIN 1
#define DT3155_VER_EXT 0
#define DT3155_VERSION  __stringify(DT3155_VER_MAJ)	"."		\
			__stringify(DT3155_VER_MIN)	"."		\
			__stringify(DT3155_VER_EXT)

/* DT3155 Base Register offsets (memory mapped) */
#define EVEN_DMA_START	 0x00
#define ODD_DMA_START	 0x0C
#define EVEN_DMA_STRIDE  0x18
#define ODD_DMA_STRIDE	 0x24
#define EVEN_PIXEL_FMT	 0x30
#define ODD_PIXEL_FMT	 0x34
#define FIFO_TRIGER	 0x38
#define XFER_MODE	 0x3C
#define CSR1		 0x40
#define RETRY_WAIT_CNT	 0x44
#define INT_CSR		 0x48
#define EVEN_FLD_MASK	 0x4C
#define ODD_FLD_MASK	 0x50
#define MASK_LENGTH	 0x54
#define FIFO_FLAG_CNT	 0x58
#define IIC_CLK_DUR	 0x5C
#define IIC_CSR1	 0x60
#define IIC_CSR2	 0x64

/*  DT3155 Internal Registers indexes (i2c/IIC mapped) */
#define CSR2	     0x10
#define EVEN_CSR     0x11
#define ODD_CSR      0x12
#define CONFIG	     0x13
#define DT_ID	     0x1F
#define X_CLIP_START 0x20
#define Y_CLIP_START 0x22
#define X_CLIP_END   0x24
#define Y_CLIP_END   0x26
#define AD_ADDR      0x30
#define AD_LUT	     0x31
#define AD_CMD	     0x32
#define DIG_OUT      0x40
#define PM_LUT_ADDR  0x50
#define PM_LUT_DATA  0x51

/* AD command register values  */
#define AD_CMD_REG   0x00
#define AD_POS_REF   0x01
#define AD_NEG_REF   0x02

/* CSR1 bit masks */
#define CRPT_DIS       0x00004000
#define FLD_CRPT_ODD   0x00000200
#define FLD_CRPT_EVEN  0x00000100
#define FIFO_EN        0x00000080
#define SRST	       0x00000040
#define FLD_DN_ODD     0x00000020
#define FLD_DN_EVEN    0x00000010
#define CAP_CONT_ODD   0x00000002
#define CAP_CONT_EVEN  0x00000001

/*  INT_CSR bit masks */
#define FLD_START_EN	 0x00000400
#define FLD_END_ODD_EN	 0x00000200
#define FLD_END_EVEN_EN  0x00000100
#define FLD_START	 0x00000004
#define FLD_END_ODD	 0x00000002
#define FLD_END_EVEN	 0x00000001

/* IIC_CSR1 bit masks */
#define DIRECT_ABORT	 0x00000200

/* IIC_CSR2 bit masks */
#define NEW_CYCLE   0x01000000
#define DIR_RD	    0x00010000
#define IIC_READ    0x01010000
#define IIC_WRITE   0x01000000

/* CSR2 bit masks */
#define DISP_PASS     0x40
#define BUSY_ODD      0x20
#define BUSY_EVEN     0x10
#define SYNC_PRESENT  0x08
#define VT_50HZ       0x04
#define SYNC_SNTL     0x02
#define CHROM_FILT    0x01
#define VT_60HZ       0x00

/* CSR_EVEN/ODD bit masks */
#define CSR_ERROR	0x04
#define CSR_SNGL	0x02
#define CSR_DONE	0x01

/* CONFIG bit masks */
#define PM_LUT_PGM     0x80
#define PM_LUT_SEL     0x40
#define CLIP_EN        0x20
#define HSCALE_EN      0x10
#define EXT_TRIG_UP    0x0C
#define EXT_TRIG_DOWN  0x04
#define ACQ_MODE_NEXT  0x02
#define ACQ_MODE_ODD   0x01
#define ACQ_MODE_EVEN  0x00

/* AD_CMD bit masks */
#define VIDEO_CNL_1  0x00
#define VIDEO_CNL_2  0x40
#define VIDEO_CNL_3  0x80
#define VIDEO_CNL_4  0xC0
#define SYNC_CNL_1   0x00
#define SYNC_CNL_2   0x10
#define SYNC_CNL_3   0x20
#define SYNC_CNL_4   0x30
#define SYNC_LVL_1   0x00
#define SYNC_LVL_2   0x04
#define SYNC_LVL_3   0x08
#define SYNC_LVL_4   0x0C

/* DT3155 identificator */
#define DT3155_ID   0x20

#ifdef CONFIG_DT3155_CCIR
#define DMA_STRIDE 768
#else
#define DMA_STRIDE 640
#endif

struct dt3155_stats {
	int free_bufs_empty;
	int corrupted_fields;
	int dma_map_failed;
	int start_before_end;
};

/*    per board private data structure   */
struct dt3155_priv {
	struct video_device *vdev;
	struct file *acq_fp;
	int streaming;
	struct pci_dev *pdev;
	struct videobuf_queue *vidq;
	struct videobuf_buffer *curr_buf;
	struct task_struct *thread;
	irq_handler_t irq_handler;
	struct videobuf_qtype_ops qt_ops;
	struct list_head dmaq;
	wait_queue_head_t do_dma;
	struct mutex mux;
	spinlock_t lock;
	unsigned int field_count;
	struct dt3155_stats stats;
	void *regs;
	int users;
	u8 csr2, config;
};

#endif /*  __KERNEL__  */

#endif /*  _DT3155_H_  */
