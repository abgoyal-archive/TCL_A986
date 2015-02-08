

#ifndef _DT3155_INC
#define _DT3155_INC

#include <linux/types.h>
#include <linux/time.h>		/* struct timeval */


/* Uncomment this for 50Hz CCIR */
#define CCIR 1

/* Can be 1 or 2 */
#define MAXBOARDS 1

#define BOARD_MAX_BUFFS	3
#define MAXBUFFERS	(BOARD_MAX_BUFFS*MAXBOARDS)

#define PCI_PAGE_SIZE	(1 << 12)

#ifdef CCIR
#define DT3155_MAX_ROWS	576
#define DT3155_MAX_COLS	768
#define FORMAT50HZ	1
#else
#define DT3155_MAX_ROWS	480
#define DT3155_MAX_COLS	640
#define FORMAT50HZ	0
#endif

/* Configuration structure */
struct dt3155_config {
	u32 acq_mode;
	u32 cols, rows;
	u32 continuous;
};


/* hold data for each frame */
struct frame_info {
	u32 addr;		/* address of the buffer with the frame */
	u32 tag;		/* unique number for the frame */
	struct timeval time;	/* time that capture took place */
};

struct dt3155_fbuffer {
	int    nbuffers;

	struct frame_info frame_info[BOARD_MAX_BUFFS];

	int empty_buffers[BOARD_MAX_BUFFS];	/* indexes empty frames */
	int empty_len;				/* Number of empty buffers */
						/* Zero means empty */

	int active_buf;			/* Where data is currently dma'ing */
	int locked_buf;			/* Buffers used by user */

	int ready_que[BOARD_MAX_BUFFS];
	u32 ready_head;	/* The most recent buffer located here */
	u32 ready_len;	/* The number of ready buffers */

	int even_happened;
	int even_stopped;

	int stop_acquire;	/* Flag to stop interrupts */
	u32 frame_count;	/* Counter for frames acquired by this card */
};



#define DT3155_MODE_FRAME	1
#define DT3155_MODE_FIELD	2

#define DT3155_SNAP		1
#define DT3155_ACQ		2

/* There is one status structure for each card. */
struct dt3155_status {
	int fixed_mode;		/* if 1, we are in fixed frame mode */
	u32 reg_addr;	/* Register address for a single card */
	u32 mem_addr;	/* Buffer start addr for this card */
	u32 mem_size;	/* This is the amount of mem available  */
	u32 irq;		/* this card's irq */
	struct dt3155_config config;		/* configuration struct */
	struct dt3155_fbuffer fbuffer;	/* frame buffer state struct */
	u32 state;		/* this card's state */
	u32 device_installed;	/* Flag if installed. 1=installed */
};

/* Reference to global status structure */
extern struct dt3155_status dt3155_status[MAXBOARDS];

#define DT3155_STATE_IDLE	0x00
#define DT3155_STATE_FRAME	0x01
#define DT3155_STATE_FLD	0x02
#define DT3155_STATE_STOP	0x100
#define DT3155_STATE_ERROR	0x200
#define DT3155_STATE_MODE	0x0ff

#define DT3155_IOC_MAGIC	'!'

#define DT3155_SET_CONFIG	_IOW(DT3155_IOC_MAGIC, 1, struct dt3155_config)
#define DT3155_GET_CONFIG	_IOR(DT3155_IOC_MAGIC, 2, struct dt3155_status)
#define DT3155_STOP		_IO(DT3155_IOC_MAGIC, 3)
#define DT3155_START		_IO(DT3155_IOC_MAGIC, 4)
#define DT3155_FLUSH		_IO(DT3155_IOC_MAGIC, 5)
#define DT3155_IOC_MAXNR	5

/* Error codes */

#define DT_ERR_NO_BUFFERS	0x10000	/* not used but it might be one day */
#define DT_ERR_CORRUPT		0x20000
#define DT_ERR_OVERRUN		0x30000
#define DT_ERR_I2C_TIMEOUT	0x40000
#define DT_ERR_MASK		0xff0000/* not used but it might be one day */

/* User code will probably want to declare one of these for each card */
struct dt3155_read {
	u32 offset;
	u32 frame_seq;
	u32 state;

	struct frame_info frame_info;
};

#endif /* _DT3155_inc */
