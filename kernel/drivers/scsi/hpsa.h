
#ifndef HPSA_H
#define HPSA_H

#include <scsi/scsicam.h>

#define IO_OK		0
#define IO_ERROR	1

struct ctlr_info;

struct access_method {
	void (*submit_command)(struct ctlr_info *h,
		struct CommandList *c);
	void (*set_intr_mask)(struct ctlr_info *h, unsigned long val);
	unsigned long (*fifo_full)(struct ctlr_info *h);
	bool (*intr_pending)(struct ctlr_info *h);
	unsigned long (*command_completed)(struct ctlr_info *h);
};

struct hpsa_scsi_dev_t {
	int devtype;
	int bus, target, lun;		/* as presented to the OS */
	unsigned char scsi3addr[8];	/* as presented to the HW */
#define RAID_CTLR_LUNID "\0\0\0\0\0\0\0\0"
	unsigned char device_id[16];    /* from inquiry pg. 0x83 */
	unsigned char vendor[8];        /* bytes 8-15 of inquiry data */
	unsigned char model[16];        /* bytes 16-31 of inquiry data */
	unsigned char revision[4];      /* bytes 32-35 of inquiry data */
	unsigned char raid_level;	/* from inquiry page 0xC1 */
};

struct ctlr_info {
	int	ctlr;
	char	devname[8];
	char    *product_name;
	char	firm_ver[4]; /* Firmware version */
	struct pci_dev *pdev;
	u32	board_id;
	void __iomem *vaddr;
	unsigned long paddr;
	int 	nr_cmds; /* Number of commands allowed on this controller */
	struct CfgTable __iomem *cfgtable;
	int     max_sg_entries;
	int	interrupts_enabled;
	int	major;
	int 	max_commands;
	int	commands_outstanding;
	int 	max_outstanding; /* Debug */
	int	usage_count;  /* number of opens all all minor devices */
#	define PERF_MODE_INT	0
#	define DOORBELL_INT	1
#	define SIMPLE_MODE_INT	2
#	define MEMQ_MODE_INT	3
	unsigned int intr[4];
	unsigned int msix_vector;
	unsigned int msi_vector;
	struct access_method access;

	/* queue and queue Info */
	struct hlist_head reqQ;
	struct hlist_head cmpQ;
	unsigned int Qdepth;
	unsigned int maxQsinceinit;
	unsigned int maxSG;
	spinlock_t lock;
	int maxsgentries;
	u8 max_cmd_sg_entries;
	int chainsize;
	struct SGDescriptor **cmd_sg_list;

	/* pointers to command and error info pool */
	struct CommandList 	*cmd_pool;
	dma_addr_t		cmd_pool_dhandle;
	struct ErrorInfo 	*errinfo_pool;
	dma_addr_t		errinfo_pool_dhandle;
	unsigned long  		*cmd_pool_bits;
	int			nr_allocs;
	int			nr_frees;
	int			busy_initializing;
	int			busy_scanning;
	int			scan_finished;
	spinlock_t		scan_lock;
	wait_queue_head_t	scan_wait_queue;

	struct Scsi_Host *scsi_host;
	spinlock_t devlock; /* to protect hba[ctlr]->dev[];  */
	int ndevices; /* number of used elements in .dev[] array. */
#define HPSA_MAX_SCSI_DEVS_PER_HBA 256
	struct hpsa_scsi_dev_t *dev[HPSA_MAX_SCSI_DEVS_PER_HBA];
	/*
	 * Performant mode tables.
	 */
	u32 trans_support;
	u32 trans_offset;
	struct TransTable_struct *transtable;
	unsigned long transMethod;

	/*
	 * Performant mode completion buffer
	 */
	u64 *reply_pool;
	dma_addr_t reply_pool_dhandle;
	u64 *reply_pool_head;
	size_t reply_pool_size;
	unsigned char reply_pool_wraparound;
	u32 *blockFetchTable;
	unsigned char *hba_inquiry_data;
};
#define HPSA_ABORT_MSG 0
#define HPSA_DEVICE_RESET_MSG 1
#define HPSA_BUS_RESET_MSG 2
#define HPSA_HOST_RESET_MSG 3
#define HPSA_MSG_SEND_RETRY_LIMIT 10
#define HPSA_MSG_SEND_RETRY_INTERVAL_MSECS 1000

#define HPSA_MAX_POLL_TIME_SECS (20)

#define HPSA_TUR_RETRY_LIMIT (20)
#define HPSA_MAX_WAIT_INTERVAL_SECS (30)

#define HPSA_BOARD_READY_WAIT_SECS (120)
#define HPSA_BOARD_READY_POLL_INTERVAL_MSECS (100)
#define HPSA_BOARD_READY_POLL_INTERVAL \
	((HPSA_BOARD_READY_POLL_INTERVAL_MSECS * HZ) / 1000)
#define HPSA_BOARD_READY_ITERATIONS \
	((HPSA_BOARD_READY_WAIT_SECS * 1000) / \
		HPSA_BOARD_READY_POLL_INTERVAL_MSECS)
#define HPSA_POST_RESET_PAUSE_MSECS (3000)
#define HPSA_POST_RESET_NOOP_RETRIES (12)

/*  Defining the diffent access_menthods */
#define SA5_DOORBELL	0x20
#define SA5_REQUEST_PORT_OFFSET	0x40
#define SA5_REPLY_INTR_MASK_OFFSET	0x34
#define SA5_REPLY_PORT_OFFSET		0x44
#define SA5_INTR_STATUS		0x30
#define SA5_SCRATCHPAD_OFFSET	0xB0

#define SA5_CTCFG_OFFSET	0xB4
#define SA5_CTMEM_OFFSET	0xB8

#define SA5_INTR_OFF		0x08
#define SA5B_INTR_OFF		0x04
#define SA5_INTR_PENDING	0x08
#define SA5B_INTR_PENDING	0x04
#define FIFO_EMPTY		0xffffffff
#define HPSA_FIRMWARE_READY	0xffff0000 /* value in scratchpad register */

#define HPSA_ERROR_BIT		0x02

/* Performant mode flags */
#define SA5_PERF_INTR_PENDING   0x04
#define SA5_PERF_INTR_OFF       0x05
#define SA5_OUTDB_STATUS_PERF_BIT       0x01
#define SA5_OUTDB_CLEAR_PERF_BIT        0x01
#define SA5_OUTDB_CLEAR         0xA0
#define SA5_OUTDB_CLEAR_PERF_BIT        0x01
#define SA5_OUTDB_STATUS        0x9C


#define HPSA_INTR_ON 	1
#define HPSA_INTR_OFF	0
static void SA5_submit_command(struct ctlr_info *h,
	struct CommandList *c)
{
	dev_dbg(&h->pdev->dev, "Sending %x, tag = %x\n", c->busaddr,
		c->Header.Tag.lower);
	writel(c->busaddr, h->vaddr + SA5_REQUEST_PORT_OFFSET);
	h->commands_outstanding++;
	if (h->commands_outstanding > h->max_outstanding)
		h->max_outstanding = h->commands_outstanding;
}

static void SA5_intr_mask(struct ctlr_info *h, unsigned long val)
{
	if (val) { /* Turn interrupts on */
		h->interrupts_enabled = 1;
		writel(0, h->vaddr + SA5_REPLY_INTR_MASK_OFFSET);
	} else { /* Turn them off */
		h->interrupts_enabled = 0;
		writel(SA5_INTR_OFF,
			h->vaddr + SA5_REPLY_INTR_MASK_OFFSET);
	}
}

static void SA5_performant_intr_mask(struct ctlr_info *h, unsigned long val)
{
	if (val) { /* turn on interrupts */
		h->interrupts_enabled = 1;
		writel(0, h->vaddr + SA5_REPLY_INTR_MASK_OFFSET);
	} else {
		h->interrupts_enabled = 0;
		writel(SA5_PERF_INTR_OFF,
			h->vaddr + SA5_REPLY_INTR_MASK_OFFSET);
	}
}

static unsigned long SA5_performant_completed(struct ctlr_info *h)
{
	unsigned long register_value = FIFO_EMPTY;

	/* flush the controller write of the reply queue by reading
	 * outbound doorbell status register.
	 */
	register_value = readl(h->vaddr + SA5_OUTDB_STATUS);
	/* msi auto clears the interrupt pending bit. */
	if (!(h->msi_vector || h->msix_vector)) {
		writel(SA5_OUTDB_CLEAR_PERF_BIT, h->vaddr + SA5_OUTDB_CLEAR);
		/* Do a read in order to flush the write to the controller
		 * (as per spec.)
		 */
		register_value = readl(h->vaddr + SA5_OUTDB_STATUS);
	}

	if ((*(h->reply_pool_head) & 1) == (h->reply_pool_wraparound)) {
		register_value = *(h->reply_pool_head);
		(h->reply_pool_head)++;
		h->commands_outstanding--;
	} else {
		register_value = FIFO_EMPTY;
	}
	/* Check for wraparound */
	if (h->reply_pool_head == (h->reply_pool + h->max_commands)) {
		h->reply_pool_head = h->reply_pool;
		h->reply_pool_wraparound ^= 1;
	}

	return register_value;
}

static unsigned long SA5_fifo_full(struct ctlr_info *h)
{
	if (h->commands_outstanding >= h->max_commands)
		return 1;
	else
		return 0;

}
static unsigned long SA5_completed(struct ctlr_info *h)
{
	unsigned long register_value
		= readl(h->vaddr + SA5_REPLY_PORT_OFFSET);

	if (register_value != FIFO_EMPTY)
		h->commands_outstanding--;

#ifdef HPSA_DEBUG
	if (register_value != FIFO_EMPTY)
		dev_dbg(&h->pdev->dev, "Read %lx back from board\n",
			register_value);
	else
		dev_dbg(&h->pdev->dev, "hpsa: FIFO Empty read\n");
#endif

	return register_value;
}
static bool SA5_intr_pending(struct ctlr_info *h)
{
	unsigned long register_value  =
		readl(h->vaddr + SA5_INTR_STATUS);
	dev_dbg(&h->pdev->dev, "intr_pending %lx\n", register_value);
	return register_value & SA5_INTR_PENDING;
}

static bool SA5_performant_intr_pending(struct ctlr_info *h)
{
	unsigned long register_value = readl(h->vaddr + SA5_INTR_STATUS);

	if (!register_value)
		return false;

	if (h->msi_vector || h->msix_vector)
		return true;

	/* Read outbound doorbell to flush */
	register_value = readl(h->vaddr + SA5_OUTDB_STATUS);
	return register_value & SA5_OUTDB_STATUS_PERF_BIT;
}

static struct access_method SA5_access = {
	SA5_submit_command,
	SA5_intr_mask,
	SA5_fifo_full,
	SA5_intr_pending,
	SA5_completed,
};

static struct access_method SA5_performant_access = {
	SA5_submit_command,
	SA5_performant_intr_mask,
	SA5_fifo_full,
	SA5_performant_intr_pending,
	SA5_performant_completed,
};

struct board_type {
	u32	board_id;
	char	*product_name;
	struct access_method *access;
};

#endif /* HPSA_H */

