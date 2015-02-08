


#ifndef IRDA_VLSI_FIR_H
#define IRDA_VLSI_FIR_H


/* definitions not present in pci_ids.h */

#ifndef PCI_CLASS_WIRELESS_IRDA
#define PCI_CLASS_WIRELESS_IRDA		0x0d00
#endif

#ifndef PCI_CLASS_SUBCLASS_MASK
#define PCI_CLASS_SUBCLASS_MASK		0xffff
#endif

/* ================================================================ */

/* non-standard PCI registers */

enum vlsi_pci_regs {
	VLSI_PCI_CLKCTL		= 0x40,		/* chip clock input control */
	VLSI_PCI_MSTRPAGE	= 0x41,		/* addr [31:24] for all busmaster cycles */
	VLSI_PCI_IRMISC		= 0x42		/* mainly legacy UART related */
};

/* ------------------------------------------ */

/* VLSI_PCI_CLKCTL: Clock Control Register (u8, rw) */


enum vlsi_pci_clkctl {

	/* PLL control */

	CLKCTL_PD_INV		= 0x04,		/* PD#: inverted power down signal,
						 * i.e. PLL is powered, if PD_INV set */
	CLKCTL_LOCK		= 0x40,		/* (ro) set, if PLL is locked */

	/* clock source selection */

	CLKCTL_EXTCLK		= 0x20,		/* set to select external clock input, not PLL */
	CLKCTL_XCKSEL		= 0x10,		/* set to indicate EXTCLK is 40MHz, not 48MHz */

	/* IrDA block control */

	CLKCTL_CLKSTP		= 0x80,		/* set to disconnect from selected clock source */
	CLKCTL_WAKE		= 0x08		/* set to enable wakeup feature: whenever IR activity
						 * is detected, PD_INV gets set(?) and CLKSTP cleared */
};

/* ------------------------------------------ */

/* VLSI_PCI_MSTRPAGE: Master Page Register (u8, rw) and busmastering stuff */

#define DMA_MASK_USED_BY_HW	0xffffffff
#define DMA_MASK_MSTRPAGE	0x00ffffff
#define MSTRPAGE_VALUE		(DMA_MASK_MSTRPAGE >> 24)

	/* PCI busmastering is somewhat special for this guy - in short:
	 *
	 * We select to operate using fixed MSTRPAGE=0, use ISA DMA
	 * address restrictions to make the PCI BM api aware of this,
	 * but ensure the hardware is dealing with real 32bit access.
	 *
	 * In detail:
	 * The chip executes normal 32bit busmaster cycles, i.e.
	 * drives all 32 address lines. These addresses however are
	 * composed of [0:23] taken from various busaddr-pointers
	 * and [24:31] taken from the MSTRPAGE register in the VLSI82C147
	 * config space. Therefore _all_ busmastering must be
	 * targeted to/from one single 16MB (busaddr-) superpage!
	 * The point is to make sure all the allocations for memory
	 * locations with busmaster access (ring descriptors, buffers)
	 * are indeed bus-mappable to the same 16MB range (for x86 this
	 * means they must reside in the same 16MB physical memory address
	 * range). The only constraint we have which supports "several objects
	 * mappable to common 16MB range" paradigma, is the old ISA DMA
	 * restriction to the first 16MB of physical address range.
	 * Hence the approach here is to enable PCI busmaster support using
	 * the correct 32bit dma-mask used by the chip. Afterwards the device's
	 * dma-mask gets restricted to 24bit, which must be honoured somehow by
	 * all allocations for memory areas to be exposed to the chip ...
	 *
	 * Note:
	 * Don't be surprised to get "Setting latency timer..." messages every
	 * time when PCI busmastering is enabled for the chip.
	 * The chip has its PCI latency timer RO fixed at 0 - which is not a
	 * problem here, because it is never requesting _burst_ transactions.
	 */

/* ------------------------------------------ */

/* VLSI_PCIIRMISC: IR Miscellaneous Register (u8, rw) */


enum vlsi_pci_irmisc {

	/* IR transceiver control */

	IRMISC_IRRAIL		= 0x40,		/* (ro?) IR rail power indication (and control?)
						 * 0=3.3V / 1=5V. Probably set during power-on?
						 * unclear - not touched by driver */
	IRMISC_IRPD		= 0x08,		/* transceiver power down, if set */

	/* legacy UART control */

	IRMISC_UARTTST		= 0x80,		/* UART test mode - "always write 0" */
	IRMISC_UARTEN		= 0x04,		/* enable UART address decoding */

	/* bits [1:0] IRMISC_UARTSEL to select legacy UART address */

	IRMISC_UARTSEL_3f8	= 0x00,
	IRMISC_UARTSEL_2f8	= 0x01,
	IRMISC_UARTSEL_3e8	= 0x02,
	IRMISC_UARTSEL_2e8	= 0x03
};

/* ================================================================ */

/* registers mapped to 32 byte PCI IO space */


enum vlsi_pio_regs {
	VLSI_PIO_IRINTR		= 0x00,		/* interrupt enable/request (u8, rw) */
	VLSI_PIO_RINGPTR	= 0x02,		/* rx/tx ring pointer (u16, ro) */
	VLSI_PIO_RINGBASE	= 0x04,		/* [23:10] of ring address (u16, rw) */
	VLSI_PIO_RINGSIZE	= 0x06,		/* rx/tx ring size (u16, rw) */
	VLSI_PIO_PROMPT		= 0x08, 	/* triggers ring processing (u16, wo) */
	/* 0x0a-0x0f: reserved / duplicated UART regs */
	VLSI_PIO_IRCFG		= 0x10,		/* configuration select (u16, rw) */
	VLSI_PIO_SIRFLAG	= 0x12,		/* BOF/EOF for filtered SIR (u16, ro) */
	VLSI_PIO_IRENABLE	= 0x14,		/* enable and status register (u16, rw/ro) */
	VLSI_PIO_PHYCTL		= 0x16,		/* physical layer current status (u16, ro) */
	VLSI_PIO_NPHYCTL	= 0x18,		/* next physical layer select (u16, rw) */
	VLSI_PIO_MAXPKT		= 0x1a,		/* [11:0] max len for packet receive (u16, rw) */
	VLSI_PIO_RCVBCNT	= 0x1c		/* current receive-FIFO byte count (u16, ro) */
	/* 0x1e-0x1f: reserved / duplicated UART regs */
};

/* ------------------------------------------ */

/* VLSI_PIO_IRINTR: Interrupt Register (u8, rw) */


enum vlsi_pio_irintr {
	IRINTR_ACTEN	= 0x80,	/* activity interrupt enable */
	IRINTR_ACTIVITY	= 0x40,	/* activity monitor (traffic detected) */
	IRINTR_RPKTEN	= 0x20,	/* receive packet interrupt enable*/
	IRINTR_RPKTINT	= 0x10,	/* rx-packet transfered from fifo to memory finished */
	IRINTR_TPKTEN	= 0x08,	/* transmit packet interrupt enable */
	IRINTR_TPKTINT	= 0x04,	/* last bit of tx-packet+crc shifted to ir-pulser */
	IRINTR_OE_EN	= 0x02,	/* UART rx fifo overrun error interrupt enable */
	IRINTR_OE_INT	= 0x01	/* UART rx fifo overrun error (read LSR to clear) */
};

/* we use this mask to check whether the (shared PCI) interrupt is ours */

#define IRINTR_INT_MASK		(IRINTR_ACTIVITY|IRINTR_RPKTINT|IRINTR_TPKTINT)

/* ------------------------------------------ */

/* VLSI_PIO_RINGPTR: Ring Pointer Read-Back Register (u16, ro) */


#define MAX_RING_DESCR		64	/* tx, rx rings may contain up to 64 descr each */

#define RINGPTR_RX_MASK		(MAX_RING_DESCR-1)
#define RINGPTR_TX_MASK		((MAX_RING_DESCR-1)<<8)

#define RINGPTR_GET_RX(p)	((p)&RINGPTR_RX_MASK)
#define RINGPTR_GET_TX(p)	(((p)&RINGPTR_TX_MASK)>>8)

/* ------------------------------------------ */

/* VLSI_PIO_RINGBASE: Ring Pointer Base Address Register (u16, ro) */


#define BUS_TO_RINGBASE(p)	(((p)>>10)&0x3fff)

/* ------------------------------------------ */

/* VLSI_PIO_RINGSIZE: Ring Size Register (u16, rw) */


#define SIZE_TO_BITS(num)		((((num)-1)>>2)&0x0f)
#define TX_RX_TO_RINGSIZE(tx,rx)	((SIZE_TO_BITS(tx)<<12)|(SIZE_TO_BITS(rx)<<8))
#define RINGSIZE_TO_RXSIZE(rs)		((((rs)&0x0f00)>>6)+4)
#define RINGSIZE_TO_TXSIZE(rs)		((((rs)&0xf000)>>10)+4)


/* ------------------------------------------ */

/* VLSI_PIO_PROMPT: Ring Prompting Register (u16, write-to-start) */


/* ------------------------------------------ */

/* VLSI_PIO_IRCFG: IR Config Register (u16, rw) */


enum vlsi_pio_ircfg {
	IRCFG_LOOP	= 0x4000,	/* enable loopback test mode */
	IRCFG_ENTX	= 0x1000,	/* transmit enable */
	IRCFG_ENRX	= 0x0800,	/* receive enable */
	IRCFG_MSTR	= 0x0400,	/* master enable */
	IRCFG_RXANY	= 0x0200,	/* receive any packet */
	IRCFG_CRC16	= 0x0080,	/* 16bit (not 32bit) CRC select for MIR/FIR */
	IRCFG_FIR	= 0x0040,	/* FIR 4PPM encoding mode enable */
	IRCFG_MIR	= 0x0020,	/* MIR HDLC encoding mode enable */
	IRCFG_SIR	= 0x0010,	/* SIR encoding mode enable */
	IRCFG_SIRFILT	= 0x0008,	/* enable SIR decode filter (receiver unwrapping) */
	IRCFG_SIRTEST	= 0x0004,	/* allow SIR decode filter when not in SIR mode */
	IRCFG_TXPOL	= 0x0002,	/* invert tx polarity when set */
	IRCFG_RXPOL	= 0x0001	/* invert rx polarity when set */
};

/* ------------------------------------------ */

/* VLSI_PIO_SIRFLAG: SIR Flag Register (u16, ro) */


/* ------------------------------------------ */

/* VLSI_PIO_IRENABLE: IR Enable Register (u16, rw/ro) */


enum vlsi_pio_irenable {
	IRENABLE_PHYANDCLOCK	= 0x8000,  /* enable IR phy and gate the mode config (rw) */
	IRENABLE_CFGER		= 0x4000,  /* mode configuration error (ro) */
	IRENABLE_FIR_ON		= 0x2000,  /* FIR on status (ro) */
	IRENABLE_MIR_ON		= 0x1000,  /* MIR on status (ro) */
	IRENABLE_SIR_ON		= 0x0800,  /* SIR on status (ro) */
	IRENABLE_ENTXST		= 0x0400,  /* transmit enable status (ro) */
	IRENABLE_ENRXST		= 0x0200,  /* Receive enable status (ro) */
	IRENABLE_CRC16_ON	= 0x0100   /* 16bit (not 32bit) CRC enabled status (ro) */
};

#define	  IRENABLE_MASK	    0xff00  /* Read mask */

/* ------------------------------------------ */

/* VLSI_PIO_PHYCTL: IR Physical Layer Current Control Register (u16, ro) */


/* ------------------------------------------ */

/* VLSI_PIO_NPHYCTL: IR Physical Layer Next Control Register (u16, rw) */


#define PHYCTL_BAUD_SHIFT	10
#define PHYCTL_BAUD_MASK	0xfc00
#define PHYCTL_PLSWID_SHIFT	5
#define PHYCTL_PLSWID_MASK	0x03e0
#define PHYCTL_PREAMB_SHIFT	0
#define PHYCTL_PREAMB_MASK	0x001f

#define PHYCTL_TO_BAUD(bwp)	(((bwp)&PHYCTL_BAUD_MASK)>>PHYCTL_BAUD_SHIFT)
#define PHYCTL_TO_PLSWID(bwp)	(((bwp)&PHYCTL_PLSWID_MASK)>>PHYCTL_PLSWID_SHIFT)
#define PHYCTL_TO_PREAMB(bwp)	(((bwp)&PHYCTL_PREAMB_MASK)>>PHYCTL_PREAMB_SHIFT)

#define BWP_TO_PHYCTL(b,w,p)	((((b)<<PHYCTL_BAUD_SHIFT)&PHYCTL_BAUD_MASK) \
				 | (((w)<<PHYCTL_PLSWID_SHIFT)&PHYCTL_PLSWID_MASK) \
				 | (((p)<<PHYCTL_PREAMB_SHIFT)&PHYCTL_PREAMB_MASK))

#define BAUD_BITS(br)		((115200/(br))-1)

static inline unsigned
calc_width_bits(unsigned baudrate, unsigned widthselect, unsigned clockselect)
{
	unsigned	tmp;

	if (widthselect)	/* nominal 3/16 puls width */
		return (clockselect) ? 12 : 24;

	tmp = ((clockselect) ? 12 : 24) / (BAUD_BITS(baudrate)+1);

	/* intermediate result of integer division needed here */

	return (tmp>0) ? (tmp-1) : 0;
}

#define PHYCTL_SIR(br,ws,cs)	BWP_TO_PHYCTL(BAUD_BITS(br),calc_width_bits((br),(ws),(cs)),0)
#define PHYCTL_MIR(cs)		BWP_TO_PHYCTL(0,((cs)?9:10),1)
#define PHYCTL_FIR		BWP_TO_PHYCTL(0,0,15)


/* ------------------------------------------ */


/* VLSI_PIO_MAXPKT: Maximum Packet Length register (u16, rw) */

/* maximum acceptable length for received packets */

/* hw imposed limitation - register uses only [11:0] */
#define MAX_PACKET_LENGTH	0x0fff

/* IrLAP I-field (apparently not defined elsewhere) */
#define IRDA_MTU		2048

/* complete packet consists of A(1)+C(1)+I(<=IRDA_MTU) */
#define IRLAP_SKB_ALLOCSIZE	(1+1+IRDA_MTU)


#define XFER_BUF_SIZE		MAX_PACKET_LENGTH

/* ------------------------------------------ */

/* VLSI_PIO_RCVBCNT: Receive Byte Count Register (u16, ro) */


#define RCVBCNT_MASK	0x0fff

/******************************************************************/


struct ring_descr_hw {
	volatile __le16	rd_count;	/* tx/rx count [11:0] */
	__le16		reserved;
	union {
		__le32	addr;		/* [23:0] of the buffer's busaddress */
		struct {
			u8		addr_res[3];
			volatile u8	status;		/* descriptor status */
		} __attribute__((packed)) rd_s;
	} __attribute((packed)) rd_u;
} __attribute__ ((packed));

#define rd_addr		rd_u.addr
#define rd_status	rd_u.rd_s.status

/* ring descriptor status bits */

#define RD_ACTIVE		0x80	/* descriptor owned by hw (both TX,RX) */

/* TX ring descriptor status */

#define	RD_TX_DISCRC		0x40	/* do not send CRC (for SIR) */
#define	RD_TX_BADCRC		0x20	/* force a bad CRC */
#define	RD_TX_PULSE		0x10	/* send indication pulse after this frame (MIR/FIR) */
#define	RD_TX_FRCEUND		0x08	/* force underrun */
#define	RD_TX_CLRENTX		0x04	/* clear ENTX after this frame */
#define	RD_TX_UNDRN		0x01	/* TX fifo underrun (probably PCI problem) */

/* RX ring descriptor status */

#define RD_RX_PHYERR		0x40	/* physical encoding error */
#define RD_RX_CRCERR		0x20	/* CRC error (MIR/FIR) */
#define RD_RX_LENGTH		0x10	/* frame exceeds buffer length */
#define RD_RX_OVER		0x08	/* RX fifo overrun (probably PCI problem) */
#define RD_RX_SIRBAD		0x04	/* EOF missing: BOF follows BOF (SIR, filtered) */

#define RD_RX_ERROR		0x7c	/* any error in received frame */

/* the memory required to hold the 2 descriptor rings */
#define HW_RING_AREA_SIZE	(2 * MAX_RING_DESCR * sizeof(struct ring_descr_hw))

/******************************************************************/


struct ring_descr {
	struct ring_descr_hw	*hw;
	struct sk_buff		*skb;
	void			*buf;
};


static inline int rd_is_active(struct ring_descr *rd)
{
	return ((rd->hw->rd_status & RD_ACTIVE) != 0);
}

static inline void rd_activate(struct ring_descr *rd)
{
	rd->hw->rd_status |= RD_ACTIVE;
}

static inline void rd_set_status(struct ring_descr *rd, u8 s)
{
	rd->hw->rd_status = s;	 /* may pass ownership to the hardware */
}

static inline void rd_set_addr_status(struct ring_descr *rd, dma_addr_t a, u8 s)
{
	/* order is important for two reasons:
	 *  - overlayed: writing addr overwrites status
	 *  - we want to write status last so we have valid address in
	 *    case status has RD_ACTIVE set
	 */

	if ((a & ~DMA_MASK_MSTRPAGE)>>24 != MSTRPAGE_VALUE) {
		IRDA_ERROR("%s: pci busaddr inconsistency!\n", __func__);
		dump_stack();
		return;
	}

	a &= DMA_MASK_MSTRPAGE;  /* clear highbyte to make sure we won't write
				  * to status - just in case MSTRPAGE_VALUE!=0
				  */
	rd->hw->rd_addr = cpu_to_le32(a);
	wmb();
	rd_set_status(rd, s);	 /* may pass ownership to the hardware */
}

static inline void rd_set_count(struct ring_descr *rd, u16 c)
{
	rd->hw->rd_count = cpu_to_le16(c);
}

static inline u8 rd_get_status(struct ring_descr *rd)
{
	return rd->hw->rd_status;
}

static inline dma_addr_t rd_get_addr(struct ring_descr *rd)
{
	dma_addr_t	a;

	a = le32_to_cpu(rd->hw->rd_addr);
	return (a & DMA_MASK_MSTRPAGE) | (MSTRPAGE_VALUE << 24);
}

static inline u16 rd_get_count(struct ring_descr *rd)
{
	return le16_to_cpu(rd->hw->rd_count);
}

/******************************************************************/


struct vlsi_ring {
	struct pci_dev		*pdev;
	int			dir;
	unsigned		len;
	unsigned		size;
	unsigned		mask;
	atomic_t		head, tail;
	struct ring_descr	*rd;
};

/* ring processing helpers */

static inline struct ring_descr *ring_last(struct vlsi_ring *r)
{
	int t;

	t = atomic_read(&r->tail) & r->mask;
	return (((t+1) & r->mask) == (atomic_read(&r->head) & r->mask)) ? NULL : &r->rd[t];
}

static inline struct ring_descr *ring_put(struct vlsi_ring *r)
{
	atomic_inc(&r->tail);
	return ring_last(r);
}

static inline struct ring_descr *ring_first(struct vlsi_ring *r)
{
	int h;

	h = atomic_read(&r->head) & r->mask;
	return (h == (atomic_read(&r->tail) & r->mask)) ? NULL : &r->rd[h];
}

static inline struct ring_descr *ring_get(struct vlsi_ring *r)
{
	atomic_inc(&r->head);
	return ring_first(r);
}

/******************************************************************/

/* our private compound VLSI-PCI-IRDA device information */

typedef struct vlsi_irda_dev {
	struct pci_dev		*pdev;

	struct irlap_cb		*irlap;

	struct qos_info		qos;

	unsigned		mode;
	int			baud, new_baud;

	dma_addr_t		busaddr;
	void			*virtaddr;
	struct vlsi_ring	*tx_ring, *rx_ring;

	struct timeval		last_rx;

	spinlock_t		lock;
	struct mutex		mtx;

	u8			resume_ok;	
	struct proc_dir_entry	*proc_entry;

} vlsi_irda_dev_t;

/********************************************************/


#define VLSI_TX_DROP		0x0001
#define VLSI_TX_FIFO		0x0002

#define VLSI_RX_DROP		0x0100
#define VLSI_RX_OVER		0x0200
#define VLSI_RX_LENGTH  	0x0400
#define VLSI_RX_FRAME		0x0800
#define VLSI_RX_CRC		0x1000

/********************************************************/

#endif /* IRDA_VLSI_FIR_H */

