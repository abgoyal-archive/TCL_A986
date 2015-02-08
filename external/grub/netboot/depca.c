/* Etherboot: depca.h merged, comments from Linux driver retained */

#include "etherboot.h"
#include "nic.h"
#include "cards.h"

#define DEPCA_NICSR ioaddr+0x00   /* Network interface CSR */
#define DEPCA_RBI   ioaddr+0x02   /* RAM buffer index (2k buffer mode) */
#define DEPCA_DATA  ioaddr+0x04   /* LANCE registers' data port */
#define DEPCA_ADDR  ioaddr+0x06   /* LANCE registers' address port */
#define DEPCA_HBASE ioaddr+0x08   /* EISA high memory base address reg. */
#define DEPCA_PROM  ioaddr+0x0c   /* Ethernet address ROM data port */
#define DEPCA_CNFG  ioaddr+0x0c   /* EISA Configuration port */
#define DEPCA_RBSA  ioaddr+0x0e   /* RAM buffer starting address (2k buff.) */

#define CSR0       0
#define CSR1       1
#define CSR2       2
#define CSR3       3

 
#define TO       	0x0100	/* Time Out for remote boot */
#define SHE      	0x0080  /* SHadow memory Enable */
#define BS       	0x0040  /* Bank Select */
#define BUF      	0x0020	/* BUFfer size (1->32k, 0->64k) */
#define RBE      	0x0010	/* Remote Boot Enable (1->net boot) */
#define AAC      	0x0008  /* Address ROM Address Counter (1->enable) */
#define _128KB      	0x0008  /* 128kB Network RAM (1->enable) */
#define IM       	0x0004	/* Interrupt Mask (1->mask) */
#define IEN      	0x0002	/* Interrupt tristate ENable (1->enable) */
#define LED      	0x0001	/* LED control */


#define ERR     	0x8000 	/* Error summary */
#define BABL    	0x4000 	/* Babble transmitter timeout error  */
#define CERR    	0x2000 	/* Collision Error */
#define MISS    	0x1000 	/* Missed packet */
#define MERR    	0x0800 	/* Memory Error */
#define RINT    	0x0400 	/* Receiver Interrupt */
#define TINT    	0x0200 	/* Transmit Interrupt */
#define IDON    	0x0100 	/* Initialization Done */
#define INTR    	0x0080 	/* Interrupt Flag */
#define INEA    	0x0040 	/* Interrupt Enable */
#define RXON    	0x0020 	/* Receiver on */
#define TXON    	0x0010 	/* Transmitter on */
#define TDMD    	0x0008 	/* Transmit Demand */
#define STOP    	0x0004 	/* Stop */
#define STRT    	0x0002 	/* Start */
#define INIT    	0x0001 	/* Initialize */
#define INTM            0xff00  /* Interrupt Mask */
#define INTE            0xfff0  /* Interrupt Enable */


#define BSWP    	0x0004	/* Byte SWaP */
#define ACON    	0x0002	/* ALE control */
#define BCON    	0x0001	/* Byte CONtrol */


#define PROM       	0x8000 	/* Promiscuous Mode */
#define EMBA       	0x0080	/* Enable Modified Back-off Algorithm */
#define INTL       	0x0040 	/* Internal Loopback */
#define DRTY       	0x0020 	/* Disable Retry */
#define COLL       	0x0010 	/* Force Collision */
#define DTCR       	0x0008 	/* Disable Transmit CRC */
#define LOOP       	0x0004 	/* Loopback */
#define DTX        	0x0002 	/* Disable the Transmitter */
#define DRX        	0x0001 	/* Disable the Receiver */


#define R_OWN       0x80000000 	/* Owner bit 0 = host, 1 = lance */
#define R_ERR     	0x4000 	/* Error Summary */
#define R_FRAM    	0x2000 	/* Framing Error */
#define R_OFLO    	0x1000 	/* Overflow Error */
#define R_CRC     	0x0800 	/* CRC Error */
#define R_BUFF    	0x0400 	/* Buffer Error */
#define R_STP     	0x0200 	/* Start of Packet */
#define R_ENP     	0x0100 	/* End of Packet */


#define T_OWN       0x80000000 	/* Owner bit 0 = host, 1 = lance */
#define T_ERR     	0x4000 	/* Error Summary */
#define T_ADD_FCS 	0x2000 	/* More the 1 retry needed to Xmit */
#define T_MORE    	0x1000	/* >1 retry to transmit packet */
#define T_ONE     	0x0800 	/* 1 try needed to transmit the packet */
#define T_DEF     	0x0400 	/* Deferred */
#define T_STP       0x02000000 	/* Start of Packet */
#define T_ENP       0x01000000	/* End of Packet */
#define T_FLAGS     0xff000000  /* TX Flags Field */


#define TMD3_BUFF    0x8000	/* BUFFer error */
#define TMD3_UFLO    0x4000	/* UnderFLOw error */
#define TMD3_RES     0x2000	/* REServed */
#define TMD3_LCOL    0x1000	/* Late COLlision */
#define TMD3_LCAR    0x0800	/* Loss of CARrier */
#define TMD3_RTRY    0x0400	/* ReTRY error */

#define PROBE_LENGTH    32

#define NUM_RX_DESC     2               /* Number of RX descriptors */
#define NUM_TX_DESC     2               /* Number of TX descriptors */
#define RX_BUFF_SZ	1536            /* Buffer size for each Rx buffer */
#define TX_BUFF_SZ	1536            /* Buffer size for each Tx buffer */

#define DEPCA_IO_PORTS	{0x300, 0x200, 0}

#ifndef	DEPCA_MODEL
#define	DEPCA_MODEL	DEPCA
#endif

static enum {
	DEPCA, DE100, DE101, DE200, DE201, DE202, DE210, DE212, DE422, unknown
} adapter = DEPCA_MODEL;


static char *adapter_name[] = {
	"DEPCA",
	"DE100","DE101",
	"DE200","DE201","DE202",
	"DE210","DE212",
	"DE422",
	""
};

#ifndef	DEPCA_RAM_BASE
#define DEPCA_RAM_BASE	0xd0000
#endif

#define ALIGN4      ((u32)4 - 1)       /* 1 longword align */
#define ALIGN8      ((u32)8 - 1)       /* 2 longword (quadword) align */
#define ALIGN         ALIGN8              /* Keep the LANCE happy... */

typedef	long		s32;
typedef	unsigned long	u32;
typedef	short		s16;
typedef	unsigned short	u16;
typedef	char		s8;
typedef	unsigned char	u8;

struct depca_rx_desc {
    volatile s32 base;
    s16 buf_length;		/* This length is negative 2's complement! */
    s16 msg_length;		/* This length is "normal". */
};

struct depca_tx_desc {
    volatile s32 base;
    s16 length;		        /* This length is negative 2's complement! */
    s16 misc;                   /* Errors and TDR info */
};

#define LA_MASK 0x0000ffff      /* LANCE address mask for mapping network RAM
				   to LANCE memory address space */

struct depca_init {
    u16 mode;	                /* Mode register */
    u8  phys_addr[ETH_ALEN];	/* Physical ethernet address */
    u8  mcast_table[8];	        /* Multicast Hash Table. */
    u32 rx_ring;     	        /* Rx ring base pointer & ring length */
    u32 tx_ring;	        /* Tx ring base pointer & ring length */
};

struct depca_private {
	struct depca_rx_desc	*rx_ring;
	struct depca_tx_desc	*tx_ring;
	struct depca_init	init_block;	/* Shadow init block */
	char			*rx_memcpy[NUM_RX_DESC];
	char			*tx_memcpy[NUM_TX_DESC];
	u32			bus_offset;	/* ISA bus address offset */
	u32			sh_mem;		/* address of shared mem */
	u32			dma_buffs;	/* Rx & Tx buffer start */
	int			rx_cur, tx_cur;	/* Next free ring entry */
	int			txRingMask, rxRingMask;
	s32			rx_rlen, tx_rlen;
	/* log2([rt]xRingMask+1) for the descriptors */
};

static Address		mem_start = DEPCA_RAM_BASE;
static Address		mem_len, offset;
static unsigned short	ioaddr = 0;
static struct depca_private	lp;

#define STOP_DEPCA \
    outw(CSR0, DEPCA_ADDR);\
    outw(STOP, DEPCA_DATA)

/* Initialize the lance Rx and Tx descriptor rings. */
static void depca_init_ring(struct nic *nic)
{
	int	i;
	u32	p;

	lp.rx_cur = lp.tx_cur = 0;
	/* Initialize the base addresses and length of each buffer in the ring */
	for (i = 0; i <= lp.rxRingMask; i++) {
		writel((p = lp.dma_buffs + i * RX_BUFF_SZ) | R_OWN, &lp.rx_ring[i].base);
		writew(-RX_BUFF_SZ, &lp.rx_ring[i].buf_length);
		lp.rx_memcpy[i] = (char *) (p + lp.bus_offset);
	}
	for (i = 0; i <= lp.txRingMask; i++) {
		writel((p = lp.dma_buffs + (i + lp.txRingMask + 1) * TX_BUFF_SZ) & 0x00ffffff, &lp.tx_ring[i].base);
		lp.tx_memcpy[i] = (char *) (p + lp.bus_offset);
	}

	/* Set up the initialization block */
	lp.init_block.rx_ring = ((u32) ((u32) lp.rx_ring) & LA_MASK) | lp.rx_rlen;
	lp.init_block.tx_ring = ((u32) ((u32) lp.tx_ring) & LA_MASK) | lp.tx_rlen;
	for (i = 0; i < ETH_ALEN; i++)
		lp.init_block.phys_addr[i] = nic->node_addr[i];
	lp.init_block.mode = 0x0000;	/* Enable the Tx and Rx */
	memset(lp.init_block.mcast_table, 0, sizeof(lp.init_block.mcast_table));
}

static void LoadCSRs(void)
{
	outw(CSR1, DEPCA_ADDR);	/* initialisation block address LSW */
	outw((u16) (lp.sh_mem & LA_MASK), DEPCA_DATA);
	outw(CSR2, DEPCA_ADDR);	/* initialisation block address MSW */
	outw((u16) ((lp.sh_mem & LA_MASK) >> 16), DEPCA_DATA);
	outw(CSR3, DEPCA_ADDR);	/* ALE control */
	outw(ACON, DEPCA_DATA);
	outw(CSR0, DEPCA_ADDR);	/* Point back to CSR0 */
}

static int InitRestartDepca(void)
{
	int		i;

	/* Copy the shadow init_block to shared memory */
	memcpy_toio((char *)lp.sh_mem, &lp.init_block, sizeof(struct depca_init));
	outw(CSR0, DEPCA_ADDR);		/* point back to CSR0 */
	outw(INIT, DEPCA_DATA);		/* initialise DEPCA */

	for (i = 0; i < 100 && !(inw(DEPCA_DATA) & IDON); i++)
		;
	if (i < 100) {
		/* clear IDON by writing a 1, and start LANCE */
		outw(IDON | STRT, DEPCA_DATA);
	} else {
		printf("DEPCA not initialised\n");
		return (1);
	}
	return (0);
}

static void depca_reset(struct nic *nic)
{
	s16	nicsr;
	int	i, j;

	STOP_DEPCA;
	nicsr = inb(DEPCA_NICSR);
	nicsr = ((nicsr & ~SHE & ~RBE & ~IEN) | IM);
	outb(nicsr, DEPCA_NICSR);
	if (inw(DEPCA_DATA) != STOP)
	{
		printf("depca: Cannot stop NIC\n");
		return;
	}

	/* Initialisation block */
	lp.sh_mem = mem_start;
	mem_start += sizeof(struct depca_init);
	/* Tx & Rx descriptors (aligned to a quadword boundary) */
	mem_start = (mem_start + ALIGN) & ~ALIGN;
	lp.rx_ring = (struct depca_rx_desc *) mem_start;
	mem_start += (sizeof(struct depca_rx_desc) * NUM_RX_DESC);
	lp.tx_ring = (struct depca_tx_desc *) mem_start;
	mem_start += (sizeof(struct depca_tx_desc) * NUM_TX_DESC);

	lp.bus_offset = mem_start & 0x00ff0000;
	/* LANCE re-mapped start address */
	lp.dma_buffs = mem_start & LA_MASK;

	/* Finish initialising the ring information. */
	lp.rxRingMask = NUM_RX_DESC - 1;
	lp.txRingMask = NUM_TX_DESC - 1;

	/* Calculate Tx/Rx RLEN size for the descriptors. */
	for (i = 0, j = lp.rxRingMask; j > 0; i++) {
		j >>= 1;
	}
	lp.rx_rlen = (s32) (i << 29);
	for (i = 0, j = lp.txRingMask; j > 0; i++) {
		j >>= 1;
	}
	lp.tx_rlen = (s32) (i << 29);

	/* Load the initialisation block */
	depca_init_ring(nic);
	LoadCSRs();
	InitRestartDepca();
}

static int depca_poll(struct nic *nic)
{
	int		entry;
	u32		status;

	entry = lp.rx_cur;
	if ((status = readl(&lp.rx_ring[entry].base) & R_OWN))
		return (0);
	memcpy(nic->packet, lp.rx_memcpy[entry], nic->packetlen = lp.rx_ring[entry].msg_length);
	lp.rx_ring[entry].base |= R_OWN;
	lp.rx_cur = (++lp.rx_cur) & lp.rxRingMask;
	return (1);
}

static void depca_transmit(
	struct nic *nic,
	const char *d,			/* Destination */
	unsigned int t,			/* Type */
	unsigned int s,			/* size */
	const char *p)			/* Packet */
{
	int		entry, len;
	char		*mem;

	/* send the packet to destination */
	/*
	** Caution: the right order is important here... dont
	** setup the ownership rights until all the other
	** information is in place
	*/
	mem = lp.tx_memcpy[entry = lp.tx_cur];
	memcpy_toio(mem, d, ETH_ALEN);
	memcpy_toio(mem + ETH_ALEN, nic->node_addr, ETH_ALEN);
	mem[ETH_ALEN * 2] = t >> 8;
	mem[ETH_ALEN * 2 + 1] = t;
	memcpy_toio(mem + ETH_HLEN, p, s);
	s += ETH_HLEN;
	len = (s < ETH_ZLEN ? ETH_ZLEN : s);
	/* clean out flags */
	writel(readl(&lp.tx_ring[entry].base) & ~T_FLAGS, &lp.tx_ring[entry].base);
	/* clears other error flags */
	writew(0x0000, &lp.tx_ring[entry].misc);
	/* packet length in buffer */
	writew(-len, &lp.tx_ring[entry].length);
	/* start and end of packet, ownership */
	writel(readl(&lp.tx_ring[entry].base) | (T_STP|T_ENP|T_OWN), &lp.tx_ring[entry].base);
	/* update current pointers */
	lp.tx_cur = (++lp.tx_cur) & lp.txRingMask;
}

static void depca_disable(struct nic *nic)
{
	STOP_DEPCA;
}

static int depca_probe1(struct nic *nic)
{
	u8	data, nicsr;
	/* This is only correct for little endian machines, but then
	   Etherboot doesn't work on anything but a PC */
	u8	sig[] = { 0xFF, 0x00, 0x55, 0xAA, 0xFF, 0x00, 0x55, 0xAA };
	int	i, j;
	long	sum, chksum;

	data = inb(DEPCA_PROM);		/* clear counter on DEPCA */
	data = inb(DEPCA_PROM);		/* read data */
	if (data == 0x8) {
		nicsr = inb(DEPCA_NICSR);
		nicsr |= AAC;
		outb(nicsr, DEPCA_NICSR);
	}
	for (i = 0, j = 0; j < (int)sizeof(sig) && i < PROBE_LENGTH+((int)sizeof(sig))-1; ++i) {
		data = inb(DEPCA_PROM);
		if (data == sig[j])		/* track signature */
			++j;
		else
			j = (data == sig[0]) ? 1 : 0;
	}
	if (j != sizeof(sig))
		return (0);
	/* put the card in its initial state */
	STOP_DEPCA;
	nicsr = ((inb(DEPCA_NICSR) & ~SHE & ~RBE & ~IEN) | IM);
	outb(nicsr, DEPCA_NICSR);
	if (inw(DEPCA_DATA) != STOP)
		return (0);
	memcpy((char *)mem_start, sig, sizeof(sig));
	if (memcmp((char *)mem_start, sig, sizeof(sig)) != 0)
		return (0);
	for (i = 0, j = 0, sum = 0; j < 3; j++) {
		sum <<= 1;
		if (sum > 0xFFFF)
			sum -= 0xFFFF;
		sum += (u8)(nic->node_addr[i++] = inb(DEPCA_PROM));
		sum += (u16)((nic->node_addr[i++] = inb(DEPCA_PROM)) << 8);
		if (sum > 0xFFFF)
			sum -= 0xFFFF;
	}
	if (sum == 0xFFFF)
		sum = 0;
	chksum = (u8)inb(DEPCA_PROM);
	chksum |= (u16)(inb(DEPCA_PROM) << 8);
	mem_len = (adapter == DEPCA) ? (48 << 10) : (64 << 10);
	offset = 0;
	if (nicsr & BUF) {
		offset = 0x8000;
		nicsr &= ~BS;
		mem_len -= (32 << 10);
	}
	if (adapter != DEPCA)	/* enable shadow RAM */
		outb(nicsr |= SHE, DEPCA_NICSR);
	printf("%s base %#hX, memory [%#hX-%#hX], addr %!",
		adapter_name[adapter], ioaddr, mem_start, mem_start + mem_len,
		nic->node_addr);
	if (sum != chksum)
		printf(" (bad checksum)");
	putchar('\n');
	return (1);
}

struct nic *depca_probe(struct nic *nic, unsigned short *probe_addrs)
{
	static unsigned short	base[] = DEPCA_IO_PORTS;
	int			i;

	if (probe_addrs == 0 || probe_addrs[0] == 0)
		probe_addrs = base;	/* Use defaults */
	for (i = 0; (ioaddr = base[i]) != 0; ++i) {
		if (depca_probe1(nic))
			break;
	}
	if (ioaddr == 0)
		return (0);
	depca_reset(nic);
	/* point to NIC specific routines */
	nic->reset = depca_reset;
	nic->poll = depca_poll;
	nic->transmit = depca_transmit;
	nic->disable = depca_disable;
	return (nic);
}
