

/* Sorry not us. It's inherted code from FreeBSD. [The authors] */

#include "etherboot.h"
#include "nic.h"
#include "pci.h"
#include "cards.h"
#include "timer.h"

#undef	virt_to_bus
#define	virt_to_bus(x)	((unsigned long)x)

static int ioaddr;

typedef unsigned char  u8;
typedef   signed char  s8;
typedef unsigned short u16;
typedef   signed short s16;
typedef unsigned int   u32;
typedef   signed int   s32;

enum speedo_offsets {
  SCBStatus = 0, SCBCmd = 2,      /* Rx/Command Unit command and status. */
  SCBPointer = 4,                 /* General purpose pointer. */
  SCBPort = 8,                    /* Misc. commands and operands.  */
  SCBflash = 12, SCBeeprom = 14,  /* EEPROM and flash memory control. */
  SCBCtrlMDI = 16,                /* MDI interface control. */
  SCBEarlyRx = 20,                /* Early receive byte count. */
};

static int do_eeprom_cmd(int cmd, int cmd_len);
void hd(void *where, int n);

/***********************************************************************/
/*                       I82557 related defines                        */
/***********************************************************************/

/*  EEPROM_Ctrl bits. */
#define EE_SHIFT_CLK    0x01    /* EEPROM shift clock. */
#define EE_CS           0x02    /* EEPROM chip select. */
#define EE_DATA_WRITE   0x04    /* EEPROM chip data in. */
#define EE_DATA_READ    0x08    /* EEPROM chip data out. */
#define EE_WRITE_0      0x4802
#define EE_WRITE_1      0x4806
#define EE_ENB          (0x4800 | EE_CS)

#define udelay(n)       waiton_timer2(((n)*TICKS_PER_MS)/1000)

/* The EEPROM commands include the alway-set leading bit. */
#define EE_READ_CMD     6

/* The SCB accepts the following controls for the Tx and Rx units: */
#define  CU_START       0x0010
#define  CU_RESUME      0x0020
#define  CU_STATSADDR   0x0040
#define  CU_SHOWSTATS   0x0050  /* Dump statistics counters. */
#define  CU_CMD_BASE    0x0060  /* Base address to add to add CU commands. */
#define  CU_DUMPSTATS   0x0070  /* Dump then reset stats counters. */

#define  RX_START       0x0001
#define  RX_RESUME      0x0002
#define  RX_ABORT       0x0004
#define  RX_ADDR_LOAD   0x0006
#define  RX_RESUMENR    0x0007
#define INT_MASK        0x0100
#define DRVR_INT        0x0200          /* Driver generated interrupt. */

enum phy_chips { NonSuchPhy=0, I82553AB, I82553C, I82503, DP83840, S80C240,
                                         S80C24, PhyUndefined, DP83840A=10, };

/* Commands that can be put in a command list entry. */
enum commands {
  CmdNOp = 0,
  CmdIASetup = 1,
  CmdConfigure = 2,
  CmdMulticastList = 3,
  CmdTx = 4,
  CmdTDR = 5,
  CmdDump = 6,
  CmdDiagnose = 7,

  /* And some extra flags: */
  CmdSuspend = 0x4000,      /* Suspend after completion. */
  CmdIntr = 0x2000,         /* Interrupt after completion. */
  CmdTxFlex = 0x0008,       /* Use "Flexible mode" for CmdTx command. */
};

static inline void wait_for_cmd_done(int cmd_ioaddr)
{
  short wait = 100;
  do   ;
  while(inb(cmd_ioaddr) && --wait >= 0);
}

/* Elements of the dump_statistics block. This block must be lword aligned. */
static struct speedo_stats {
        u32 tx_good_frames;
        u32 tx_coll16_errs;
        u32 tx_late_colls;
        u32 tx_underruns;
        u32 tx_lost_carrier;
        u32 tx_deferred;
        u32 tx_one_colls;
        u32 tx_multi_colls;
        u32 tx_total_colls;
        u32 rx_good_frames;
        u32 rx_crc_errs;
        u32 rx_align_errs;
        u32 rx_resource_errs;
        u32 rx_overrun_errs;
        u32 rx_colls_errs;
        u32 rx_runt_errs;
        u32 done_marker;
} lstats;

/* A speedo3 TX buffer descriptor with two buffers... */
static struct TxFD {
  volatile s16 status;
  s16 command;
  u32 link;          /* void * */
  u32 tx_desc_addr;  /* (almost) Always points to the tx_buf_addr element. */
  s32 count;         /* # of TBD (=2), Tx start thresh., etc. */
                     /* This constitutes two "TBD" entries: hdr and data */
  u32 tx_buf_addr0;  /* void *, header of frame to be transmitted.  */
  s32 tx_buf_size0;  /* Length of Tx hdr. */
  u32 tx_buf_addr1;  /* void *, data to be transmitted.  */
  s32 tx_buf_size1;  /* Length of Tx data. */
} txfd;

struct RxFD {               /* Receive frame descriptor. */
  volatile s16 status;
  s16 command;
  u32 link;                 /* struct RxFD * */
  u32 rx_buf_addr;          /* void * */
  u16 count;
  u16 size;
  char packet[1518];
};

#ifdef	USE_LOWMEM_BUFFER
#define rxfd ((struct RxFD *)(0x10000 - sizeof(struct RxFD)))
#define ACCESS(x) x->
#else
static struct RxFD rxfd;
#define ACCESS(x) x.
#endif

static int congenb = 0;         /* Enable congestion control in the DP83840. */
static int txfifo = 8;          /* Tx FIFO threshold in 4 byte units, 0-15 */
static int rxfifo = 8;          /* Rx FIFO threshold, default 32 bytes. */
static int txdmacount = 0;      /* Tx DMA burst length, 0-127, default 0. */
static int rxdmacount = 0;      /* Rx DMA length, 0 means no preemption. */

static struct ConfCmd {
  s16 status;
  s16 command;
  u32 link;
  unsigned char data[22];
} confcmd = {
  0, CmdConfigure,
  (u32) & txfd,
  {22, 0x08, 0, 0,  0, 0x80, 0x32, 0x03,  1, /* 1=Use MII  0=Use AUI */
   0, 0x2E, 0,  0x60, 0,
   0xf2, 0x48,   0, 0x40, 0xf2, 0x80,        /* 0x40=Force full-duplex */
   0x3f, 0x05, }
};

/***********************************************************************/
/*                       Locally used functions                        */
/***********************************************************************/


static int mdio_write(int phy_id, int location, int value)
{
  int val, boguscnt = 64*4;         /* <64 usec. to complete, typ 27 ticks */

  outl(0x04000000 | (location<<16) | (phy_id<<21) | value,
       ioaddr + SCBCtrlMDI);
  do {
    udelay(16);

    val = inl(ioaddr + SCBCtrlMDI);
    if (--boguscnt < 0) {
      printf(" mdio_write() timed out with val = %X.\n", val);
    }
  } while (! (val & 0x10000000));
  return val & 0xffff;
}

static int mdio_read(int phy_id, int location)
{
  int val, boguscnt = 64*4;               /* <64 usec. to complete, typ 27 ticks */
  outl(0x08000000 | (location<<16) | (phy_id<<21), ioaddr + SCBCtrlMDI);
  do {
    udelay(16);

    val = inl(ioaddr + SCBCtrlMDI);
    if (--boguscnt < 0) {
      printf( " mdio_read() timed out with val = %X.\n", val);
    }
  } while (! (val & 0x10000000));
  return val & 0xffff;
}

static int do_eeprom_cmd(int cmd, int cmd_len)
{
	unsigned retval = 0;
	long ee_addr = ioaddr + SCBeeprom;

	outw(EE_ENB, ee_addr); udelay(2);
	outw(EE_ENB | EE_SHIFT_CLK, ee_addr); udelay(2);

	/* Shift the command bits out. */
	do {
		short dataval = (cmd & (1 << cmd_len)) ? EE_WRITE_1 : EE_WRITE_0;
		outw(dataval, ee_addr); udelay(2);
		outw(dataval | EE_SHIFT_CLK, ee_addr); udelay(2);
		retval = (retval << 1) | ((inw(ee_addr) & EE_DATA_READ) ? 1 : 0);
	} while (--cmd_len >= 0);
	outw(EE_ENB, ee_addr); udelay(2);

	/* Terminate the EEPROM access. */
	outw(EE_ENB & ~EE_CS, ee_addr);
	return retval;
}

static inline void whereami (const char *str)
{
#if	0
  printf ("%s\n", str);
  sleep (2);
#endif
}


static void eepro100_reset(struct nic *nic)
{
  outl(0, ioaddr + SCBPort);
}


static void eepro100_transmit(struct nic *nic, const char *d, unsigned int t, unsigned int s, const char *p)
{
  struct eth_hdr {
    unsigned char dst_addr[ETH_ALEN];
    unsigned char src_addr[ETH_ALEN];
    unsigned short type;
  } hdr;
  unsigned short status;
  int to;
  int s1, s2;

  status = inw(ioaddr + SCBStatus);
  /* Acknowledge all of the current interrupt sources ASAP. */
  outw(status & 0xfc00, ioaddr + SCBStatus);

#ifdef	DEBUG
  printf ("transmitting type %hX packet (%d bytes). status = %hX, cmd=%hX\n",
	  t, s, status, inw (ioaddr + SCBCmd));
#endif

  memcpy (&hdr.dst_addr, d, ETH_ALEN);
  memcpy (&hdr.src_addr, nic->node_addr, ETH_ALEN);

  hdr.type = htons (t);

  txfd.status = 0;
  txfd.command = CmdSuspend | CmdTx | CmdTxFlex;
  txfd.link   = virt_to_bus (&txfd);
  txfd.count   = 0x02208000;
  txfd.tx_desc_addr = (u32)&txfd.tx_buf_addr0;

  txfd.tx_buf_addr0 = virt_to_bus (&hdr);
  txfd.tx_buf_size0 = sizeof (hdr);

  txfd.tx_buf_addr1 = virt_to_bus (p);
  txfd.tx_buf_size1 = s;

#ifdef	DEBUG
  printf ("txfd: \n");
  hd (&txfd, sizeof (txfd));
#endif

  outl(virt_to_bus(&txfd), ioaddr + SCBPointer);
  outw(INT_MASK | CU_START, ioaddr + SCBCmd);
  wait_for_cmd_done(ioaddr + SCBCmd);

  s1 = inw (ioaddr + SCBStatus);
  load_timer2(10*TICKS_PER_MS);		/* timeout 10 ms for transmit */
  while (!txfd.status && timer2_running())
    /* Wait */;
  s2 = inw (ioaddr + SCBStatus);

#ifdef	DEBUG
  printf ("s1 = %hX, s2 = %hX.\n", s1, s2);
#endif
}


static int eepro100_poll(struct nic *nic)
{
  if (!ACCESS(rxfd)status)
    return 0;

  /* Ok. We got a packet. Now restart the reciever.... */
  ACCESS(rxfd)status = 0;
  ACCESS(rxfd)command = 0xc000;
  outl(virt_to_bus(&(ACCESS(rxfd)status)), ioaddr + SCBPointer);
  outw(INT_MASK | RX_START, ioaddr + SCBCmd);
  wait_for_cmd_done(ioaddr + SCBCmd);

#ifdef	DEBUG
  printf ("Got a packet: Len = %d.\n", ACCESS(rxfd)count & 0x3fff);
#endif
  nic->packetlen =  ACCESS(rxfd)count & 0x3fff;
  memcpy (nic->packet, ACCESS(rxfd)packet, nic->packetlen);
#ifdef	DEBUG
  hd (nic->packet, 0x30);
#endif
  return 1;
}

static void eepro100_disable(struct nic *nic)
{
    /* See if this PartialReset solves the problem with interfering with
       kernel operation after Etherboot hands over. - Ken 20001102 */
    outl(2, ioaddr + SCBPort);
}


struct nic *eepro100_probe(struct nic *nic, unsigned short *probeaddrs, struct pci_device *p)
{
	unsigned short sum = 0;
	int i;
	int read_cmd, ee_size;
	unsigned short value;
	int options;
	int promisc;

	/* we cache only the first few words of the EEPROM data
	   be careful not to access beyond this array */
	unsigned short eeprom[16];

	if (probeaddrs == 0 || probeaddrs[0] == 0)
		return 0;
	ioaddr = probeaddrs[0] & ~3; /* Mask the bit that says "this is an io addr" */

	adjust_pci_device(p);

	if ((do_eeprom_cmd(EE_READ_CMD << 24, 27) & 0xffe0000)
		== 0xffe0000) {
		ee_size = 0x100;
		read_cmd = EE_READ_CMD << 24;
	} else {
		ee_size = 0x40;
		read_cmd = EE_READ_CMD << 22;
	}

	for (i = 0, sum = 0; i < ee_size; i++) {
		unsigned short value = do_eeprom_cmd(read_cmd | (i << 16), 27);
		if (i < (int)(sizeof(eeprom)/sizeof(eeprom[0])))
			eeprom[i] = value;
		sum += value;
	}

  for (i=0;i<ETH_ALEN;i++) {
	nic->node_addr[i] =  (eeprom[i/2] >> (8*(i&1))) & 0xff;
  }
  printf ("Ethernet addr: %!\n", nic->node_addr);

  if (sum != 0xBABA)
	printf("eepro100: Invalid EEPROM checksum %#hX, "
	       "check settings before activating this device!\n", sum);
  outl(0, ioaddr + SCBPort);
  udelay (10000);

  whereami ("Got eeprom.");

  outl(virt_to_bus(&lstats), ioaddr + SCBPointer);
  outw(INT_MASK | CU_STATSADDR, ioaddr + SCBCmd);
  wait_for_cmd_done(ioaddr + SCBCmd);

  whereami ("set stats addr.");
  /* INIT RX stuff. */

  /* Base = 0 */
  outl(0, ioaddr + SCBPointer);
  outw(INT_MASK | RX_ADDR_LOAD, ioaddr + SCBCmd);
  wait_for_cmd_done(ioaddr + SCBCmd);

  whereami ("set rx base addr.");

  ACCESS(rxfd)status  = 0x0001;
  ACCESS(rxfd)command = 0x0000;
  ACCESS(rxfd)link    = virt_to_bus(&(ACCESS(rxfd)status));
  ACCESS(rxfd)rx_buf_addr = (int) &nic->packet;
  ACCESS(rxfd)count   = 0;
  ACCESS(rxfd)size    = 1528;

  outl(virt_to_bus(&(ACCESS(rxfd)status)), ioaddr + SCBPointer);
  outw(INT_MASK | RX_START, ioaddr + SCBCmd);
  wait_for_cmd_done(ioaddr + SCBCmd);

  whereami ("started RX process.");

  /* Start the reciever.... */
  ACCESS(rxfd)status = 0;
  ACCESS(rxfd)command = 0xc000;
  outl(virt_to_bus(&(ACCESS(rxfd)status)), ioaddr + SCBPointer);
  outw(INT_MASK | RX_START, ioaddr + SCBCmd);

  /* INIT TX stuff. */

  /* Base = 0 */
  outl(0, ioaddr + SCBPointer);
  outw(INT_MASK | CU_CMD_BASE, ioaddr + SCBCmd);
  wait_for_cmd_done(ioaddr + SCBCmd);

  whereami ("set TX base addr.");

  txfd.command      = (CmdIASetup);
  txfd.status       = 0x0000;
  txfd.link         = virt_to_bus (&confcmd);

  {
	char *t = (char *)&txfd.tx_desc_addr;

	for (i=0;i<ETH_ALEN;i++)
		t[i] = nic->node_addr[i];
  }

#ifdef	DEBUG
  printf ("Setup_eaddr:\n");
  hd (&txfd, 0x20);
#endif
  /*      options = 0x40; */ /* 10mbps half duplex... */
  options = 0x00;            /* Autosense */

  promisc = 0;

  if (   ((eeprom[6]>>8) & 0x3f) == DP83840
	  || ((eeprom[6]>>8) & 0x3f) == DP83840A) {
	int mdi_reg23 = mdio_read(eeprom[6] & 0x1f, 23) | 0x0422;
	if (congenb)
	  mdi_reg23 |= 0x0100;
	printf("  DP83840 specific setup, setting register 23 to %hX.\n",
	       mdi_reg23);
	mdio_write(eeprom[6] & 0x1f, 23, mdi_reg23);
  }
  whereami ("Done DP8340 special setup.");
  if (options != 0) {
	mdio_write(eeprom[6] & 0x1f, 0,
		   ((options & 0x20) ? 0x2000 : 0) |    /* 100mbps? */
		   ((options & 0x10) ? 0x0100 : 0)); /* Full duplex? */
	whereami ("set mdio_register.");
  }

  confcmd.command  = CmdSuspend | CmdConfigure;
  confcmd.status   = 0x0000;
  confcmd.link     = virt_to_bus (&txfd);
  confcmd.data[1]  = (txfifo << 4) | rxfifo;
  confcmd.data[4]  = rxdmacount;
  confcmd.data[5]  = txdmacount + 0x80;
  confcmd.data[15] = promisc ? 0x49: 0x48;
  confcmd.data[19] = (options & 0x10) ? 0xC0 : 0x80;
  confcmd.data[21] = promisc ? 0x0D: 0x05;

  outl(virt_to_bus(&txfd), ioaddr + SCBPointer);
  outw(INT_MASK | CU_START, ioaddr + SCBCmd);
  wait_for_cmd_done(ioaddr + SCBCmd);

  whereami ("started TX thingy (config, iasetup).");

  load_timer2(10*TICKS_PER_MS);
  while (!txfd.status && timer2_running())
	/* Wait */;

  nic->reset = eepro100_reset;
  nic->poll = eepro100_poll;
  nic->transmit = eepro100_transmit;
  nic->disable = eepro100_disable;
  return nic;
}

/*********************************************************************/

#ifdef	DEBUG

/* Hexdump a number of bytes from memory... */
void hd (void *where, int n)
{
  int i;

  while (n > 0) {
    printf ("%X ", where);
    for (i=0;i < ( (n>16)?16:n);i++)
      printf (" %hhX", ((char *)where)[i]);
    printf ("\n");
    n -= 16;
    where += 16;
  }
}
#endif

