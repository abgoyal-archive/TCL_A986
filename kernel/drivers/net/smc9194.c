

static const char version[] =
	"smc9194.c:v0.14 12/15/00 by Erik Stahlman (erik@vt.edu)\n";

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/crc32.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/bitops.h>

#include <asm/io.h>

#include "smc9194.h"

#define DRV_NAME "smc9194"


#ifdef __sh__
#undef USE_32_BIT
#else
#define USE_32_BIT 1
#endif

#if defined(__H8300H__) || defined(__H8300S__)
#define NO_AUTOPROBE
#undef insl
#undef outsl
#define insl(a,b,l)  io_insl_noswap(a,b,l)
#define outsl(a,b,l) io_outsl_noswap(a,b,l)
#endif


struct devlist {
	unsigned int port;
	unsigned int irq;
};

#if defined(CONFIG_H8S_EDOSK2674)
static struct devlist smc_devlist[] __initdata = {
	{.port = 0xf80000, .irq = 16},
	{.port = 0,        .irq = 0 },
};
#else
static struct devlist smc_devlist[] __initdata = {
	{.port = 0x200, .irq = 0},
	{.port = 0x220, .irq = 0},
	{.port = 0x240, .irq = 0},
	{.port = 0x260, .irq = 0},
	{.port = 0x280, .irq = 0},
	{.port = 0x2A0, .irq = 0},
	{.port = 0x2C0, .irq = 0},
	{.port = 0x2E0, .irq = 0},
	{.port = 0x300, .irq = 0},
	{.port = 0x320, .irq = 0},
	{.port = 0x340, .irq = 0},
	{.port = 0x360, .irq = 0},
	{.port = 0x380, .irq = 0},
	{.port = 0x3A0, .irq = 0},
	{.port = 0x3C0, .irq = 0},
	{.port = 0x3E0, .irq = 0},
	{.port = 0,     .irq = 0},
};
#endif
#define MEMORY_WAIT_TIME 16

#define SMC_DEBUG 0

#if (SMC_DEBUG > 2 )
#define PRINTK3(x) printk x
#else
#define PRINTK3(x)
#endif

#if SMC_DEBUG > 1
#define PRINTK2(x) printk x
#else
#define PRINTK2(x)
#endif

#ifdef SMC_DEBUG
#define PRINTK(x) printk x
#else
#define PRINTK(x)
#endif


#define CARDNAME "SMC9194"


/* store this information for the driver.. */
struct smc_local {
	/*
	   If I have to wait until memory is available to send
	   a packet, I will store the skbuff here, until I get the
	   desired memory.  Then, I'll send it out and free it.
	*/
	struct sk_buff * saved_skb;

	/*
 	 . This keeps track of how many packets that I have
 	 . sent out.  When an TX_EMPTY interrupt comes, I know
	 . that all of these have been sent.
	*/
	int	packets_waiting;
};



struct net_device *smc_init(int unit);

static int smc_open(struct net_device *dev);

static void smc_timeout(struct net_device *dev);

static int smc_close(struct net_device *dev);

static void smc_set_multicast_list(struct net_device *dev);



static irqreturn_t smc_interrupt(int irq, void *);
static inline void smc_rcv( struct net_device *dev );
static inline void smc_tx( struct net_device * dev );


static int smc_probe(struct net_device *dev, int ioaddr);

#if SMC_DEBUG > 2
static void print_packet( byte *, int );
#endif

#define tx_done(dev) 1

/* this is called to actually send the packet to the chip */
static void smc_hardware_send_packet( struct net_device * dev );

static netdev_tx_t  smc_wait_to_send_packet( struct sk_buff * skb,
					     struct net_device *dev );

/* this does a soft reset on the device */
static void smc_reset( int ioaddr );

/* Enable Interrupts, Receive, and Transmit */
static void smc_enable( int ioaddr );

/* this puts the device in an inactive state */
static void smc_shutdown( int ioaddr );

static int smc_findirq( int ioaddr );

static void smc_reset( int ioaddr )
{
	/* This resets the registers mostly to defaults, but doesn't
	   affect EEPROM.  That seems unnecessary */
	SMC_SELECT_BANK( 0 );
	outw( RCR_SOFTRESET, ioaddr + RCR );

	/* this should pause enough for the chip to be happy */
	SMC_DELAY( );

	/* Set the transmit and receive configuration registers to
	   default values */
	outw( RCR_CLEAR, ioaddr + RCR );
	outw( TCR_CLEAR, ioaddr + TCR );

	/* set the control register to automatically
	   release successfully transmitted packets, to make the best
	   use out of our limited memory */
	SMC_SELECT_BANK( 1 );
	outw( inw( ioaddr + CONTROL ) | CTL_AUTO_RELEASE , ioaddr + CONTROL );

	/* Reset the MMU */
	SMC_SELECT_BANK( 2 );
	outw( MC_RESET, ioaddr + MMU_CMD );

	/* Note:  It doesn't seem that waiting for the MMU busy is needed here,
	   but this is a place where future chipsets _COULD_ break.  Be wary
 	   of issuing another MMU command right after this */

	outb( 0, ioaddr + INT_MASK );
}

static void smc_enable( int ioaddr )
{
	SMC_SELECT_BANK( 0 );
	/* see the header file for options in TCR/RCR NORMAL*/
	outw( TCR_NORMAL, ioaddr + TCR );
	outw( RCR_NORMAL, ioaddr + RCR );

	/* now, enable interrupts */
	SMC_SELECT_BANK( 2 );
	outb( SMC_INTERRUPT_MASK, ioaddr + INT_MASK );
}

static void smc_shutdown( int ioaddr )
{
	/* no more interrupts for me */
	SMC_SELECT_BANK( 2 );
	outb( 0, ioaddr + INT_MASK );

	/* and tell the card to stay away from that nasty outside world */
	SMC_SELECT_BANK( 0 );
	outb( RCR_CLEAR, ioaddr + RCR );
	outb( TCR_CLEAR, ioaddr + TCR );
#if 0
	/* finally, shut the chip down */
	SMC_SELECT_BANK( 1 );
	outw( inw( ioaddr + CONTROL ), CTL_POWERDOWN, ioaddr + CONTROL  );
#endif
}




static void smc_setmulticast(int ioaddr, struct net_device *dev)
{
	int			i;
	unsigned char		multicast_table[ 8 ];
	struct netdev_hw_addr *ha;
	/* table for flipping the order of 3 bits */
	unsigned char invert3[] = { 0, 4, 2, 6, 1, 5, 3, 7 };

	/* start with a table of all zeros: reject all */
	memset( multicast_table, 0, sizeof( multicast_table ) );

	netdev_for_each_mc_addr(ha, dev) {
		int position;

		/* make sure this is a multicast address - shouldn't this
		   be a given if we have it here ? */
		if (!(*ha->addr & 1))
			continue;

		/* only use the low order bits */
		position = ether_crc_le(6, ha->addr) & 0x3f;

		/* do some messy swapping to put the bit in the right spot */
		multicast_table[invert3[position&7]] |=
					(1<<invert3[(position>>3)&7]);

	}
	/* now, the table can be loaded into the chipset */
	SMC_SELECT_BANK( 3 );

	for ( i = 0; i < 8 ; i++ ) {
		outb( multicast_table[i], ioaddr + MULTICAST1 + i );
	}
}

static netdev_tx_t smc_wait_to_send_packet(struct sk_buff *skb,
					   struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr 	= dev->base_addr;
	word 			length;
	unsigned short 		numPages;
	word			time_out;

	netif_stop_queue(dev);
	/* Well, I want to send the packet.. but I don't know
	   if I can send it right now...  */

	if ( lp->saved_skb) {
		/* THIS SHOULD NEVER HAPPEN. */
		dev->stats.tx_aborted_errors++;
		printk(CARDNAME": Bad Craziness - sent packet while busy.\n" );
		return NETDEV_TX_BUSY;
	}
	lp->saved_skb = skb;

	length = skb->len;

	if (length < ETH_ZLEN) {
		if (skb_padto(skb, ETH_ZLEN)) {
			netif_wake_queue(dev);
			return NETDEV_TX_OK;
		}
		length = ETH_ZLEN;
	}

	/*
	** The MMU wants the number of pages to be the number of 256 bytes
	** 'pages', minus 1 ( since a packet can't ever have 0 pages :) )
	**
	** Pkt size for allocating is data length +6 (for additional status words,
	** length and ctl!) If odd size last byte is included in this header.
	*/
	numPages =  ((length & 0xfffe) + 6) / 256;

	if (numPages > 7 ) {
		printk(CARDNAME": Far too big packet error.\n");
		/* freeing the packet is a good thing here... but should
		 . any packets of this size get down here?   */
		dev_kfree_skb (skb);
		lp->saved_skb = NULL;
		/* this IS an error, but, i don't want the skb saved */
		netif_wake_queue(dev);
		return NETDEV_TX_OK;
	}
	/* either way, a packet is waiting now */
	lp->packets_waiting++;

	/* now, try to allocate the memory */
	SMC_SELECT_BANK( 2 );
	outw( MC_ALLOC | numPages, ioaddr + MMU_CMD );
	/*
 	. Performance Hack
	.
 	. wait a short amount of time.. if I can send a packet now, I send
	. it now.  Otherwise, I enable an interrupt and wait for one to be
	. available.
	.
	. I could have handled this a slightly different way, by checking to
	. see if any memory was available in the FREE MEMORY register.  However,
	. either way, I need to generate an allocation, and the allocation works
	. no matter what, so I saw no point in checking free memory.
	*/
	time_out = MEMORY_WAIT_TIME;
	do {
		word	status;

		status = inb( ioaddr + INTERRUPT );
		if ( status & IM_ALLOC_INT ) {
			/* acknowledge the interrupt */
			outb( IM_ALLOC_INT, ioaddr + INTERRUPT );
  			break;
		}
   	} while ( -- time_out );

   	if ( !time_out ) {
		/* oh well, wait until the chip finds memory later */
		SMC_ENABLE_INT( IM_ALLOC_INT );
		PRINTK2((CARDNAME": memory allocation deferred.\n"));
		/* it's deferred, but I'll handle it later */
		return NETDEV_TX_OK;
   	}
	/* or YES! I can send the packet now.. */
	smc_hardware_send_packet(dev);
	netif_wake_queue(dev);
	return NETDEV_TX_OK;
}

static void smc_hardware_send_packet( struct net_device * dev )
{
	struct smc_local *lp = netdev_priv(dev);
	byte	 		packet_no;
	struct sk_buff * 	skb = lp->saved_skb;
	word			length;
	unsigned int		ioaddr;
	byte			* buf;

	ioaddr = dev->base_addr;

	if ( !skb ) {
		PRINTK((CARDNAME": In XMIT with no packet to send\n"));
		return;
	}
	length = ETH_ZLEN < skb->len ? skb->len : ETH_ZLEN;
	buf = skb->data;

	/* If I get here, I _know_ there is a packet slot waiting for me */
	packet_no = inb( ioaddr + PNR_ARR + 1 );
	if ( packet_no & 0x80 ) {
		/* or isn't there?  BAD CHIP! */
		printk(KERN_DEBUG CARDNAME": Memory allocation failed.\n");
		dev_kfree_skb_any(skb);
		lp->saved_skb = NULL;
		netif_wake_queue(dev);
		return;
	}

	/* we have a packet address, so tell the card to use it */
	outb( packet_no, ioaddr + PNR_ARR );

	/* point to the beginning of the packet */
	outw( PTR_AUTOINC , ioaddr + POINTER );

   	PRINTK3((CARDNAME": Trying to xmit packet of length %x\n", length ));
#if SMC_DEBUG > 2
	print_packet( buf, length );
#endif

	/* send the packet length ( +6 for status, length and ctl byte )
 	   and the status word ( set to zeros ) */
#ifdef USE_32_BIT
	outl(  (length +6 ) << 16 , ioaddr + DATA_1 );
#else
	outw( 0, ioaddr + DATA_1 );
	/* send the packet length ( +6 for status words, length, and ctl*/
	outb( (length+6) & 0xFF,ioaddr + DATA_1 );
	outb( (length+6) >> 8 , ioaddr + DATA_1 );
#endif

	/* send the actual data
	 . I _think_ it's faster to send the longs first, and then
	 . mop up by sending the last word.  It depends heavily
 	 . on alignment, at least on the 486.  Maybe it would be
 	 . a good idea to check which is optimal?  But that could take
	 . almost as much time as is saved?
	*/
#ifdef USE_32_BIT
	if ( length & 0x2  ) {
		outsl(ioaddr + DATA_1, buf,  length >> 2 );
#if !defined(__H8300H__) && !defined(__H8300S__)
		outw( *((word *)(buf + (length & 0xFFFFFFFC))),ioaddr +DATA_1);
#else
		ctrl_outw( *((word *)(buf + (length & 0xFFFFFFFC))),ioaddr +DATA_1);
#endif
	}
	else
		outsl(ioaddr + DATA_1, buf,  length >> 2 );
#else
	outsw(ioaddr + DATA_1 , buf, (length ) >> 1);
#endif
	/* Send the last byte, if there is one.   */

	if ( (length & 1) == 0 ) {
		outw( 0, ioaddr + DATA_1 );
	} else {
		outb( buf[length -1 ], ioaddr + DATA_1 );
		outb( 0x20, ioaddr + DATA_1);
	}

	/* enable the interrupts */
	SMC_ENABLE_INT( (IM_TX_INT | IM_TX_EMPTY_INT) );

	/* and let the chipset deal with it */
	outw( MC_ENQUEUE , ioaddr + MMU_CMD );

	PRINTK2((CARDNAME": Sent packet of length %d\n", length));

	lp->saved_skb = NULL;
	dev_kfree_skb_any (skb);

	dev->trans_start = jiffies;

	/* we can send another packet */
	netif_wake_queue(dev);
}

static int io;
static int irq;
static int ifport;

struct net_device * __init smc_init(int unit)
{
	struct net_device *dev = alloc_etherdev(sizeof(struct smc_local));
	struct devlist *smcdev = smc_devlist;
	int err = 0;

	if (!dev)
		return ERR_PTR(-ENODEV);

	if (unit >= 0) {
		sprintf(dev->name, "eth%d", unit);
		netdev_boot_setup_check(dev);
		io = dev->base_addr;
		irq = dev->irq;
	}

	if (io > 0x1ff) {	/* Check a single specified location. */
		err = smc_probe(dev, io);
	} else if (io != 0) {	/* Don't probe at all. */
		err = -ENXIO;
	} else {
		for (;smcdev->port; smcdev++) {
			if (smc_probe(dev, smcdev->port) == 0)
				break;
		}
		if (!smcdev->port)
			err = -ENODEV;
	}
	if (err)
		goto out;
	err = register_netdev(dev);
	if (err)
		goto out1;
	return dev;
out1:
	free_irq(dev->irq, dev);
	release_region(dev->base_addr, SMC_IO_EXTENT);
out:
	free_netdev(dev);
	return ERR_PTR(err);
}

static int __init smc_findirq(int ioaddr)
{
#ifndef NO_AUTOPROBE
	int	timeout = 20;
	unsigned long cookie;


	cookie = probe_irq_on();

	/*
	 * What I try to do here is trigger an ALLOC_INT. This is done
	 * by allocating a small chunk of memory, which will give an interrupt
	 * when done.
	 */


	SMC_SELECT_BANK(2);
	/* enable ALLOCation interrupts ONLY */
	outb( IM_ALLOC_INT, ioaddr + INT_MASK );

	/*
 	 . Allocate 512 bytes of memory.  Note that the chip was just
	 . reset so all the memory is available
	*/
	outw( MC_ALLOC | 1, ioaddr + MMU_CMD );

	/*
	 . Wait until positive that the interrupt has been generated
	*/
	while ( timeout ) {
		byte	int_status;

		int_status = inb( ioaddr + INTERRUPT );

		if ( int_status & IM_ALLOC_INT )
			break;		/* got the interrupt */
		timeout--;
	}
	/* there is really nothing that I can do here if timeout fails,
	   as probe_irq_off will return a 0 anyway, which is what I
	   want in this case.   Plus, the clean up is needed in both
	   cases.  */

	/* DELAY HERE!
	   On a fast machine, the status might change before the interrupt
	   is given to the processor.  This means that the interrupt was
	   never detected, and probe_irq_off fails to report anything.
	   This should fix probe_irq_* problems.
	*/
	SMC_DELAY();
	SMC_DELAY();

	/* and disable all interrupts again */
	outb( 0, ioaddr + INT_MASK );

	/* and return what I found */
	return probe_irq_off(cookie);
#else /* NO_AUTOPROBE */
	struct devlist *smcdev;
	for (smcdev = smc_devlist; smcdev->port; smcdev++) {
		if (smcdev->port == ioaddr)
			return smcdev->irq;
	}
	return 0;
#endif
}

static const struct net_device_ops smc_netdev_ops = {
	.ndo_open		 = smc_open,
	.ndo_stop		= smc_close,
	.ndo_start_xmit    	= smc_wait_to_send_packet,
	.ndo_tx_timeout	    	= smc_timeout,
	.ndo_set_multicast_list	= smc_set_multicast_list,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_set_mac_address 	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};


static int __init smc_probe(struct net_device *dev, int ioaddr)
{
	int i, memory, retval;
	static unsigned version_printed;
	unsigned int bank;

	const char *version_string;
	const char *if_string;

	/* registers */
	word revision_register;
	word base_address_register;
	word configuration_register;
	word memory_info_register;
	word memory_cfg_register;

	/* Grab the region so that no one else tries to probe our ioports. */
	if (!request_region(ioaddr, SMC_IO_EXTENT, DRV_NAME))
		return -EBUSY;

	dev->irq = irq;
	dev->if_port = ifport;

	/* First, see if the high byte is 0x33 */
	bank = inw( ioaddr + BANK_SELECT );
	if ( (bank & 0xFF00) != 0x3300 ) {
		retval = -ENODEV;
		goto err_out;
	}
	/* The above MIGHT indicate a device, but I need to write to further
 	 	test this.  */
	outw( 0x0, ioaddr + BANK_SELECT );
	bank = inw( ioaddr + BANK_SELECT );
	if ( (bank & 0xFF00 ) != 0x3300 ) {
		retval = -ENODEV;
		goto err_out;
	}
#if !defined(CONFIG_H8S_EDOSK2674)
	/* well, we've already written once, so hopefully another time won't
 	   hurt.  This time, I need to switch the bank register to bank 1,
	   so I can access the base address register */
	SMC_SELECT_BANK(1);
	base_address_register = inw( ioaddr + BASE );
	if ( ioaddr != ( base_address_register >> 3 & 0x3E0 ) )  {
		printk(CARDNAME ": IOADDR %x doesn't match configuration (%x). "
			"Probably not a SMC chip\n",
			ioaddr, base_address_register >> 3 & 0x3E0 );
		/* well, the base address register didn't match.  Must not have
		   been a SMC chip after all. */
		retval = -ENODEV;
		goto err_out;
	}
#else
	(void)base_address_register; /* Warning suppression */
#endif


	/*  check if the revision register is something that I recognize.
	    These might need to be added to later, as future revisions
	    could be added.  */
	SMC_SELECT_BANK(3);
	revision_register  = inw( ioaddr + REVISION );
	if ( !chip_ids[ ( revision_register  >> 4 ) & 0xF  ] ) {
		/* I don't recognize this chip, so... */
		printk(CARDNAME ": IO %x: Unrecognized revision register:"
			" %x, Contact author.\n", ioaddr, revision_register);

		retval = -ENODEV;
		goto err_out;
	}

	/* at this point I'll assume that the chip is an SMC9xxx.
	   It might be prudent to check a listing of MAC addresses
	   against the hardware address, or do some other tests. */

	if (version_printed++ == 0)
		printk("%s", version);

	/* fill in some of the fields */
	dev->base_addr = ioaddr;

	/*
 	 . Get the MAC address ( bank 1, regs 4 - 9 )
	*/
	SMC_SELECT_BANK( 1 );
	for ( i = 0; i < 6; i += 2 ) {
		word	address;

		address = inw( ioaddr + ADDR0 + i  );
		dev->dev_addr[ i + 1] = address >> 8;
		dev->dev_addr[ i ] = address & 0xFF;
	}

	/* get the memory information */

	SMC_SELECT_BANK( 0 );
	memory_info_register = inw( ioaddr + MIR );
	memory_cfg_register  = inw( ioaddr + MCR );
	memory = ( memory_cfg_register >> 9 )  & 0x7;  /* multiplier */
	memory *= 256 * ( memory_info_register & 0xFF );

	/*
	 Now, I want to find out more about the chip.  This is sort of
 	 redundant, but it's cleaner to have it in both, rather than having
 	 one VERY long probe procedure.
	*/
	SMC_SELECT_BANK(3);
	revision_register  = inw( ioaddr + REVISION );
	version_string = chip_ids[ ( revision_register  >> 4 ) & 0xF  ];
	if ( !version_string ) {
		/* I shouldn't get here because this call was done before.... */
		retval = -ENODEV;
		goto err_out;
	}

	/* is it using AUI or 10BaseT ? */
	if ( dev->if_port == 0 ) {
		SMC_SELECT_BANK(1);
		configuration_register = inw( ioaddr + CONFIG );
		if ( configuration_register & CFG_AUI_SELECT )
			dev->if_port = 2;
		else
			dev->if_port = 1;
	}
	if_string = interfaces[ dev->if_port - 1 ];

	/* now, reset the chip, and put it into a known state */
	smc_reset( ioaddr );

	/*
	 . If dev->irq is 0, then the device has to be banged on to see
	 . what the IRQ is.
 	 .
	 . This banging doesn't always detect the IRQ, for unknown reasons.
	 . a workaround is to reset the chip and try again.
	 .
	 . Interestingly, the DOS packet driver *SETS* the IRQ on the card to
	 . be what is requested on the command line.   I don't do that, mostly
	 . because the card that I have uses a non-standard method of accessing
	 . the IRQs, and because this _should_ work in most configurations.
	 .
	 . Specifying an IRQ is done with the assumption that the user knows
	 . what (s)he is doing.  No checking is done!!!!
 	 .
	*/
	if ( dev->irq < 2 ) {
		int	trials;

		trials = 3;
		while ( trials-- ) {
			dev->irq = smc_findirq( ioaddr );
			if ( dev->irq )
				break;
			/* kick the card and try again */
			smc_reset( ioaddr );
		}
	}
	if (dev->irq == 0 ) {
		printk(CARDNAME": Couldn't autodetect your IRQ. Use irq=xx.\n");
		retval = -ENODEV;
		goto err_out;
	}

	/* now, print out the card info, in a short format.. */

	printk("%s: %s(r:%d) at %#3x IRQ:%d INTF:%s MEM:%db ", dev->name,
		version_string, revision_register & 0xF, ioaddr, dev->irq,
		if_string, memory );
	/*
	 . Print the Ethernet address
	*/
	printk("ADDR: %pM\n", dev->dev_addr);

	/* Grab the IRQ */
      	retval = request_irq(dev->irq, smc_interrupt, 0, DRV_NAME, dev);
      	if (retval) {
		printk("%s: unable to get IRQ %d (irqval=%d).\n", DRV_NAME,
			dev->irq, retval);
  	  	goto err_out;
      	}

	dev->netdev_ops			= &smc_netdev_ops;
	dev->watchdog_timeo		= HZ/20;

	return 0;

err_out:
	release_region(ioaddr, SMC_IO_EXTENT);
	return retval;
}

#if SMC_DEBUG > 2
static void print_packet( byte * buf, int length )
{
#if 0
	int i;
	int remainder;
	int lines;

	printk("Packet of length %d\n", length);
	lines = length / 16;
	remainder = length % 16;

	for ( i = 0; i < lines ; i ++ ) {
		int cur;

		for ( cur = 0; cur < 8; cur ++ ) {
			byte a, b;

			a = *(buf ++ );
			b = *(buf ++ );
			printk("%02x%02x ", a, b );
		}
		printk("\n");
	}
	for ( i = 0; i < remainder/2 ; i++ ) {
		byte a, b;

		a = *(buf ++ );
		b = *(buf ++ );
		printk("%02x%02x ", a, b );
	}
	printk("\n");
#endif
}
#endif


static int smc_open(struct net_device *dev)
{
	int	ioaddr = dev->base_addr;

	int	i;	/* used to set hw ethernet address */

	/* clear out all the junk that was put here before... */
	memset(netdev_priv(dev), 0, sizeof(struct smc_local));

	/* reset the hardware */

	smc_reset( ioaddr );
	smc_enable( ioaddr );

	/* Select which interface to use */

	SMC_SELECT_BANK( 1 );
	if ( dev->if_port == 1 ) {
		outw( inw( ioaddr + CONFIG ) & ~CFG_AUI_SELECT,
			ioaddr + CONFIG );
	}
	else if ( dev->if_port == 2 ) {
		outw( inw( ioaddr + CONFIG ) | CFG_AUI_SELECT,
			ioaddr + CONFIG );
	}

	/*
  		According to Becker, I have to set the hardware address
		at this point, because the (l)user can set it with an
		ioctl.  Easily done...
	*/
	SMC_SELECT_BANK( 1 );
	for ( i = 0; i < 6; i += 2 ) {
		word	address;

		address = dev->dev_addr[ i + 1 ] << 8 ;
		address  |= dev->dev_addr[ i ];
		outw( address, ioaddr + ADDR0 + i );
	}

	netif_start_queue(dev);
	return 0;
}


static void smc_timeout(struct net_device *dev)
{
	/* If we get here, some higher level has decided we are broken.
	   There should really be a "kick me" function call instead. */
	printk(KERN_WARNING CARDNAME": transmit timed out, %s?\n",
		tx_done(dev) ? "IRQ conflict" :
		"network cable problem");
	/* "kick" the adaptor */
	smc_reset( dev->base_addr );
	smc_enable( dev->base_addr );
	dev->trans_start = jiffies; /* prevent tx timeout */
	/* clear anything saved */
	((struct smc_local *)netdev_priv(dev))->saved_skb = NULL;
	netif_wake_queue(dev);
}

static void smc_rcv(struct net_device *dev)
{
	int 	ioaddr = dev->base_addr;
	int 	packet_number;
	word	status;
	word	packet_length;

	/* assume bank 2 */

	packet_number = inw( ioaddr + FIFO_PORTS );

	if ( packet_number & FP_RXEMPTY ) {
		/* we got called , but nothing was on the FIFO */
		PRINTK((CARDNAME ": WARNING: smc_rcv with nothing on FIFO.\n"));
		/* don't need to restore anything */
		return;
	}

	/*  start reading from the start of the packet */
	outw( PTR_READ | PTR_RCV | PTR_AUTOINC, ioaddr + POINTER );

	/* First two words are status and packet_length */
	status 		= inw( ioaddr + DATA_1 );
	packet_length 	= inw( ioaddr + DATA_1 );

	packet_length &= 0x07ff;  /* mask off top bits */

	PRINTK2(("RCV: STATUS %4x LENGTH %4x\n", status, packet_length ));
	/*
	 . the packet length contains 3 extra words :
	 . status, length, and an extra word with an odd byte .
	*/
	packet_length -= 6;

	if ( !(status & RS_ERRORS ) ){
		/* do stuff to make a new packet */
		struct sk_buff  * skb;
		byte		* data;

		/* read one extra byte */
		if ( status & RS_ODDFRAME )
			packet_length++;

		/* set multicast stats */
		if ( status & RS_MULTICAST )
			dev->stats.multicast++;

		skb = dev_alloc_skb( packet_length + 5);

		if ( skb == NULL ) {
			printk(KERN_NOTICE CARDNAME ": Low memory, packet dropped.\n");
			dev->stats.rx_dropped++;
			goto done;
		}

		/*
		 ! This should work without alignment, but it could be
		 ! in the worse case
		*/

		skb_reserve( skb, 2 );   /* 16 bit alignment */

		data = skb_put( skb, packet_length);

#ifdef USE_32_BIT
		/* QUESTION:  Like in the TX routine, do I want
		   to send the DWORDs or the bytes first, or some
		   mixture.  A mixture might improve already slow PIO
		   performance  */
		PRINTK3((" Reading %d dwords (and %d bytes)\n",
			packet_length >> 2, packet_length & 3 ));
		insl(ioaddr + DATA_1 , data, packet_length >> 2 );
		/* read the left over bytes */
		insb( ioaddr + DATA_1, data + (packet_length & 0xFFFFFC),
			packet_length & 0x3  );
#else
		PRINTK3((" Reading %d words and %d byte(s)\n",
			(packet_length >> 1 ), packet_length & 1 ));
		insw(ioaddr + DATA_1 , data, packet_length >> 1);
		if ( packet_length & 1 ) {
			data += packet_length & ~1;
			*(data++) = inb( ioaddr + DATA_1 );
		}
#endif
#if	SMC_DEBUG > 2
			print_packet( data, packet_length );
#endif

		skb->protocol = eth_type_trans(skb, dev );
		netif_rx(skb);
		dev->stats.rx_packets++;
		dev->stats.rx_bytes += packet_length;
	} else {
		/* error ... */
		dev->stats.rx_errors++;

		if ( status & RS_ALGNERR )  dev->stats.rx_frame_errors++;
		if ( status & (RS_TOOSHORT | RS_TOOLONG ) )
			dev->stats.rx_length_errors++;
		if ( status & RS_BADCRC)	dev->stats.rx_crc_errors++;
	}

done:
	/*  error or good, tell the card to get rid of this packet */
	outw( MC_RELEASE, ioaddr + MMU_CMD );
}


static void smc_tx( struct net_device * dev )
{
	int	ioaddr = dev->base_addr;
	struct smc_local *lp = netdev_priv(dev);
	byte saved_packet;
	byte packet_no;
	word tx_status;


	/* assume bank 2  */

	saved_packet = inb( ioaddr + PNR_ARR );
	packet_no = inw( ioaddr + FIFO_PORTS );
	packet_no &= 0x7F;

	/* select this as the packet to read from */
	outb( packet_no, ioaddr + PNR_ARR );

	/* read the first word from this packet */
	outw( PTR_AUTOINC | PTR_READ, ioaddr + POINTER );

	tx_status = inw( ioaddr + DATA_1 );
	PRINTK3((CARDNAME": TX DONE STATUS: %4x\n", tx_status));

	dev->stats.tx_errors++;
	if ( tx_status & TS_LOSTCAR ) dev->stats.tx_carrier_errors++;
	if ( tx_status & TS_LATCOL  ) {
		printk(KERN_DEBUG CARDNAME
			": Late collision occurred on last xmit.\n");
		dev->stats.tx_window_errors++;
	}
#if 0
		if ( tx_status & TS_16COL ) { ... }
#endif

	if ( tx_status & TS_SUCCESS ) {
		printk(CARDNAME": Successful packet caused interrupt\n");
	}
	/* re-enable transmit */
	SMC_SELECT_BANK( 0 );
	outw( inw( ioaddr + TCR ) | TCR_ENABLE, ioaddr + TCR );

	/* kill the packet */
	SMC_SELECT_BANK( 2 );
	outw( MC_FREEPKT, ioaddr + MMU_CMD );

	/* one less packet waiting for me */
	lp->packets_waiting--;

	outb( saved_packet, ioaddr + PNR_ARR );
}


static irqreturn_t smc_interrupt(int irq, void * dev_id)
{
	struct net_device *dev 	= dev_id;
	int ioaddr 		= dev->base_addr;
	struct smc_local *lp = netdev_priv(dev);

	byte	status;
	word	card_stats;
	byte	mask;
	int	timeout;
	/* state registers */
	word	saved_bank;
	word	saved_pointer;
	int handled = 0;


	PRINTK3((CARDNAME": SMC interrupt started\n"));

	saved_bank = inw( ioaddr + BANK_SELECT );

	SMC_SELECT_BANK(2);
	saved_pointer = inw( ioaddr + POINTER );

	mask = inb( ioaddr + INT_MASK );
	/* clear all interrupts */
	outb( 0, ioaddr + INT_MASK );


	/* set a timeout value, so I don't stay here forever */
	timeout = 4;

	PRINTK2((KERN_WARNING CARDNAME ": MASK IS %x\n", mask));
	do {
		/* read the status flag, and mask it */
		status = inb( ioaddr + INTERRUPT ) & mask;
		if (!status )
			break;

		handled = 1;

		PRINTK3((KERN_WARNING CARDNAME
			": Handling interrupt status %x\n", status));

		if (status & IM_RCV_INT) {
			/* Got a packet(s). */
			PRINTK2((KERN_WARNING CARDNAME
				": Receive Interrupt\n"));
			smc_rcv(dev);
		} else if (status & IM_TX_INT ) {
			PRINTK2((KERN_WARNING CARDNAME
				": TX ERROR handled\n"));
			smc_tx(dev);
			outb(IM_TX_INT, ioaddr + INTERRUPT );
		} else if (status & IM_TX_EMPTY_INT ) {
			/* update stats */
			SMC_SELECT_BANK( 0 );
			card_stats = inw( ioaddr + COUNTER );
			/* single collisions */
			dev->stats.collisions += card_stats & 0xF;
			card_stats >>= 4;
			/* multiple collisions */
			dev->stats.collisions += card_stats & 0xF;

			/* these are for when linux supports these statistics */

			SMC_SELECT_BANK( 2 );
			PRINTK2((KERN_WARNING CARDNAME
				": TX_BUFFER_EMPTY handled\n"));
			outb( IM_TX_EMPTY_INT, ioaddr + INTERRUPT );
			mask &= ~IM_TX_EMPTY_INT;
			dev->stats.tx_packets += lp->packets_waiting;
			lp->packets_waiting = 0;

		} else if (status & IM_ALLOC_INT ) {
			PRINTK2((KERN_DEBUG CARDNAME
				": Allocation interrupt\n"));
			/* clear this interrupt so it doesn't happen again */
			mask &= ~IM_ALLOC_INT;

			smc_hardware_send_packet( dev );

			/* enable xmit interrupts based on this */
			mask |= ( IM_TX_EMPTY_INT | IM_TX_INT );

			/* and let the card send more packets to me */
			netif_wake_queue(dev);

			PRINTK2((CARDNAME": Handoff done successfully.\n"));
		} else if (status & IM_RX_OVRN_INT ) {
			dev->stats.rx_errors++;
			dev->stats.rx_fifo_errors++;
			outb( IM_RX_OVRN_INT, ioaddr + INTERRUPT );
		} else if (status & IM_EPH_INT ) {
			PRINTK((CARDNAME ": UNSUPPORTED: EPH INTERRUPT\n"));
		} else if (status & IM_ERCV_INT ) {
			PRINTK((CARDNAME ": UNSUPPORTED: ERCV INTERRUPT\n"));
			outb( IM_ERCV_INT, ioaddr + INTERRUPT );
		}
	} while ( timeout -- );


	/* restore state register */
	SMC_SELECT_BANK( 2 );
	outb( mask, ioaddr + INT_MASK );

	PRINTK3((KERN_WARNING CARDNAME ": MASK is now %x\n", mask));
	outw( saved_pointer, ioaddr + POINTER );

	SMC_SELECT_BANK( saved_bank );

	PRINTK3((CARDNAME ": Interrupt done\n"));
	return IRQ_RETVAL(handled);
}


static int smc_close(struct net_device *dev)
{
	netif_stop_queue(dev);
	/* clear everything */
	smc_shutdown( dev->base_addr );

	/* Update the statistics here. */
	return 0;
}

static void smc_set_multicast_list(struct net_device *dev)
{
	short ioaddr = dev->base_addr;

	SMC_SELECT_BANK(0);
	if ( dev->flags & IFF_PROMISC )
		outw( inw(ioaddr + RCR ) | RCR_PROMISC, ioaddr + RCR );


	/* Here, I am setting this to accept all multicast packets.
	   I don't need to zero the multicast table, because the flag is
	   checked before the table is
	*/
	else if (dev->flags & IFF_ALLMULTI)
		outw( inw(ioaddr + RCR ) | RCR_ALMUL, ioaddr + RCR );

	/* We just get all multicast packets even if we only want them
	 . from one source.  This will be changed at some future
	 . point. */
	else if (!netdev_mc_empty(dev)) {
		/* support hardware multicasting */

		/* be sure I get rid of flags I might have set */
		outw( inw( ioaddr + RCR ) & ~(RCR_PROMISC | RCR_ALMUL),
			ioaddr + RCR );
		/* NOTE: this has to set the bank, so make sure it is the
		   last thing called.  The bank is set to zero at the top */
		smc_setmulticast(ioaddr, dev);
	}
	else  {
		outw( inw( ioaddr + RCR ) & ~(RCR_PROMISC | RCR_ALMUL),
			ioaddr + RCR );

		/*
		  since I'm disabling all multicast entirely, I need to
		  clear the multicast list
		*/
		SMC_SELECT_BANK( 3 );
		outw( 0, ioaddr + MULTICAST1 );
		outw( 0, ioaddr + MULTICAST2 );
		outw( 0, ioaddr + MULTICAST3 );
		outw( 0, ioaddr + MULTICAST4 );
	}
}

#ifdef MODULE

static struct net_device *devSMC9194;
MODULE_LICENSE("GPL");

module_param(io, int, 0);
module_param(irq, int, 0);
module_param(ifport, int, 0);
MODULE_PARM_DESC(io, "SMC 99194 I/O base address");
MODULE_PARM_DESC(irq, "SMC 99194 IRQ number");
MODULE_PARM_DESC(ifport, "SMC 99194 interface port (0-default, 1-TP, 2-AUI)");

int __init init_module(void)
{
	if (io == 0)
		printk(KERN_WARNING
		CARDNAME": You shouldn't use auto-probing with insmod!\n" );

	/* copy the parameters from insmod into the device structure */
	devSMC9194 = smc_init(-1);
	if (IS_ERR(devSMC9194))
		return PTR_ERR(devSMC9194);
	return 0;
}

void __exit cleanup_module(void)
{
	unregister_netdev(devSMC9194);
	free_irq(devSMC9194->irq, devSMC9194);
	release_region(devSMC9194->base_addr, SMC_IO_EXTENT);
	free_netdev(devSMC9194);
}

#endif /* MODULE */
