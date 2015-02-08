
#ifndef _LINUX_ARCDEVICE_H
#define _LINUX_ARCDEVICE_H

#include <asm/timex.h>
#include <linux/if_arcnet.h>

#ifdef __KERNEL__

#ifndef bool
#define bool int
#endif

#define RECON_THRESHOLD 30


#define TX_TIMEOUT (HZ * 200 / 1000)


/* Display warnings about the driver being an ALPHA version. */
#undef ALPHA_WARNING


#define D_NORMAL	1	/* important operational info             */
#define D_EXTRA		2	/* useful, but non-vital information      */
#define	D_INIT		4	/* show init/probe messages               */
#define D_INIT_REASONS	8	/* show reasons for discarding probes     */
#define D_RECON		32	/* print a message whenever token is lost */
#define D_PROTO		64	/* debug auto-protocol support            */
/* debug levels below give LOTS of output during normal operation! */
#define D_DURING	128	/* trace operations (including irq's)     */
#define D_TX	        256	/* show tx packets                        */
#define D_RX		512	/* show rx packets                        */
#define D_SKB		1024	/* show skb's                             */
#define D_SKB_SIZE	2048	/* show skb sizes			  */
#define D_TIMING	4096	/* show time needed to copy buffers to card */
#define D_DEBUG         8192    /* Very detailed debug line for line */

#ifndef ARCNET_DEBUG_MAX
#define ARCNET_DEBUG_MAX (127)	/* change to ~0 if you want detailed debugging */
#endif

#ifndef ARCNET_DEBUG
#define ARCNET_DEBUG (D_NORMAL|D_EXTRA)
#endif
extern int arcnet_debug;

/* macros to simplify debug checking */
#define BUGLVL(x) if ((ARCNET_DEBUG_MAX)&arcnet_debug&(x))
#define BUGMSG2(x,msg,args...) do { BUGLVL(x) printk(msg, ## args); } while (0)
#define BUGMSG(x,msg,args...) \
	BUGMSG2(x, "%s%6s: " msg, \
            x==D_NORMAL	? KERN_WARNING \
            		: x < D_DURING ? KERN_INFO : KERN_DEBUG, \
	    dev->name , ## args)

/* see how long a function call takes to run, expressed in CPU cycles */
#define TIME(name, bytes, call) BUGLVL(D_TIMING) { \
	    unsigned long _x, _y; \
	    _x = get_cycles(); \
	    call; \
	    _y = get_cycles(); \
	    BUGMSG(D_TIMING, \
	       "%s: %d bytes in %lu cycles == " \
	       "%lu Kbytes/100Mcycle\n",\
		   name, bytes, _y - _x, \
		   100000000 / 1024 * bytes / (_y - _x + 1));\
	} \
	else { \
		    call;\
	}


#define RESETtime (300)

#define MTU	253		/* normal packet max size */
#define MinTU	257		/* extended packet min size */
#define XMTU	508		/* extended packet max size */

/* status/interrupt mask bit fields */
#define TXFREEflag	0x01	/* transmitter available */
#define TXACKflag       0x02	/* transmitted msg. ackd */
#define RECONflag       0x04	/* network reconfigured */
#define TESTflag        0x08	/* test flag */
#define EXCNAKflag      0x08    /* excesive nak flag */
#define RESETflag       0x10	/* power-on-reset */
#define RES1flag        0x20	/* reserved - usually set by jumper */
#define RES2flag        0x40	/* reserved - usually set by jumper */
#define NORXflag        0x80	/* receiver inhibited */

/* Flags used for IO-mapped memory operations */
#define AUTOINCflag     0x40	/* Increase location with each access */
#define IOMAPflag       0x02	/* (for 90xx) Use IO mapped memory, not mmap */
#define ENABLE16flag    0x80	/* (for 90xx) Enable 16-bit mode */

#define NOTXcmd         0x01	/* disable transmitter */
#define NORXcmd         0x02	/* disable receiver */
#define TXcmd           0x03	/* enable transmitter */
#define RXcmd           0x04	/* enable receiver */
#define CONFIGcmd       0x05	/* define configuration */
#define CFLAGScmd       0x06	/* clear flags */
#define TESTcmd         0x07	/* load test flags */

/* flags for "clear flags" command */
#define RESETclear      0x08	/* power-on-reset */
#define CONFIGclear     0x10	/* system reconfigured */

#define EXCNAKclear     0x0E    /* Clear and acknowledge the excive nak bit */

/* flags for "load test flags" command */
#define TESTload        0x08	/* test flag (diagnostic) */

/* byte deposited into first address of buffers on reset */
#define TESTvalue       0321	/* that's octal for 0xD1 :) */

/* for "enable receiver" command */
#define RXbcasts        0x80	/* receive broadcasts */

/* flags for "define configuration" command */
#define NORMALconf      0x00	/* 1-249 byte packets */
#define EXTconf         0x08	/* 250-504 byte packets */

#define ARC_IS_5MBIT    1   /* card default speed is 5MBit */
#define ARC_CAN_10MBIT  2   /* card uses COM20022, supporting 10MBit,
				 but default is 2.5MBit. */


/* information needed to define an encapsulation driver */
struct ArcProto {
	char suffix;		/* a for RFC1201, e for ether-encap, etc. */
	int mtu;		/* largest possible packet */
	int is_ip;              /* This is a ip plugin - not a raw thing */

	void (*rx) (struct net_device * dev, int bufnum,
		    struct archdr * pkthdr, int length);
	int (*build_header) (struct sk_buff * skb, struct net_device *dev,
			     unsigned short ethproto, uint8_t daddr);

	/* these functions return '1' if the skb can now be freed */
	int (*prepare_tx) (struct net_device * dev, struct archdr * pkt, int length,
			   int bufnum);
	int (*continue_tx) (struct net_device * dev, int bufnum);
	int (*ack_tx) (struct net_device * dev, int acked);
};

extern struct ArcProto *arc_proto_map[256], *arc_proto_default,
	*arc_bcast_proto, *arc_raw_proto;


struct Incoming {
	struct sk_buff *skb;	/* packet data buffer             */
	__be16 sequence;	/* sequence number of assembly    */
	uint8_t lastpacket,	/* number of last packet (from 1) */
		numpackets;	/* number of packets in split     */
};


/* only needed for RFC1201 */
struct Outgoing {
	struct ArcProto *proto;	/* protocol driver that owns this:
				 *   if NULL, no packet is pending.
				 */
	struct sk_buff *skb;	/* buffer from upper levels */
	struct archdr *pkt;	/* a pointer into the skb */
	uint16_t length,	/* bytes total */
		dataleft,	/* bytes left */
		segnum,		/* segment being sent */
		numsegs;	/* number of segments */
};


struct arcnet_local {
	uint8_t config,		/* current value of CONFIG register */
		timeout,	/* Extended timeout for COM20020 */
		backplane,	/* Backplane flag for COM20020 */
		clockp,		/* COM20020 clock divider */
		clockm,		/* COM20020 clock multiplier flag */
		setup,		/* Contents of setup1 register */
		setup2,		/* Contents of setup2 register */
		intmask;	/* current value of INTMASK register */
	uint8_t default_proto[256];	/* default encap to use for each host */
	int	cur_tx,		/* buffer used by current transmit, or -1 */
		next_tx,	/* buffer where a packet is ready to send */
		cur_rx;		/* current receive buffer */
	int	lastload_dest,	/* can last loaded packet be acked? */
		lasttrans_dest;	/* can last TX'd packet be acked? */
	int	timed_out;	/* need to process TX timeout and drop packet */
	unsigned long last_timeout;	/* time of last reported timeout */
	char *card_name;	/* card ident string */
	int card_flags;		/* special card features */


	/* On preemtive and SMB a lock is needed */
	spinlock_t lock;

	/*
	 * Buffer management: an ARCnet card has 4 x 512-byte buffers, each of
	 * which can be used for either sending or receiving.  The new dynamic
	 * buffer management routines use a simple circular queue of available
	 * buffers, and take them as they're needed.  This way, we simplify
	 * situations in which we (for example) want to pre-load a transmit
	 * buffer, or start receiving while we copy a received packet to
	 * memory.
	 * 
	 * The rules: only the interrupt handler is allowed to _add_ buffers to
	 * the queue; thus, this doesn't require a lock.  Both the interrupt
	 * handler and the transmit function will want to _remove_ buffers, so
	 * we need to handle the situation where they try to do it at the same
	 * time.
	 * 
	 * If next_buf == first_free_buf, the queue is empty.  Since there are
	 * only four possible buffers, the queue should never be full.
	 */
	atomic_t buf_lock;
	int buf_queue[5];
	int next_buf, first_free_buf;

	/* network "reconfiguration" handling */
	unsigned long first_recon; /* time of "first" RECON message to count */
	unsigned long last_recon;  /* time of most recent RECON */
	int num_recons;		/* number of RECONs between first and last. */
	bool network_down;	/* do we think the network is down? */

	bool excnak_pending;    /* We just got an excesive nak interrupt */

	struct {
		uint16_t sequence;	/* sequence number (incs with each packet) */
		__be16 aborted_seq;

		struct Incoming incoming[256];	/* one from each address */
	} rfc1201;

	/* really only used by rfc1201, but we'll pretend it's not */
	struct Outgoing outgoing;	/* packet currently being sent */

	/* hardware-specific functions */
	struct {
		struct module *owner;
		void (*command) (struct net_device * dev, int cmd);
		int (*status) (struct net_device * dev);
		void (*intmask) (struct net_device * dev, int mask);
		bool (*reset) (struct net_device * dev, bool really_reset);
		void (*open) (struct net_device * dev);
		void (*close) (struct net_device * dev);

		void (*copy_to_card) (struct net_device * dev, int bufnum, int offset,
				      void *buf, int count);
		void (*copy_from_card) (struct net_device * dev, int bufnum, int offset,
					void *buf, int count);
	} hw;

	void __iomem *mem_start;	/* pointer to ioremap'ed MMIO */
};


#define ARCRESET(x)  (lp->hw.reset(dev, (x)))
#define ACOMMAND(x)  (lp->hw.command(dev, (x)))
#define ASTATUS()    (lp->hw.status(dev))
#define AINTMASK(x)  (lp->hw.intmask(dev, (x)))



#if ARCNET_DEBUG_MAX & D_SKB
void arcnet_dump_skb(struct net_device *dev, struct sk_buff *skb, char *desc);
#else
#define arcnet_dump_skb(dev,skb,desc) ;
#endif

void arcnet_unregister_proto(struct ArcProto *proto);
irqreturn_t arcnet_interrupt(int irq, void *dev_id);
struct net_device *alloc_arcdev(const char *name);

int arcnet_open(struct net_device *dev);
int arcnet_close(struct net_device *dev);
netdev_tx_t arcnet_send_packet(struct sk_buff *skb,
				     struct net_device *dev);
void arcnet_timeout(struct net_device *dev);

#endif				/* __KERNEL__ */
#endif				/* _LINUX_ARCDEVICE_H */
