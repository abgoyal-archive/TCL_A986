

#ifndef DE600_IO
#define DE600_IO	0x378
#endif

#define DATA_PORT	(DE600_IO)
#define STATUS_PORT	(DE600_IO + 1)
#define COMMAND_PORT	(DE600_IO + 2)

#ifndef DE600_IRQ
#define DE600_IRQ	7
#endif

#define SELECT_NIC	0x04 /* select Network Interface Card */
#define SELECT_PRN	0x1c /* select Printer */
#define NML_PRN		0xec /* normal Printer situation */
#define IRQEN		0x10 /* enable IRQ line */

#define RX_BUSY		0x80
#define RX_GOOD		0x40
#define TX_FAILED16	0x10
#define TX_BUSY		0x08

#define WRITE_DATA	0x00 /* write memory */
#define READ_DATA	0x01 /* read memory */
#define STATUS		0x02 /* read  status register */
#define COMMAND		0x03 /* write command register (see COMMAND below) */
#define NULL_COMMAND	0x04 /* null command */
#define RX_LEN		0x05 /* read  received packet length */
#define TX_ADDR		0x06 /* set adapter transmit memory address */
#define RW_ADDR		0x07 /* set adapter read/write memory address */
#define HI_NIBBLE	0x08 /* read/write the high nibble of data,
				or-ed with rest of command */

#define RX_ALL		0x01 /* PROMISCUOUS */
#define RX_BP		0x02 /* default: BROADCAST & PHYSICAL ADDRESS */
#define RX_MBP		0x03 /* MULTICAST, BROADCAST & PHYSICAL ADDRESS */

#define TX_ENABLE	0x04 /* bit 2 */
#define RX_ENABLE	0x08 /* bit 3 */

#define RESET		0x80 /* set bit 7 high */
#define STOP_RESET	0x00 /* set bit 7 low */

#define RX_PAGE2_SELECT	0x10 /* bit 4, only 2 pages to select */
#define RX_BASE_PAGE	0x20 /* bit 5, always set when specifying RX_ADDR */
#define FLIP_IRQ	0x40 /* bit 6 */

#define MEM_2K		0x0800 /* 2048 */
#define MEM_4K		0x1000 /* 4096 */
#define MEM_6K		0x1800 /* 6144 */
#define NODE_ADDRESS	0x2000 /* 8192 */

#define RUNT 60		/* Too small Ethernet packet */


/* Routines used internally. (See "convenience macros") */
static u8	de600_read_status(struct net_device *dev);
static u8	de600_read_byte(unsigned char type, struct net_device *dev);

/* Put in the device structure. */
static int	de600_open(struct net_device *dev);
static int	de600_close(struct net_device *dev);
static int	de600_start_xmit(struct sk_buff *skb, struct net_device *dev);

/* Dispatch from interrupts. */
static irqreturn_t de600_interrupt(int irq, void *dev_id);
static int	de600_tx_intr(struct net_device *dev, int irq_status);
static void	de600_rx_intr(struct net_device *dev);

/* Initialization */
static void	trigger_interrupt(struct net_device *dev);
static int	adapter_init(struct net_device *dev);


#define select_prn() outb_p(SELECT_PRN, COMMAND_PORT); DE600_SLOW_DOWN
#define select_nic() outb_p(SELECT_NIC, COMMAND_PORT); DE600_SLOW_DOWN

/* Thanks for hints from Mark Burton <markb@ordern.demon.co.uk> */
#define de600_put_byte(data) ( \
	outb_p(((data) << 4)   | WRITE_DATA            , DATA_PORT), \
	outb_p(((data) & 0xf0) | WRITE_DATA | HI_NIBBLE, DATA_PORT))

#define de600_put_command(cmd) ( \
	outb_p(( rx_page        << 4)   | COMMAND            , DATA_PORT), \
	outb_p(( rx_page        & 0xf0) | COMMAND | HI_NIBBLE, DATA_PORT), \
	outb_p(((rx_page | cmd) << 4)   | COMMAND            , DATA_PORT), \
	outb_p(((rx_page | cmd) & 0xf0) | COMMAND | HI_NIBBLE, DATA_PORT))

#define de600_setup_address(addr,type) ( \
	outb_p((((addr) << 4) & 0xf0) | type            , DATA_PORT), \
	outb_p(( (addr)       & 0xf0) | type | HI_NIBBLE, DATA_PORT), \
	outb_p((((addr) >> 4) & 0xf0) | type            , DATA_PORT), \
	outb_p((((addr) >> 8) & 0xf0) | type | HI_NIBBLE, DATA_PORT))

#define rx_page_adr() ((rx_page & RX_PAGE2_SELECT)?(MEM_6K):(MEM_4K))

/* Flip bit, only 2 pages */
#define next_rx_page() (rx_page ^= RX_PAGE2_SELECT)

#define tx_page_adr(a) (((a) + 1) * MEM_2K)
