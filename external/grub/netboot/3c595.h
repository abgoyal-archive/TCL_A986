




#define TX_INIT_RATE         16
#define TX_INIT_MAX_RATE     64
#define RX_INIT_LATENCY      64
#define RX_INIT_EARLY_THRESH 64
#define MIN_RX_EARLY_THRESHF   16 /* not less than ether_header */
#define MIN_RX_EARLY_THRESHL   4

#define EEPROMSIZE      0x40
#define MAX_EEPROMBUSY  1000
#define VX_LAST_TAG     0xd7
#define VX_MAX_BOARDS   16
#define VX_ID_PORT      0x100

#define BASE 	(eth_nic_base)

#define EEPROM_CMD_RD    0x0080	/* Read:  Address required (5 bits) */
#define EEPROM_CMD_WR    0x0040	/* Write: Address required (5 bits) */
#define EEPROM_CMD_ERASE 0x00c0	/* Erase: Address required (5 bits) */
#define EEPROM_CMD_EWEN  0x0030	/* Erase/Write Enable: No data required */

#define EEPROM_BUSY		(1<<15)



#define EEPROM_NODE_ADDR_0	0x0	/* Word */
#define EEPROM_NODE_ADDR_1	0x1	/* Word */
#define EEPROM_NODE_ADDR_2	0x2	/* Word */
#define EEPROM_PROD_ID		0x3	/* 0x9[0-f]50 */
#define EEPROM_MFG_ID		0x7	/* 0x6d50 */
#define EEPROM_ADDR_CFG		0x8	/* Base addr */
#define EEPROM_RESOURCE_CFG	0x9	/* IRQ. Bits 12-15 */
#define EEPROM_OEM_ADDR_0	0xa	/* Word */
#define EEPROM_OEM_ADDR_1	0xb	/* Word */
#define EEPROM_OEM_ADDR_2	0xc	/* Word */
#define EEPROM_SOFT_INFO_2	0xf     /* Software information 2 */

#define NO_RX_OVN_ANOMALY       (1<<5)


#define VX_COMMAND		0x0e	/* Write. BASE+0x0e is always a
					 * command reg. */
#define VX_STATUS		0x0e	/* Read. BASE+0x0e is always status
					 * reg. */
#define VX_WINDOW		0x0f	/* Read. BASE+0x0f is always window
					 * reg. */
/* Write */
#define VX_W0_EEPROM_DATA	0x0c
#define VX_W0_EEPROM_COMMAND	0x0a
#define VX_W0_RESOURCE_CFG	0x08
#define VX_W0_ADDRESS_CFG	0x06 
#define VX_W0_CONFIG_CTRL	0x04
        /* Read */
#define VX_W0_PRODUCT_ID	0x02
#define VX_W0_MFG_ID		0x00


/* Write */
#define VX_W1_TX_PIO_WR_2	0x02
#define VX_W1_TX_PIO_WR_1	0x00
/* Read */
#define VX_W1_FREE_TX		0x0c
#define VX_W1_TX_STATUS		0x0b	/* byte */
#define VX_W1_TIMER		0x0a	/* byte */
#define VX_W1_RX_STATUS		0x08
#define VX_W1_RX_PIO_RD_2	0x02
#define VX_W1_RX_PIO_RD_1	0x00

/* Read/Write */
#define VX_W2_ADDR_5		0x05
#define VX_W2_ADDR_4		0x04
#define VX_W2_ADDR_3		0x03
#define VX_W2_ADDR_2		0x02
#define VX_W2_ADDR_1		0x01
#define VX_W2_ADDR_0		0x00

/* Read */
#define VX_W3_INTERNAL_CFG	0x00
#define VX_W3_RESET_OPT		0x08
#define VX_W3_FREE_TX		0x0c
#define VX_W3_FREE_RX		0x0a

/* Read/Write */
#define VX_W4_MEDIA_TYPE	0x0a
#define VX_W4_CTRLR_STATUS	0x08
#define VX_W4_NET_DIAG		0x06
#define VX_W4_FIFO_DIAG		0x04
#define VX_W4_HOST_DIAG		0x02
#define VX_W4_TX_DIAG		0x00

/* Read */
#define VX_W5_READ_0_MASK	0x0c
#define VX_W5_INTR_MASK		0x0a
#define VX_W5_RX_FILTER		0x08
#define VX_W5_RX_EARLY_THRESH	0x06
#define VX_W5_TX_AVAIL_THRESH	0x02
#define VX_W5_TX_START_THRESH	0x00

/* Read/Write */
#define TX_TOTAL_OK		0x0c
#define RX_TOTAL_OK		0x0a
#define TX_DEFERRALS		0x08
#define RX_FRAMES_OK		0x07
#define TX_FRAMES_OK		0x06
#define RX_OVERRUNS		0x05
#define TX_COLLISIONS		0x04
#define TX_AFTER_1_COLLISION	0x03
#define TX_AFTER_X_COLLISIONS	0x02
#define TX_NO_SQE		0x01
#define TX_CD_LOST		0x00


#define GLOBAL_RESET		(unsigned short) 0x0000	/* Wait at least 1ms
							 * after issuing */
#define WINDOW_SELECT		(unsigned short) (0x1<<11)
#define START_TRANSCEIVER	(unsigned short) (0x2<<11)	/* Read ADDR_CFG reg to
							 * determine whether
							 * this is needed. If
							 * so; wait 800 uSec
							 * before using trans-
							 * ceiver. */
#define RX_DISABLE		(unsigned short) (0x3<<11)	/* state disabled on
							 * power-up */
#define RX_ENABLE		(unsigned short) (0x4<<11)
#define RX_RESET		(unsigned short) (0x5<<11)
#define RX_DISCARD_TOP_PACK	(unsigned short) (0x8<<11)
#define TX_ENABLE		(unsigned short) (0x9<<11)
#define TX_DISABLE		(unsigned short) (0xa<<11)
#define TX_RESET		(unsigned short) (0xb<<11)
#define REQ_INTR		(unsigned short) (0xc<<11)
#define ACK_INTR		(unsigned short) (0x6800)
#	define C_INTR_LATCH	(unsigned short) (ACK_INTR|0x1)
#	define C_CARD_FAILURE	(unsigned short) (ACK_INTR|0x2)
#	define C_TX_COMPLETE	(unsigned short) (ACK_INTR|0x4)
#	define C_TX_AVAIL	(unsigned short) (ACK_INTR|0x8)
#	define C_RX_COMPLETE	(unsigned short) (ACK_INTR|0x10)
#	define C_RX_EARLY	(unsigned short) (ACK_INTR|0x20)
#	define C_INT_RQD		(unsigned short) (ACK_INTR|0x40)
#	define C_UPD_STATS	(unsigned short) (ACK_INTR|0x80)
#define SET_INTR_MASK		(unsigned short) (0xe<<11)
#define SET_RD_0_MASK		(unsigned short) (0xf<<11)
#define SET_RX_FILTER		(unsigned short) (0x10<<11)
#	define FIL_INDIVIDUAL	(unsigned short) (0x1)
#	define FIL_MULTICAST     (unsigned short) (0x02)
#	define FIL_BRDCST        (unsigned short) (0x04)
#	define FIL_PROMISC       (unsigned short) (0x08)
#define SET_RX_EARLY_THRESH	(unsigned short) (0x11<<11)
#define SET_TX_AVAIL_THRESH	(unsigned short) (0x12<<11)
#define SET_TX_START_THRESH	(unsigned short) (0x13<<11)
#define STATS_ENABLE		(unsigned short) (0x15<<11)
#define STATS_DISABLE		(unsigned short) (0x16<<11)
#define STOP_TRANSCEIVER	(unsigned short) (0x17<<11)

#define S_INTR_LATCH		(unsigned short) (0x1)
#define S_CARD_FAILURE		(unsigned short) (0x2)
#define S_TX_COMPLETE		(unsigned short) (0x4)
#define S_TX_AVAIL		(unsigned short) (0x8)
#define S_RX_COMPLETE		(unsigned short) (0x10)
#define S_RX_EARLY		(unsigned short) (0x20)
#define S_INT_RQD		(unsigned short) (0x40)
#define S_UPD_STATS		(unsigned short) (0x80)
#define S_COMMAND_IN_PROGRESS	(unsigned short) (0x1000)

#define VX_BUSY_WAIT while (inw(BASE + VX_STATUS) & S_COMMAND_IN_PROGRESS)


#define ACF_CONNECTOR_BITS	14  
#define ACF_CONNECTOR_UTP	0
#define ACF_CONNECTOR_AUI	1
#define ACF_CONNECTOR_BNC	3
   
#define INTERNAL_CONNECTOR_BITS 20
#define INTERNAL_CONNECTOR_MASK 0x01700000

#define ERR_INCOMPLETE  (unsigned short) (0x8000)
#define ERR_RX          (unsigned short) (0x4000)
#define ERR_MASK        (unsigned short) (0x7800)
#define ERR_OVERRUN     (unsigned short) (0x4000)
#define ERR_RUNT        (unsigned short) (0x5800)
#define ERR_ALIGNMENT   (unsigned short) (0x6000)
#define ERR_CRC         (unsigned short) (0x6800)
#define ERR_OVERSIZE    (unsigned short) (0x4800)
#define ERR_DRIBBLE     (unsigned short) (0x1000)

#define TXS_COMPLETE		0x80
#define TXS_INTR_REQ		0x40
#define TXS_JABBER		0x20
#define TXS_UNDERRUN		0x10
#define TXS_MAX_COLLISION	0x8
#define TXS_STATUS_OVERFLOW	0x4

#define RS_AUI			(1<<5)
#define RS_BNC			(1<<4)
#define RS_UTP			(1<<3)
#define	RS_T4			(1<<0)
#define	RS_TX			(1<<1)
#define	RS_FX			(1<<2)
#define	RS_MII			(1<<6)


#define FIFOS_RX_RECEIVING	(unsigned short) 0x8000
#define FIFOS_RX_UNDERRUN	(unsigned short) 0x2000
#define FIFOS_RX_STATUS_OVERRUN	(unsigned short) 0x1000
#define FIFOS_RX_OVERRUN	(unsigned short) 0x0800
#define FIFOS_TX_OVERRUN	(unsigned short) 0x0400

#define TAG_ADAPTER                     0xd0
#define ACTIVATE_ADAPTER_TO_CONFIG      0xff
#define ENABLE_DRQ_IRQ                  0x0001
#define MFG_ID                          0x506d  /* `TCM' */
#define PROD_ID                         0x5090
#define GO_WINDOW(x)		outw(WINDOW_SELECT|(x),BASE+VX_COMMAND)
#define JABBER_GUARD_ENABLE	0x40
#define LINKBEAT_ENABLE		0x80
#define	ENABLE_UTP		(JABBER_GUARD_ENABLE | LINKBEAT_ENABLE)
#define DISABLE_UTP		0x0
#define RX_BYTES_MASK		(unsigned short) (0x07ff)
#define RX_ERROR        0x4000
#define RX_INCOMPLETE   0x8000
#define TX_INDICATE		1<<15
#define is_eeprom_busy(b)	(inw((b)+VX_W0_EEPROM_COMMAND)&EEPROM_BUSY)

#define	VX_IOSIZE	0x20

#define VX_CONNECTORS 8

