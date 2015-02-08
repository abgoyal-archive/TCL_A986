

#ifndef __LINUX_XHCI_HCD_H
#define __LINUX_XHCI_HCD_H

#include <linux/usb.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/usb/hcd.h>

/* Code sharing between pci-quirks and xhci hcd */
#include	"xhci-ext-caps.h"

/* xHCI PCI Configuration Registers */
#define XHCI_SBRN_OFFSET	(0x60)

/* Max number of USB devices for any host controller - limit in section 6.1 */
#define MAX_HC_SLOTS		256
/* Section 5.3.3 - MaxPorts */
#define MAX_HC_PORTS		127


struct xhci_cap_regs {
	u32	hc_capbase;
	u32	hcs_params1;
	u32	hcs_params2;
	u32	hcs_params3;
	u32	hcc_params;
	u32	db_off;
	u32	run_regs_off;
	/* Reserved up to (CAPLENGTH - 0x1C) */
};

/* hc_capbase bitmasks */
/* bits 7:0 - how long is the Capabilities register */
#define HC_LENGTH(p)		XHCI_HC_LENGTH(p)
/* bits 31:16	*/
#define HC_VERSION(p)		(((p) >> 16) & 0xffff)

/* HCSPARAMS1 - hcs_params1 - bitmasks */
/* bits 0:7, Max Device Slots */
#define HCS_MAX_SLOTS(p)	(((p) >> 0) & 0xff)
#define HCS_SLOTS_MASK		0xff
/* bits 8:18, Max Interrupters */
#define HCS_MAX_INTRS(p)	(((p) >> 8) & 0x7ff)
/* bits 24:31, Max Ports - max value is 0x7F = 127 ports */
#define HCS_MAX_PORTS(p)	(((p) >> 24) & 0x7f)

/* HCSPARAMS2 - hcs_params2 - bitmasks */
#define HCS_IST(p)		(((p) >> 0) & 0xf)
/* bits 4:7, max number of Event Ring segments */
#define HCS_ERST_MAX(p)		(((p) >> 4) & 0xf)
/* bit 26 Scratchpad restore - for save/restore HW state - not used yet */
/* bits 27:31 number of Scratchpad buffers SW must allocate for the HW */
#define HCS_MAX_SCRATCHPAD(p)   (((p) >> 27) & 0x1f)

/* HCSPARAMS3 - hcs_params3 - bitmasks */
/* bits 0:7, Max U1 to U0 latency for the roothub ports */
#define HCS_U1_LATENCY(p)	(((p) >> 0) & 0xff)
/* bits 16:31, Max U2 to U0 latency for the roothub ports */
#define HCS_U2_LATENCY(p)	(((p) >> 16) & 0xffff)

/* HCCPARAMS - hcc_params - bitmasks */
/* true: HC can use 64-bit address pointers */
#define HCC_64BIT_ADDR(p)	((p) & (1 << 0))
/* true: HC can do bandwidth negotiation */
#define HCC_BANDWIDTH_NEG(p)	((p) & (1 << 1))
#define HCC_64BYTE_CONTEXT(p)	((p) & (1 << 2))
/* true: HC has port power switches */
#define HCC_PPC(p)		((p) & (1 << 3))
/* true: HC has port indicators */
#define HCS_INDICATOR(p)	((p) & (1 << 4))
/* true: HC has Light HC Reset Capability */
#define HCC_LIGHT_RESET(p)	((p) & (1 << 5))
/* true: HC supports latency tolerance messaging */
#define HCC_LTC(p)		((p) & (1 << 6))
/* true: no secondary Stream ID Support */
#define HCC_NSS(p)		((p) & (1 << 7))
/* Max size for Primary Stream Arrays - 2^(n+1), where n is bits 12:15 */
#define HCC_MAX_PSA(p)		(1 << ((((p) >> 12) & 0xf) + 1))
/* Extended Capabilities pointer from PCI base - section 5.3.6 */
#define HCC_EXT_CAPS(p)		XHCI_HCC_EXT_CAPS(p)

/* db_off bitmask - bits 0:1 reserved */
#define	DBOFF_MASK	(~0x3)

/* run_regs_off bitmask - bits 0:4 reserved */
#define	RTSOFF_MASK	(~0x1f)


/* Number of registers per port */
#define	NUM_PORT_REGS	4

struct xhci_op_regs {
	u32	command;
	u32	status;
	u32	page_size;
	u32	reserved1;
	u32	reserved2;
	u32	dev_notification;
	u64	cmd_ring;
	/* rsvd: offset 0x20-2F */
	u32	reserved3[4];
	u64	dcbaa_ptr;
	u32	config_reg;
	/* rsvd: offset 0x3C-3FF */
	u32	reserved4[241];
	/* port 1 registers, which serve as a base address for other ports */
	u32	port_status_base;
	u32	port_power_base;
	u32	port_link_base;
	u32	reserved5;
	/* registers for ports 2-255 */
	u32	reserved6[NUM_PORT_REGS*254];
};

/* USBCMD - USB command - command bitmasks */
/* start/stop HC execution - do not write unless HC is halted*/
#define CMD_RUN		XHCI_CMD_RUN
#define CMD_RESET	(1 << 1)
/* Event Interrupt Enable - a '1' allows interrupts from the host controller */
#define CMD_EIE		XHCI_CMD_EIE
/* Host System Error Interrupt Enable - get out-of-band signal for HC errors */
#define CMD_HSEIE	XHCI_CMD_HSEIE
/* bits 4:6 are reserved (and should be preserved on writes). */
/* light reset (port status stays unchanged) - reset completed when this is 0 */
#define CMD_LRESET	(1 << 7)
/* FIXME: ignoring host controller save/restore state for now. */
#define CMD_CSS		(1 << 8)
#define CMD_CRS		(1 << 9)
/* Enable Wrap Event - '1' means xHC generates an event when MFINDEX wraps. */
#define CMD_EWE		XHCI_CMD_EWE
#define CMD_PM_INDEX	(1 << 11)
/* bits 12:31 are reserved (and should be preserved on writes). */

/* USBSTS - USB status - status bitmasks */
/* HC not running - set to 1 when run/stop bit is cleared. */
#define STS_HALT	XHCI_STS_HALT
/* serious error, e.g. PCI parity error.  The HC will clear the run/stop bit. */
#define STS_FATAL	(1 << 2)
/* event interrupt - clear this prior to clearing any IP flags in IR set*/
#define STS_EINT	(1 << 3)
/* port change detect */
#define STS_PORT	(1 << 4)
/* bits 5:7 reserved and zeroed */
/* save state status - '1' means xHC is saving state */
#define STS_SAVE	(1 << 8)
/* restore state status - '1' means xHC is restoring state */
#define STS_RESTORE	(1 << 9)
/* true: save or restore error */
#define STS_SRE		(1 << 10)
/* true: Controller Not Ready to accept doorbell or op reg writes after reset */
#define STS_CNR		XHCI_STS_CNR
/* true: internal Host Controller Error - SW needs to reset and reinitialize */
#define STS_HCE		(1 << 12)
/* bits 13:31 reserved and should be preserved */

#define	DEV_NOTE_MASK		(0xffff)
#define ENABLE_DEV_NOTE(x)	(1 << x)
#define	DEV_NOTE_FWAKE		ENABLE_DEV_NOTE(1)

/* CRCR - Command Ring Control Register - cmd_ring bitmasks */
/* bit 0 is the command ring cycle state */
/* stop ring operation after completion of the currently executing command */
#define CMD_RING_PAUSE		(1 << 1)
/* stop ring immediately - abort the currently executing command */
#define CMD_RING_ABORT		(1 << 2)
/* true: command ring is running */
#define CMD_RING_RUNNING	(1 << 3)
/* bits 4:5 reserved and should be preserved */
/* Command Ring pointer - bit mask for the lower 32 bits. */
#define CMD_RING_RSVD_BITS	(0x3f)

/* CONFIG - Configure Register - config_reg bitmasks */
/* bits 0:7 - maximum number of device slots enabled (NumSlotsEn) */
#define MAX_DEVS(p)	((p) & 0xff)
/* bits 8:31 - reserved and should be preserved */

/* PORTSC - Port Status and Control Register - port_status_base bitmasks */
/* true: device connected */
#define PORT_CONNECT	(1 << 0)
/* true: port enabled */
#define PORT_PE		(1 << 1)
/* bit 2 reserved and zeroed */
/* true: port has an over-current condition */
#define PORT_OC		(1 << 3)
/* true: port reset signaling asserted */
#define PORT_RESET	(1 << 4)
/* true: port has power (see HCC_PPC) */
#define PORT_POWER	(1 << 9)
#define DEV_SPEED_MASK		(0xf << 10)
#define	XDEV_FS			(0x1 << 10)
#define	XDEV_LS			(0x2 << 10)
#define	XDEV_HS			(0x3 << 10)
#define	XDEV_SS			(0x4 << 10)
#define DEV_UNDEFSPEED(p)	(((p) & DEV_SPEED_MASK) == (0x0<<10))
#define DEV_FULLSPEED(p)	(((p) & DEV_SPEED_MASK) == XDEV_FS)
#define DEV_LOWSPEED(p)		(((p) & DEV_SPEED_MASK) == XDEV_LS)
#define DEV_HIGHSPEED(p)	(((p) & DEV_SPEED_MASK) == XDEV_HS)
#define DEV_SUPERSPEED(p)	(((p) & DEV_SPEED_MASK) == XDEV_SS)
/* Bits 20:23 in the Slot Context are the speed for the device */
#define	SLOT_SPEED_FS		(XDEV_FS << 10)
#define	SLOT_SPEED_LS		(XDEV_LS << 10)
#define	SLOT_SPEED_HS		(XDEV_HS << 10)
#define	SLOT_SPEED_SS		(XDEV_SS << 10)
/* Port Indicator Control */
#define PORT_LED_OFF	(0 << 14)
#define PORT_LED_AMBER	(1 << 14)
#define PORT_LED_GREEN	(2 << 14)
#define PORT_LED_MASK	(3 << 14)
/* Port Link State Write Strobe - set this when changing link state */
#define PORT_LINK_STROBE	(1 << 16)
/* true: connect status change */
#define PORT_CSC	(1 << 17)
/* true: port enable change */
#define PORT_PEC	(1 << 18)
#define PORT_WRC	(1 << 19)
/* true: over-current change */
#define PORT_OCC	(1 << 20)
/* true: reset change - 1 to 0 transition of PORT_RESET */
#define PORT_RC		(1 << 21)
#define PORT_PLC	(1 << 22)
/* port configure error change - port failed to configure its link partner */
#define PORT_CEC	(1 << 23)
/* bit 24 reserved */
/* wake on connect (enable) */
#define PORT_WKCONN_E	(1 << 25)
/* wake on disconnect (enable) */
#define PORT_WKDISC_E	(1 << 26)
/* wake on over-current (enable) */
#define PORT_WKOC_E	(1 << 27)
/* bits 28:29 reserved */
/* true: device is removable - for USB 3.0 roothub emulation */
#define PORT_DEV_REMOVE	(1 << 30)
/* Initiate a warm port reset - complete when PORT_WRC is '1' */
#define PORT_WR		(1 << 31)

/* Port Power Management Status and Control - port_power_base bitmasks */
#define PORT_U1_TIMEOUT(p)	((p) & 0xff)
/* Inactivity timer value for transitions into U2 */
#define PORT_U2_TIMEOUT(p)	(((p) & 0xff) << 8)
/* Bits 24:31 for port testing */


struct xhci_intr_reg {
	u32	irq_pending;
	u32	irq_control;
	u32	erst_size;
	u32	rsvd;
	u64	erst_base;
	u64	erst_dequeue;
};

/* irq_pending bitmasks */
#define	ER_IRQ_PENDING(p)	((p) & 0x1)
/* bits 2:31 need to be preserved */
/* THIS IS BUGGY - FIXME - IP IS WRITE 1 TO CLEAR */
#define	ER_IRQ_CLEAR(p)		((p) & 0xfffffffe)
#define	ER_IRQ_ENABLE(p)	((ER_IRQ_CLEAR(p)) | 0x2)
#define	ER_IRQ_DISABLE(p)	((ER_IRQ_CLEAR(p)) & ~(0x2))

/* irq_control bitmasks */
#define ER_IRQ_INTERVAL_MASK	(0xffff)
/* Counter used to count down the time to the next interrupt - HW use only */
#define ER_IRQ_COUNTER_MASK	(0xffff << 16)

/* erst_size bitmasks */
/* Preserve bits 16:31 of erst_size */
#define	ERST_SIZE_MASK		(0xffff << 16)

/* erst_dequeue bitmasks */
#define ERST_DESI_MASK		(0x7)
#define ERST_EHB		(1 << 3)
#define ERST_PTR_MASK		(0xf)

struct xhci_run_regs {
	u32			microframe_index;
	u32			rsvd[7];
	struct xhci_intr_reg	ir_set[128];
};

struct xhci_doorbell_array {
	u32	doorbell[256];
};

#define	DB_TARGET_MASK		0xFFFFFF00
#define	DB_STREAM_ID_MASK	0x0000FFFF
#define	DB_TARGET_HOST		0x0
#define	DB_STREAM_ID_HOST	0x0
#define	DB_MASK			(0xff << 8)

/* Endpoint Target - bits 0:7 */
#define EPI_TO_DB(p)		(((p) + 1) & 0xff)
#define STREAM_ID_TO_DB(p)	(((p) & 0xffff) << 16)


struct xhci_container_ctx {
	unsigned type;
#define XHCI_CTX_TYPE_DEVICE  0x1
#define XHCI_CTX_TYPE_INPUT   0x2

	int size;

	u8 *bytes;
	dma_addr_t dma;
};

struct xhci_slot_ctx {
	u32	dev_info;
	u32	dev_info2;
	u32	tt_info;
	u32	dev_state;
	/* offset 0x10 to 0x1f reserved for HC internal use */
	u32	reserved[4];
};

/* dev_info bitmasks */
/* Route String - 0:19 */
#define ROUTE_STRING_MASK	(0xfffff)
/* Device speed - values defined by PORTSC Device Speed field - 20:23 */
#define DEV_SPEED	(0xf << 20)
/* bit 24 reserved */
/* Is this LS/FS device connected through a HS hub? - bit 25 */
#define DEV_MTT		(0x1 << 25)
/* Set if the device is a hub - bit 26 */
#define DEV_HUB		(0x1 << 26)
/* Index of the last valid endpoint context in this device context - 27:31 */
#define LAST_CTX_MASK	(0x1f << 27)
#define LAST_CTX(p)	((p) << 27)
#define LAST_CTX_TO_EP_NUM(p)	(((p) >> 27) - 1)
#define SLOT_FLAG	(1 << 0)
#define EP0_FLAG	(1 << 1)

/* dev_info2 bitmasks */
/* Max Exit Latency (ms) - worst case time to wake up all links in dev path */
#define MAX_EXIT	(0xffff)
/* Root hub port number that is needed to access the USB device */
#define ROOT_HUB_PORT(p)	(((p) & 0xff) << 16)
/* Maximum number of ports under a hub device */
#define XHCI_MAX_PORTS(p)	(((p) & 0xff) << 24)

/* tt_info bitmasks */
#define TT_SLOT		(0xff)
#define TT_PORT		(0xff << 8)
#define TT_THINK_TIME(p)	(((p) & 0x3) << 16)

/* dev_state bitmasks */
/* USB device address - assigned by the HC */
#define DEV_ADDR_MASK	(0xff)
/* bits 8:26 reserved */
/* Slot state */
#define SLOT_STATE	(0x1f << 27)
#define GET_SLOT_STATE(p)	(((p) & (0x1f << 27)) >> 27)


struct xhci_ep_ctx {
	u32	ep_info;
	u32	ep_info2;
	u64	deq;
	u32	tx_info;
	/* offset 0x14 - 0x1f reserved for HC internal use */
	u32	reserved[3];
};

/* ep_info bitmasks */
#define EP_STATE_MASK		(0xf)
#define EP_STATE_DISABLED	0
#define EP_STATE_RUNNING	1
#define EP_STATE_HALTED		2
#define EP_STATE_STOPPED	3
#define EP_STATE_ERROR		4
/* Mult - Max number of burtst within an interval, in EP companion desc. */
#define EP_MULT(p)		((p & 0x3) << 8)
/* bits 10:14 are Max Primary Streams */
/* bit 15 is Linear Stream Array */
/* Interval - period between requests to an endpoint - 125u increments. */
#define EP_INTERVAL(p)		((p & 0xff) << 16)
#define EP_INTERVAL_TO_UFRAMES(p)		(1 << (((p) >> 16) & 0xff))
#define EP_MAXPSTREAMS_MASK	(0x1f << 10)
#define EP_MAXPSTREAMS(p)	(((p) << 10) & EP_MAXPSTREAMS_MASK)
/* Endpoint is set up with a Linear Stream Array (vs. Secondary Stream Array) */
#define	EP_HAS_LSA		(1 << 15)

/* ep_info2 bitmasks */
#define	FORCE_EVENT	(0x1)
#define ERROR_COUNT(p)	(((p) & 0x3) << 1)
#define CTX_TO_EP_TYPE(p)	(((p) >> 3) & 0x7)
#define EP_TYPE(p)	((p) << 3)
#define ISOC_OUT_EP	1
#define BULK_OUT_EP	2
#define INT_OUT_EP	3
#define CTRL_EP		4
#define ISOC_IN_EP	5
#define BULK_IN_EP	6
#define INT_IN_EP	7
/* bit 6 reserved */
/* bit 7 is Host Initiate Disable - for disabling stream selection */
#define MAX_BURST(p)	(((p)&0xff) << 8)
#define MAX_PACKET(p)	(((p)&0xffff) << 16)
#define MAX_PACKET_MASK		(0xffff << 16)
#define MAX_PACKET_DECODED(p)	(((p) >> 16) & 0xffff)

/* tx_info bitmasks */
#define AVG_TRB_LENGTH_FOR_EP(p)	((p) & 0xffff)
#define MAX_ESIT_PAYLOAD_FOR_EP(p)	(((p) & 0xffff) << 16)


struct xhci_input_control_ctx {
	u32	drop_flags;
	u32	add_flags;
	u32	rsvd2[6];
};

struct xhci_command {
	/* Input context for changing device state */
	struct xhci_container_ctx	*in_ctx;
	u32				status;
	/* If completion is null, no one is waiting on this command
	 * and the structure can be freed after the command completes.
	 */
	struct completion		*completion;
	union xhci_trb			*command_trb;
	struct list_head		cmd_list;
};

/* drop context bitmasks */
#define	DROP_EP(x)	(0x1 << x)
/* add context bitmasks */
#define	ADD_EP(x)	(0x1 << x)

struct xhci_stream_ctx {
	/* 64-bit stream ring address, cycle state, and stream type */
	u64	stream_ring;
	/* offset 0x14 - 0x1f reserved for HC internal use */
	u32	reserved[2];
};

/* Stream Context Types (section 6.4.1) - bits 3:1 of stream ctx deq ptr */
#define	SCT_FOR_CTX(p)		(((p) << 1) & 0x7)
/* Secondary stream array type, dequeue pointer is to a transfer ring */
#define	SCT_SEC_TR		0
/* Primary stream array type, dequeue pointer is to a transfer ring */
#define	SCT_PRI_TR		1
/* Dequeue pointer is for a secondary stream array (SSA) with 8 entries */
#define SCT_SSA_8		2
#define SCT_SSA_16		3
#define SCT_SSA_32		4
#define SCT_SSA_64		5
#define SCT_SSA_128		6
#define SCT_SSA_256		7

/* Assume no secondary streams for now */
struct xhci_stream_info {
	struct xhci_ring		**stream_rings;
	/* Number of streams, including stream 0 (which drivers can't use) */
	unsigned int			num_streams;
	/* The stream context array may be bigger than
	 * the number of streams the driver asked for
	 */
	struct xhci_stream_ctx		*stream_ctx_array;
	unsigned int			num_stream_ctxs;
	dma_addr_t			ctx_array_dma;
	/* For mapping physical TRB addresses to segments in stream rings */
	struct radix_tree_root		trb_address_map;
	struct xhci_command		*free_streams_command;
};

#define	SMALL_STREAM_ARRAY_SIZE		256
#define	MEDIUM_STREAM_ARRAY_SIZE	1024

struct xhci_virt_ep {
	struct xhci_ring		*ring;
	/* Related to endpoints that are configured to use stream IDs only */
	struct xhci_stream_info		*stream_info;
	/* Temporary storage in case the configure endpoint command fails and we
	 * have to restore the device state to the previous state
	 */
	struct xhci_ring		*new_ring;
	unsigned int			ep_state;
#define SET_DEQ_PENDING		(1 << 0)
#define EP_HALTED		(1 << 1)	/* For stall handling */
#define EP_HALT_PENDING		(1 << 2)	/* For URB cancellation */
/* Transitioning the endpoint to using streams, don't enqueue URBs */
#define EP_GETTING_STREAMS	(1 << 3)
#define EP_HAS_STREAMS		(1 << 4)
/* Transitioning the endpoint to not using streams, don't enqueue URBs */
#define EP_GETTING_NO_STREAMS	(1 << 5)
	/* ----  Related to URB cancellation ---- */
	struct list_head	cancelled_td_list;
	/* The TRB that was last reported in a stopped endpoint ring */
	union xhci_trb		*stopped_trb;
	struct xhci_td		*stopped_td;
	unsigned int		stopped_stream;
	/* Watchdog timer for stop endpoint command to cancel URBs */
	struct timer_list	stop_cmd_timer;
	int			stop_cmds_pending;
	struct xhci_hcd		*xhci;
};

struct xhci_virt_device {
	/*
	 * Commands to the hardware are passed an "input context" that
	 * tells the hardware what to change in its data structures.
	 * The hardware will return changes in an "output context" that
	 * software must allocate for the hardware.  We need to keep
	 * track of input and output contexts separately because
	 * these commands might fail and we don't trust the hardware.
	 */
	struct xhci_container_ctx       *out_ctx;
	/* Used for addressing devices and configuration changes */
	struct xhci_container_ctx       *in_ctx;
	/* Rings saved to ensure old alt settings can be re-instated */
	struct xhci_ring		**ring_cache;
	int				num_rings_cached;
#define	XHCI_MAX_RINGS_CACHED	31
	struct xhci_virt_ep		eps[31];
	struct completion		cmd_completion;
	/* Status of the last command issued for this device */
	u32				cmd_status;
	struct list_head		cmd_list;
};


struct xhci_device_context_array {
	/* 64-bit device addresses; we only write 32-bit addresses */
	u64			dev_context_ptrs[MAX_HC_SLOTS];
	/* private xHCD pointers */
	dma_addr_t	dma;
};
/* TODO: write function to set the 64-bit device DMA address */


struct xhci_transfer_event {
	/* 64-bit buffer address, or immediate data */
	u64	buffer;
	u32	transfer_len;
	/* This field is interpreted differently based on the type of TRB */
	u32	flags;
};

/** Transfer Event bit fields **/
#define	TRB_TO_EP_ID(p)	(((p) >> 16) & 0x1f)

/* Completion Code - only applicable for some types of TRBs */
#define	COMP_CODE_MASK		(0xff << 24)
#define GET_COMP_CODE(p)	(((p) & COMP_CODE_MASK) >> 24)
#define COMP_SUCCESS	1
/* Data Buffer Error */
#define COMP_DB_ERR	2
/* Babble Detected Error */
#define COMP_BABBLE	3
/* USB Transaction Error */
#define COMP_TX_ERR	4
/* TRB Error - some TRB field is invalid */
#define COMP_TRB_ERR	5
/* Stall Error - USB device is stalled */
#define COMP_STALL	6
/* Resource Error - HC doesn't have memory for that device configuration */
#define COMP_ENOMEM	7
/* Bandwidth Error - not enough room in schedule for this dev config */
#define COMP_BW_ERR	8
/* No Slots Available Error - HC ran out of device slots */
#define COMP_ENOSLOTS	9
/* Invalid Stream Type Error */
#define COMP_STREAM_ERR	10
/* Slot Not Enabled Error - doorbell rung for disabled device slot */
#define COMP_EBADSLT	11
/* Endpoint Not Enabled Error */
#define COMP_EBADEP	12
/* Short Packet */
#define COMP_SHORT_TX	13
/* Ring Underrun - doorbell rung for an empty isoc OUT ep ring */
#define COMP_UNDERRUN	14
/* Ring Overrun - isoc IN ep ring is empty when ep is scheduled to RX */
#define COMP_OVERRUN	15
/* Virtual Function Event Ring Full Error */
#define COMP_VF_FULL	16
/* Parameter Error - Context parameter is invalid */
#define COMP_EINVAL	17
/* Bandwidth Overrun Error - isoc ep exceeded its allocated bandwidth */
#define COMP_BW_OVER	18
/* Context State Error - illegal context state transition requested */
#define COMP_CTX_STATE	19
/* No Ping Response Error - HC didn't get PING_RESPONSE in time to TX */
#define COMP_PING_ERR	20
/* Event Ring is full */
#define COMP_ER_FULL	21
/* Missed Service Error - HC couldn't service an isoc ep within interval */
#define COMP_MISSED_INT	23
/* Successfully stopped command ring */
#define COMP_CMD_STOP	24
/* Successfully aborted current command and stopped command ring */
#define COMP_CMD_ABORT	25
/* Stopped - transfer was terminated by a stop endpoint command */
#define COMP_STOP	26
/* Same as COMP_EP_STOPPED, but the transfered length in the event is invalid */
#define COMP_STOP_INVAL	27
/* Control Abort Error - Debug Capability - control pipe aborted */
#define COMP_DBG_ABORT	28
/* TRB type 29 and 30 reserved */
/* Isoc Buffer Overrun - an isoc IN ep sent more data than could fit in TD */
#define COMP_BUFF_OVER	31
/* Event Lost Error - xHC has an "internal event overrun condition" */
#define COMP_ISSUES	32
/* Undefined Error - reported when other error codes don't apply */
#define COMP_UNKNOWN	33
/* Invalid Stream ID Error */
#define COMP_STRID_ERR	34
/* Secondary Bandwidth Error - may be returned by a Configure Endpoint cmd */
/* FIXME - check for this */
#define COMP_2ND_BW_ERR	35
/* Split Transaction Error */
#define	COMP_SPLIT_ERR	36

struct xhci_link_trb {
	/* 64-bit segment pointer*/
	u64 segment_ptr;
	u32 intr_target;
	u32 control;
};

/* control bitfields */
#define LINK_TOGGLE	(0x1<<1)

/* Command completion event TRB */
struct xhci_event_cmd {
	/* Pointer to command TRB, or the value passed by the event data trb */
	u64 cmd_trb;
	u32 status;
	u32 flags;
};

/* flags bitmasks */
/* bits 16:23 are the virtual function ID */
/* bits 24:31 are the slot ID */
#define TRB_TO_SLOT_ID(p)	(((p) & (0xff<<24)) >> 24)
#define SLOT_ID_FOR_TRB(p)	(((p) & 0xff) << 24)

/* Stop Endpoint TRB - ep_index to endpoint ID for this TRB */
#define TRB_TO_EP_INDEX(p)		((((p) & (0x1f << 16)) >> 16) - 1)
#define	EP_ID_FOR_TRB(p)		((((p) + 1) & 0x1f) << 16)

/* Set TR Dequeue Pointer command TRB fields */
#define TRB_TO_STREAM_ID(p)		((((p) & (0xffff << 16)) >> 16))
#define STREAM_ID_FOR_TRB(p)		((((p)) & 0xffff) << 16)


/* Port Status Change Event TRB fields */
/* Port ID - bits 31:24 */
#define GET_PORT_ID(p)		(((p) & (0xff << 24)) >> 24)

/* Normal TRB fields */
/* transfer_len bitmasks - bits 0:16 */
#define	TRB_LEN(p)		((p) & 0x1ffff)
/* Interrupter Target - which MSI-X vector to target the completion event at */
#define TRB_INTR_TARGET(p)	(((p) & 0x3ff) << 22)
#define GET_INTR_TARGET(p)	(((p) >> 22) & 0x3ff)

/* Cycle bit - indicates TRB ownership by HC or HCD */
#define TRB_CYCLE		(1<<0)
#define TRB_ENT			(1<<1)
/* Interrupt on short packet */
#define TRB_ISP			(1<<2)
/* Set PCIe no snoop attribute */
#define TRB_NO_SNOOP		(1<<3)
/* Chain multiple TRBs into a TD */
#define TRB_CHAIN		(1<<4)
/* Interrupt on completion */
#define TRB_IOC			(1<<5)
/* The buffer pointer contains immediate data */
#define TRB_IDT			(1<<6)


/* Control transfer TRB specific fields */
#define TRB_DIR_IN		(1<<16)

struct xhci_generic_trb {
	u32 field[4];
};

union xhci_trb {
	struct xhci_link_trb		link;
	struct xhci_transfer_event	trans_event;
	struct xhci_event_cmd		event_cmd;
	struct xhci_generic_trb		generic;
};

/* TRB bit mask */
#define	TRB_TYPE_BITMASK	(0xfc00)
#define TRB_TYPE(p)		((p) << 10)
#define TRB_FIELD_TO_TYPE(p)	(((p) & TRB_TYPE_BITMASK) >> 10)
/* TRB type IDs */
/* bulk, interrupt, isoc scatter/gather, and control data stage */
#define TRB_NORMAL		1
/* setup stage for control transfers */
#define TRB_SETUP		2
/* data stage for control transfers */
#define TRB_DATA		3
/* status stage for control transfers */
#define TRB_STATUS		4
/* isoc transfers */
#define TRB_ISOC		5
/* TRB for linking ring segments */
#define TRB_LINK		6
#define TRB_EVENT_DATA		7
/* Transfer Ring No-op (not for the command ring) */
#define TRB_TR_NOOP		8
/* Command TRBs */
/* Enable Slot Command */
#define TRB_ENABLE_SLOT		9
/* Disable Slot Command */
#define TRB_DISABLE_SLOT	10
/* Address Device Command */
#define TRB_ADDR_DEV		11
/* Configure Endpoint Command */
#define TRB_CONFIG_EP		12
/* Evaluate Context Command */
#define TRB_EVAL_CONTEXT	13
/* Reset Endpoint Command */
#define TRB_RESET_EP		14
/* Stop Transfer Ring Command */
#define TRB_STOP_RING		15
/* Set Transfer Ring Dequeue Pointer Command */
#define TRB_SET_DEQ		16
/* Reset Device Command */
#define TRB_RESET_DEV		17
/* Force Event Command (opt) */
#define TRB_FORCE_EVENT		18
/* Negotiate Bandwidth Command (opt) */
#define TRB_NEG_BANDWIDTH	19
/* Set Latency Tolerance Value Command (opt) */
#define TRB_SET_LT		20
/* Get port bandwidth Command */
#define TRB_GET_BW		21
/* Force Header Command - generate a transaction or link management packet */
#define TRB_FORCE_HEADER	22
/* No-op Command - not for transfer rings */
#define TRB_CMD_NOOP		23
/* TRB IDs 24-31 reserved */
/* Event TRBS */
/* Transfer Event */
#define TRB_TRANSFER		32
/* Command Completion Event */
#define TRB_COMPLETION		33
/* Port Status Change Event */
#define TRB_PORT_STATUS		34
/* Bandwidth Request Event (opt) */
#define TRB_BANDWIDTH_EVENT	35
/* Doorbell Event (opt) */
#define TRB_DOORBELL		36
/* Host Controller Event */
#define TRB_HC_EVENT		37
/* Device Notification Event - device sent function wake notification */
#define TRB_DEV_NOTE		38
/* MFINDEX Wrap Event - microframe counter wrapped */
#define TRB_MFINDEX_WRAP	39
/* TRB IDs 40-47 reserved, 48-63 is vendor-defined */

/* Nec vendor-specific command completion event. */
#define	TRB_NEC_CMD_COMP	48
/* Get NEC firmware revision. */
#define	TRB_NEC_GET_FW		49

#define NEC_FW_MINOR(p)		(((p) >> 0) & 0xff)
#define NEC_FW_MAJOR(p)		(((p) >> 8) & 0xff)

#define TRBS_PER_SEGMENT	64
/* Allow two commands + a link TRB, along with any reserved command TRBs */
#define MAX_RSVD_CMD_TRBS	(TRBS_PER_SEGMENT - 3)
#define SEGMENT_SIZE		(TRBS_PER_SEGMENT*16)
#define SEGMENT_SHIFT		10
/* TRB buffer pointers can't cross 64KB boundaries */
#define TRB_MAX_BUFF_SHIFT		16
#define TRB_MAX_BUFF_SIZE	(1 << TRB_MAX_BUFF_SHIFT)

struct xhci_segment {
	union xhci_trb		*trbs;
	/* private to HCD */
	struct xhci_segment	*next;
	dma_addr_t		dma;
};

struct xhci_td {
	struct list_head	td_list;
	struct list_head	cancelled_td_list;
	struct urb		*urb;
	struct xhci_segment	*start_seg;
	union xhci_trb		*first_trb;
	union xhci_trb		*last_trb;
};

struct xhci_dequeue_state {
	struct xhci_segment *new_deq_seg;
	union xhci_trb *new_deq_ptr;
	int new_cycle_state;
};

struct xhci_ring {
	struct xhci_segment	*first_seg;
	union  xhci_trb		*enqueue;
	struct xhci_segment	*enq_seg;
	unsigned int		enq_updates;
	union  xhci_trb		*dequeue;
	struct xhci_segment	*deq_seg;
	unsigned int		deq_updates;
	struct list_head	td_list;
	/*
	 * Write the cycle state into the TRB cycle field to give ownership of
	 * the TRB to the host controller (if we are the producer), or to check
	 * if we own the TRB (if we are the consumer).  See section 4.9.1.
	 */
	u32			cycle_state;
	unsigned int		stream_id;
};

struct xhci_erst_entry {
	/* 64-bit event ring segment address */
	u64	seg_addr;
	u32	seg_size;
	/* Set to zero */
	u32	rsvd;
};

struct xhci_erst {
	struct xhci_erst_entry	*entries;
	unsigned int		num_entries;
	/* xhci->event_ring keeps track of segment dma addresses */
	dma_addr_t		erst_dma_addr;
	/* Num entries the ERST can contain */
	unsigned int		erst_size;
};

struct xhci_scratchpad {
	u64 *sp_array;
	dma_addr_t sp_dma;
	void **sp_buffers;
	dma_addr_t *sp_dma_buffers;
};

#define	ERST_NUM_SEGS	1
/* Initial allocated size of the ERST, in number of entries */
#define	ERST_SIZE	64
/* Initial number of event segment rings allocated */
#define	ERST_ENTRIES	1
/* Poll every 60 seconds */
#define	POLL_TIMEOUT	60
/* Stop endpoint command timeout (secs) for URB cancellation watchdog timer */
#define XHCI_STOP_EP_CMD_TIMEOUT	5
/* XXX: Make these module parameters */


/* There is one ehci_hci structure per controller */
struct xhci_hcd {
	/* glue to PCI and HCD framework */
	struct xhci_cap_regs __iomem *cap_regs;
	struct xhci_op_regs __iomem *op_regs;
	struct xhci_run_regs __iomem *run_regs;
	struct xhci_doorbell_array __iomem *dba;
	/* Our HCD's current interrupter register set */
	struct	xhci_intr_reg __iomem *ir_set;

	/* Cached register copies of read-only HC data */
	__u32		hcs_params1;
	__u32		hcs_params2;
	__u32		hcs_params3;
	__u32		hcc_params;

	spinlock_t	lock;

	/* packed release number */
	u8		sbrn;
	u16		hci_version;
	u8		max_slots;
	u8		max_interrupters;
	u8		max_ports;
	u8		isoc_threshold;
	int		event_ring_max;
	int		addr_64;
	/* 4KB min, 128MB max */
	int		page_size;
	/* Valid values are 12 to 20, inclusive */
	int		page_shift;
	/* only one MSI vector for now, but might need more later */
	int		msix_count;
	struct msix_entry	*msix_entries;
	/* data structures */
	struct xhci_device_context_array *dcbaa;
	struct xhci_ring	*cmd_ring;
	unsigned int		cmd_ring_reserved_trbs;
	struct xhci_ring	*event_ring;
	struct xhci_erst	erst;
	/* Scratchpad */
	struct xhci_scratchpad  *scratchpad;

	/* slot enabling and address device helpers */
	struct completion	addr_dev;
	int slot_id;
	/* Internal mirror of the HW's dcbaa */
	struct xhci_virt_device	*devs[MAX_HC_SLOTS];

	/* DMA pools */
	struct dma_pool	*device_pool;
	struct dma_pool	*segment_pool;
	struct dma_pool	*small_streams_pool;
	struct dma_pool	*medium_streams_pool;

#ifdef CONFIG_USB_XHCI_HCD_DEBUGGING
	/* Poll the rings - for debugging */
	struct timer_list	event_ring_timer;
	int			zombie;
#endif
	/* Host controller watchdog timer structures */
	unsigned int		xhc_state;
#define XHCI_STATE_DYING	(1 << 0)
	/* Statistics */
	int			noops_submitted;
	int			noops_handled;
	int			error_bitmask;
	unsigned int		quirks;
#define	XHCI_LINK_TRB_QUIRK	(1 << 0)
#define XHCI_RESET_EP_QUIRK	(1 << 1)
#define XHCI_NEC_HOST		(1 << 2)
};

/* For testing purposes */
#define NUM_TEST_NOOPS	0

/* convert between an HCD pointer and the corresponding EHCI_HCD */
static inline struct xhci_hcd *hcd_to_xhci(struct usb_hcd *hcd)
{
	return (struct xhci_hcd *) (hcd->hcd_priv);
}

static inline struct usb_hcd *xhci_to_hcd(struct xhci_hcd *xhci)
{
	return container_of((void *) xhci, struct usb_hcd, hcd_priv);
}

#ifdef CONFIG_USB_XHCI_HCD_DEBUGGING
#define XHCI_DEBUG	1
#else
#define XHCI_DEBUG	0
#endif

#define xhci_dbg(xhci, fmt, args...) \
	do { if (XHCI_DEBUG) dev_dbg(xhci_to_hcd(xhci)->self.controller , fmt , ## args); } while (0)
#define xhci_info(xhci, fmt, args...) \
	do { if (XHCI_DEBUG) dev_info(xhci_to_hcd(xhci)->self.controller , fmt , ## args); } while (0)
#define xhci_err(xhci, fmt, args...) \
	dev_err(xhci_to_hcd(xhci)->self.controller , fmt , ## args)
#define xhci_warn(xhci, fmt, args...) \
	dev_warn(xhci_to_hcd(xhci)->self.controller , fmt , ## args)

/* TODO: copied from ehci.h - can be refactored? */
/* xHCI spec says all registers are little endian */
static inline unsigned int xhci_readl(const struct xhci_hcd *xhci,
		__u32 __iomem *regs)
{
	return readl(regs);
}
static inline void xhci_writel(struct xhci_hcd *xhci,
		const unsigned int val, __u32 __iomem *regs)
{
	xhci_dbg(xhci,
			"`MEM_WRITE_DWORD(3'b000, 32'h%p, 32'h%0x, 4'hf);\n",
			regs, val);
	writel(val, regs);
}

static inline u64 xhci_read_64(const struct xhci_hcd *xhci,
		__u64 __iomem *regs)
{
	__u32 __iomem *ptr = (__u32 __iomem *) regs;
	u64 val_lo = readl(ptr);
	u64 val_hi = readl(ptr + 1);
	return val_lo + (val_hi << 32);
}
static inline void xhci_write_64(struct xhci_hcd *xhci,
		const u64 val, __u64 __iomem *regs)
{
	__u32 __iomem *ptr = (__u32 __iomem *) regs;
	u32 val_lo = lower_32_bits(val);
	u32 val_hi = upper_32_bits(val);

	xhci_dbg(xhci,
			"`MEM_WRITE_DWORD(3'b000, 64'h%p, 64'h%0lx, 4'hf);\n",
			regs, (long unsigned int) val);
	writel(val_lo, ptr);
	writel(val_hi, ptr + 1);
}

static inline int xhci_link_trb_quirk(struct xhci_hcd *xhci)
{
	u32 temp = xhci_readl(xhci, &xhci->cap_regs->hc_capbase);
	return ((HC_VERSION(temp) == 0x95) &&
			(xhci->quirks & XHCI_LINK_TRB_QUIRK));
}

/* xHCI debugging */
void xhci_print_ir_set(struct xhci_hcd *xhci, struct xhci_intr_reg *ir_set, int set_num);
void xhci_print_registers(struct xhci_hcd *xhci);
void xhci_dbg_regs(struct xhci_hcd *xhci);
void xhci_print_run_regs(struct xhci_hcd *xhci);
void xhci_print_trb_offsets(struct xhci_hcd *xhci, union xhci_trb *trb);
void xhci_debug_trb(struct xhci_hcd *xhci, union xhci_trb *trb);
void xhci_debug_segment(struct xhci_hcd *xhci, struct xhci_segment *seg);
void xhci_debug_ring(struct xhci_hcd *xhci, struct xhci_ring *ring);
void xhci_dbg_erst(struct xhci_hcd *xhci, struct xhci_erst *erst);
void xhci_dbg_cmd_ptrs(struct xhci_hcd *xhci);
void xhci_dbg_ring_ptrs(struct xhci_hcd *xhci, struct xhci_ring *ring);
void xhci_dbg_ctx(struct xhci_hcd *xhci, struct xhci_container_ctx *ctx, unsigned int last_ep);
char *xhci_get_slot_state(struct xhci_hcd *xhci,
		struct xhci_container_ctx *ctx);
void xhci_dbg_ep_rings(struct xhci_hcd *xhci,
		unsigned int slot_id, unsigned int ep_index,
		struct xhci_virt_ep *ep);

/* xHCI memory management */
void xhci_mem_cleanup(struct xhci_hcd *xhci);
int xhci_mem_init(struct xhci_hcd *xhci, gfp_t flags);
void xhci_free_virt_device(struct xhci_hcd *xhci, int slot_id);
int xhci_alloc_virt_device(struct xhci_hcd *xhci, int slot_id, struct usb_device *udev, gfp_t flags);
int xhci_setup_addressable_virt_dev(struct xhci_hcd *xhci, struct usb_device *udev);
void xhci_copy_ep0_dequeue_into_input_ctx(struct xhci_hcd *xhci,
		struct usb_device *udev);
unsigned int xhci_get_endpoint_index(struct usb_endpoint_descriptor *desc);
unsigned int xhci_get_endpoint_flag(struct usb_endpoint_descriptor *desc);
unsigned int xhci_get_endpoint_flag_from_index(unsigned int ep_index);
unsigned int xhci_last_valid_endpoint(u32 added_ctxs);
void xhci_endpoint_zero(struct xhci_hcd *xhci, struct xhci_virt_device *virt_dev, struct usb_host_endpoint *ep);
void xhci_endpoint_copy(struct xhci_hcd *xhci,
		struct xhci_container_ctx *in_ctx,
		struct xhci_container_ctx *out_ctx,
		unsigned int ep_index);
void xhci_slot_copy(struct xhci_hcd *xhci,
		struct xhci_container_ctx *in_ctx,
		struct xhci_container_ctx *out_ctx);
int xhci_endpoint_init(struct xhci_hcd *xhci, struct xhci_virt_device *virt_dev,
		struct usb_device *udev, struct usb_host_endpoint *ep,
		gfp_t mem_flags);
void xhci_ring_free(struct xhci_hcd *xhci, struct xhci_ring *ring);
void xhci_free_or_cache_endpoint_ring(struct xhci_hcd *xhci,
		struct xhci_virt_device *virt_dev,
		unsigned int ep_index);
struct xhci_stream_info *xhci_alloc_stream_info(struct xhci_hcd *xhci,
		unsigned int num_stream_ctxs,
		unsigned int num_streams, gfp_t flags);
void xhci_free_stream_info(struct xhci_hcd *xhci,
		struct xhci_stream_info *stream_info);
void xhci_setup_streams_ep_input_ctx(struct xhci_hcd *xhci,
		struct xhci_ep_ctx *ep_ctx,
		struct xhci_stream_info *stream_info);
void xhci_setup_no_streams_ep_input_ctx(struct xhci_hcd *xhci,
		struct xhci_ep_ctx *ep_ctx,
		struct xhci_virt_ep *ep);
struct xhci_ring *xhci_dma_to_transfer_ring(
		struct xhci_virt_ep *ep,
		u64 address);
struct xhci_ring *xhci_urb_to_transfer_ring(struct xhci_hcd *xhci,
		struct urb *urb);
struct xhci_ring *xhci_triad_to_transfer_ring(struct xhci_hcd *xhci,
		unsigned int slot_id, unsigned int ep_index,
		unsigned int stream_id);
struct xhci_ring *xhci_stream_id_to_ring(
		struct xhci_virt_device *dev,
		unsigned int ep_index,
		unsigned int stream_id);
struct xhci_command *xhci_alloc_command(struct xhci_hcd *xhci,
		bool allocate_in_ctx, bool allocate_completion,
		gfp_t mem_flags);
void xhci_free_command(struct xhci_hcd *xhci,
		struct xhci_command *command);

#ifdef CONFIG_PCI
/* xHCI PCI glue */
int xhci_register_pci(void);
void xhci_unregister_pci(void);
#endif

/* xHCI host controller glue */
void xhci_quiesce(struct xhci_hcd *xhci);
int xhci_halt(struct xhci_hcd *xhci);
int xhci_reset(struct xhci_hcd *xhci);
int xhci_init(struct usb_hcd *hcd);
int xhci_run(struct usb_hcd *hcd);
void xhci_stop(struct usb_hcd *hcd);
void xhci_shutdown(struct usb_hcd *hcd);
int xhci_get_frame(struct usb_hcd *hcd);
irqreturn_t xhci_irq(struct usb_hcd *hcd);
int xhci_alloc_dev(struct usb_hcd *hcd, struct usb_device *udev);
void xhci_free_dev(struct usb_hcd *hcd, struct usb_device *udev);
int xhci_alloc_streams(struct usb_hcd *hcd, struct usb_device *udev,
		struct usb_host_endpoint **eps, unsigned int num_eps,
		unsigned int num_streams, gfp_t mem_flags);
int xhci_free_streams(struct usb_hcd *hcd, struct usb_device *udev,
		struct usb_host_endpoint **eps, unsigned int num_eps,
		gfp_t mem_flags);
int xhci_address_device(struct usb_hcd *hcd, struct usb_device *udev);
int xhci_update_hub_device(struct usb_hcd *hcd, struct usb_device *hdev,
			struct usb_tt *tt, gfp_t mem_flags);
int xhci_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags);
int xhci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status);
int xhci_add_endpoint(struct usb_hcd *hcd, struct usb_device *udev, struct usb_host_endpoint *ep);
int xhci_drop_endpoint(struct usb_hcd *hcd, struct usb_device *udev, struct usb_host_endpoint *ep);
void xhci_endpoint_reset(struct usb_hcd *hcd, struct usb_host_endpoint *ep);
int xhci_reset_device(struct usb_hcd *hcd, struct usb_device *udev);
int xhci_check_bandwidth(struct usb_hcd *hcd, struct usb_device *udev);
void xhci_reset_bandwidth(struct usb_hcd *hcd, struct usb_device *udev);

/* xHCI ring, segment, TRB, and TD functions */
dma_addr_t xhci_trb_virt_to_dma(struct xhci_segment *seg, union xhci_trb *trb);
struct xhci_segment *trb_in_td(struct xhci_segment *start_seg,
		union xhci_trb *start_trb, union xhci_trb *end_trb,
		dma_addr_t suspect_dma);
int xhci_is_vendor_info_code(struct xhci_hcd *xhci, unsigned int trb_comp_code);
void xhci_ring_cmd_db(struct xhci_hcd *xhci);
void *xhci_setup_one_noop(struct xhci_hcd *xhci);
void xhci_handle_event(struct xhci_hcd *xhci);
void xhci_set_hc_event_deq(struct xhci_hcd *xhci);
int xhci_queue_slot_control(struct xhci_hcd *xhci, u32 trb_type, u32 slot_id);
int xhci_queue_address_device(struct xhci_hcd *xhci, dma_addr_t in_ctx_ptr,
		u32 slot_id);
int xhci_queue_vendor_command(struct xhci_hcd *xhci,
		u32 field1, u32 field2, u32 field3, u32 field4);
int xhci_queue_stop_endpoint(struct xhci_hcd *xhci, int slot_id,
		unsigned int ep_index);
int xhci_queue_ctrl_tx(struct xhci_hcd *xhci, gfp_t mem_flags, struct urb *urb,
		int slot_id, unsigned int ep_index);
int xhci_queue_bulk_tx(struct xhci_hcd *xhci, gfp_t mem_flags, struct urb *urb,
		int slot_id, unsigned int ep_index);
int xhci_queue_intr_tx(struct xhci_hcd *xhci, gfp_t mem_flags, struct urb *urb,
		int slot_id, unsigned int ep_index);
int xhci_queue_configure_endpoint(struct xhci_hcd *xhci, dma_addr_t in_ctx_ptr,
		u32 slot_id, bool command_must_succeed);
int xhci_queue_evaluate_context(struct xhci_hcd *xhci, dma_addr_t in_ctx_ptr,
		u32 slot_id);
int xhci_queue_reset_ep(struct xhci_hcd *xhci, int slot_id,
		unsigned int ep_index);
int xhci_queue_reset_device(struct xhci_hcd *xhci, u32 slot_id);
void xhci_find_new_dequeue_state(struct xhci_hcd *xhci,
		unsigned int slot_id, unsigned int ep_index,
		unsigned int stream_id, struct xhci_td *cur_td,
		struct xhci_dequeue_state *state);
void xhci_queue_new_dequeue_state(struct xhci_hcd *xhci,
		unsigned int slot_id, unsigned int ep_index,
		unsigned int stream_id,
		struct xhci_dequeue_state *deq_state);
void xhci_cleanup_stalled_ring(struct xhci_hcd *xhci,
		struct usb_device *udev, unsigned int ep_index);
void xhci_queue_config_ep_quirk(struct xhci_hcd *xhci,
		unsigned int slot_id, unsigned int ep_index,
		struct xhci_dequeue_state *deq_state);
void xhci_stop_endpoint_command_watchdog(unsigned long arg);

/* xHCI roothub code */
int xhci_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue, u16 wIndex,
		char *buf, u16 wLength);
int xhci_hub_status_data(struct usb_hcd *hcd, char *buf);

/* xHCI contexts */
struct xhci_input_control_ctx *xhci_get_input_control_ctx(struct xhci_hcd *xhci, struct xhci_container_ctx *ctx);
struct xhci_slot_ctx *xhci_get_slot_ctx(struct xhci_hcd *xhci, struct xhci_container_ctx *ctx);
struct xhci_ep_ctx *xhci_get_ep_ctx(struct xhci_hcd *xhci, struct xhci_container_ctx *ctx, unsigned int ep_index);

#endif /* __LINUX_XHCI_HCD_H */
