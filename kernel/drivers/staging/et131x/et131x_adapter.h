

#ifndef __ET131X_ADAPTER_H__
#define __ET131X_ADAPTER_H__

#include "et1310_address_map.h"
#include "et1310_tx.h"
#include "et1310_rx.h"

#define NUM_DESC_PER_RING_TX         512	/* TX Do not change these values */
#define NUM_TCB                      64

#define NUM_TRAFFIC_CLASSES          1

#define TX_ERROR_PERIOD             1000

#define LO_MARK_PERCENT_FOR_PSR     15
#define LO_MARK_PERCENT_FOR_RX      15

/* RFD (Receive Frame Descriptor) */
typedef struct _MP_RFD {
	struct list_head list_node;
	struct sk_buff *Packet;
	u32 PacketSize;	/* total size of receive frame */
	u16 bufferindex;
	u8 ringindex;
} MP_RFD, *PMP_RFD;

/* Enum for Flow Control */
typedef enum _eflow_control_t {
	Both = 0,
	TxOnly = 1,
	RxOnly = 2,
	None = 3
} eFLOW_CONTROL_t, *PeFLOW_CONTROL_t;

/* Struct to define some device statistics */
typedef struct _ce_stats_t {
	/* Link Input/Output stats */
	uint64_t ipackets;	/* # of in packets */
	uint64_t opackets;	/* # of out packets */

	/* MIB II variables
	 *
	 * NOTE: atomic_t types are only guaranteed to store 24-bits; if we
	 * MUST have 32, then we'll need another way to perform atomic
	 * operations
	 */
	u32 unircv;	/* # multicast packets received */
	atomic_t unixmt;	/* # multicast packets for Tx */
	u32 multircv;	/* # multicast packets received */
	atomic_t multixmt;	/* # multicast packets for Tx */
	u32 brdcstrcv;	/* # broadcast packets received */
	atomic_t brdcstxmt;	/* # broadcast packets for Tx */
	u32 norcvbuf;	/* # Rx packets discarded */
	u32 noxmtbuf;	/* # Tx packets discarded */

	/* Transciever state informations. */
	u8 xcvr_addr;
	u32 xcvr_id;

	/* Tx Statistics. */
	u32 tx_uflo;		/* Tx Underruns */

	u32 collisions;
	u32 excessive_collisions;
	u32 first_collision;
	u32 late_collisions;
	u32 max_pkt_error;
	u32 tx_deferred;

	/* Rx Statistics. */
	u32 rx_ov_flow;	/* Rx Overflow */

	u32 length_err;
	u32 alignment_err;
	u32 crc_err;
	u32 code_violations;
	u32 other_errors;

	u32 SynchrounousIterations;
	u32 InterruptStatus;
} CE_STATS_t, *PCE_STATS_t;

typedef struct _MP_POWER_MGMT {
	/* variable putting the phy into coma mode when boot up with no cable
	 * plugged in after 5 seconds
	 */
	u8 TransPhyComaModeOnBoot;

	/* Next two used to save power information at power down. This
	 * information will be used during power up to set up parts of Power
	 * Management in JAGCore
	 */
	u16 PowerDownSpeed;
	u8 PowerDownDuplex;
} MP_POWER_MGMT, *PMP_POWER_MGMT;

/* The private adapter structure */
struct et131x_adapter {
	struct net_device *netdev;
	struct pci_dev *pdev;

	struct work_struct task;

	/* Flags that indicate current state of the adapter */
	u32 Flags;
	u32 HwErrCount;

	/* Configuration  */
	u8 PermanentAddress[ETH_ALEN];
	u8 CurrentAddress[ETH_ALEN];
	bool has_eeprom;
	u8 eepromData[2];

	/* Spinlocks */
	spinlock_t Lock;

	spinlock_t TCBSendQLock;
	spinlock_t TCBReadyQLock;
	spinlock_t SendHWLock;

	spinlock_t RcvLock;
	spinlock_t RcvPendLock;
	spinlock_t FbrLock;

	spinlock_t PHYLock;

	/* Packet Filter and look ahead size */
	u32 PacketFilter;
	u32 linkspeed;
	u32 duplex_mode;

	/* multicast list */
	u32 MCAddressCount;
	u8 MCList[NIC_MAX_MCAST_LIST][ETH_ALEN];

	/* Pointer to the device's PCI register space */
	ADDRESS_MAP_t __iomem *regs;

	/* Registry parameters */
	u8 SpeedDuplex;		/* speed/duplex */
	eFLOW_CONTROL_t RegistryFlowControl;	/* for 802.3x flow control */
	u8 RegistryPhyComa;	/* Phy Coma mode enable/disable */

	u32 RegistryRxMemEnd;	/* Size of internal rx memory */
	u32 RegistryJumboPacket;	/* Max supported ethernet packet size */


	/* Derived from the registry: */
	u8 AiForceDpx;		/* duplex setting */
	u16 AiForceSpeed;		/* 'Speed', user over-ride of line speed */
	eFLOW_CONTROL_t FlowControl;	/* flow control validated by the far-end */
	enum {
		NETIF_STATUS_INVALID = 0,
		NETIF_STATUS_MEDIA_CONNECT,
		NETIF_STATUS_MEDIA_DISCONNECT,
		NETIF_STATUS_MAX
	} MediaState;

	/* Minimize init-time */
	struct timer_list ErrorTimer;
	MP_POWER_MGMT PoMgmt;
	u32 CachedMaskValue;

	/* Xcvr status at last poll */
	MI_BMSR_t Bmsr;

	/* Tx Memory Variables */
	struct tx_ring tx_ring;

	/* Rx Memory Variables */
	struct rx_ring rx_ring;

	/* Loopback specifics */
	u8 ReplicaPhyLoopbk;	/* Replica Enable */
	u8 ReplicaPhyLoopbkPF;	/* Replica Enable Pass/Fail */

	/* Stats */
	CE_STATS_t Stats;

	struct net_device_stats net_stats;
	struct net_device_stats net_stats_prev;
};

#endif /* __ET131X_ADAPTER_H__ */
