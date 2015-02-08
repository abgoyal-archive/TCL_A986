

#ifndef __ET1310_RX_H__
#define __ET1310_RX_H__

#include "et1310_address_map.h"

#define USE_FBR0 true

#ifdef USE_FBR0
/* #define FBR0_BUFFER_SIZE 256 */
#endif

/* #define FBR1_BUFFER_SIZE 2048 */

#define FBR_CHUNKS 32

#define MAX_DESC_PER_RING_RX         1024

/* number of RFDs - default and min */
#ifdef USE_FBR0
#define RFD_LOW_WATER_MARK	40
#define NIC_MIN_NUM_RFD		64
#define NIC_DEFAULT_NUM_RFD	1024
#else
#define RFD_LOW_WATER_MARK	20
#define NIC_MIN_NUM_RFD		64
#define NIC_DEFAULT_NUM_RFD	256
#endif

#define NUM_PACKETS_HANDLED	256

#define ALCATEL_BAD_STATUS	0xe47f0000
#define ALCATEL_MULTICAST_PKT	0x01000000
#define ALCATEL_BROADCAST_PKT	0x02000000

/* typedefs for Free Buffer Descriptors */
struct fbr_desc {
	u32 addr_lo;
	u32 addr_hi;
	u32 word2;		/* Bits 10-31 reserved, 0-9 descriptor */
};


struct pkt_stat_desc {
	u32 word0;
	u32 word1;
};

/* Typedefs for the RX DMA status word */



struct rx_status_block {
	u32 Word0;
	u32 Word1;
};

struct fbr_lookup {
	void *virt[MAX_DESC_PER_RING_RX];
	void *buffer1[MAX_DESC_PER_RING_RX];
	void *buffer2[MAX_DESC_PER_RING_RX];
	u32 bus_high[MAX_DESC_PER_RING_RX];
	u32 bus_low[MAX_DESC_PER_RING_RX];
};

struct rx_ring {
#ifdef USE_FBR0
	void *pFbr0RingVa;
	dma_addr_t pFbr0RingPa;
	void *Fbr0MemVa[MAX_DESC_PER_RING_RX / FBR_CHUNKS];
	dma_addr_t Fbr0MemPa[MAX_DESC_PER_RING_RX / FBR_CHUNKS];
	uint64_t Fbr0Realpa;
	uint64_t Fbr0offset;
	u32 local_Fbr0_full;
	u32 Fbr0NumEntries;
	u32 Fbr0BufferSize;
#endif
	void *pFbr1RingVa;
	dma_addr_t pFbr1RingPa;
	void *Fbr1MemVa[MAX_DESC_PER_RING_RX / FBR_CHUNKS];
	dma_addr_t Fbr1MemPa[MAX_DESC_PER_RING_RX / FBR_CHUNKS];
	uint64_t Fbr1Realpa;
	uint64_t Fbr1offset;
	struct fbr_lookup *fbr[2];	/* One per ring */
	u32 local_Fbr1_full;
	u32 Fbr1NumEntries;
	u32 Fbr1BufferSize;

	void *pPSRingVa;
	dma_addr_t pPSRingPa;
	u32 local_psr_full;
	u32 PsrNumEntries;

	struct rx_status_block *rx_status_block;
	dma_addr_t rx_status_bus;

	struct list_head RecvBufferPool;

	/* RECV */
	struct list_head RecvList;
	u32 nReadyRecv;

	u32 NumRfd;

	bool UnfinishedReceives;

	struct list_head RecvPacketPool;

	/* lookaside lists */
	struct kmem_cache *RecvLookaside;
};

#endif /* __ET1310_RX_H__ */
