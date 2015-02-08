

#ifndef __ET1310_TX_H__
#define __ET1310_TX_H__


/* Typedefs for Tx Descriptor Ring */


/* struct tx_desc represents each descriptor on the ring */
struct tx_desc {
	u32 addr_hi;
	u32 addr_lo;
	u32 len_vlan;	/* control words how to xmit the */
	u32 flags;	/* data (detailed above) */
};


/* TCB (Transmit Control Block: Host Side) */
struct tcb {
	struct tcb *next;	/* Next entry in ring */
	u32 flags;		/* Our flags for the packet */
	u32 count;		/* Used to spot stuck/lost packets */
	u32 stale;		/* Used to spot stuck/lost packets */
	struct sk_buff *skb;	/* Network skb we are tied to */
	u32 index;		/* Ring indexes */
	u32 index_start;
};

/* Structure representing our local reference(s) to the ring */
struct tx_ring {
	/* TCB (Transmit Control Block) memory and lists */
	struct tcb *tcb_ring;

	/* List of TCBs that are ready to be used */
	struct tcb *tcb_qhead;
	struct tcb *tcb_qtail;

	/* list of TCBs that are currently being sent.  NOTE that access to all
	 * three of these (including used) are controlled via the
	 * TCBSendQLock.  This lock should be secured prior to incementing /
	 * decrementing used, or any queue manipulation on send_head /
	 * tail
	 */
	struct tcb *send_head;
	struct tcb *send_tail;
	int used;

	/* The actual descriptor ring */
	struct tx_desc *tx_desc_ring;
	dma_addr_t tx_desc_ring_pa;

	/* send_idx indicates where we last wrote to in the descriptor ring. */
	u32 send_idx;

	/* The location of the write-back status block */
	u32 *tx_status;
	dma_addr_t tx_status_pa;

	/* Packets since the last IRQ: used for interrupt coalescing */
	int since_irq;
};

#endif /* __ET1310_TX_H__ */
