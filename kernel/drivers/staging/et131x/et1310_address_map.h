

#ifndef _ET1310_ADDRESS_MAP_H_
#define _ET1310_ADDRESS_MAP_H_


/* START OF GLOBAL REGISTER ADDRESS MAP */



#define ET_PM_PHY_SW_COMA		0x40
#define ET_PMCSR_INIT			0x38


#define	ET_INTR_TXDMA_ISR	0x00000008
#define ET_INTR_TXDMA_ERR	0x00000010
#define ET_INTR_RXDMA_XFR_DONE	0x00000020
#define ET_INTR_RXDMA_FB_R0_LOW	0x00000040
#define ET_INTR_RXDMA_FB_R1_LOW	0x00000080
#define ET_INTR_RXDMA_STAT_LOW	0x00000100
#define ET_INTR_RXDMA_ERR	0x00000200
#define ET_INTR_WATCHDOG	0x00004000
#define ET_INTR_WOL		0x00008000
#define ET_INTR_PHY		0x00010000
#define ET_INTR_TXMAC		0x00020000
#define ET_INTR_RXMAC		0x00040000
#define ET_INTR_MAC_STAT	0x00080000
#define ET_INTR_SLV_TIMEOUT	0x00100000





#define ET_MSI_VECTOR	0x0000001F
#define ET_MSI_TC	0x00070000


#define ET_LOOP_MAC	0x00000001
#define ET_LOOP_DMA	0x00000002

struct global_regs {			/* Location: */
	u32 txq_start_addr;			/*  0x0000 */
	u32 txq_end_addr;			/*  0x0004 */
	u32 rxq_start_addr;			/*  0x0008 */
	u32 rxq_end_addr;			/*  0x000C */
	u32 pm_csr;				/*  0x0010 */
	u32 unused;				/*  0x0014 */
	u32 int_status;				/*  0x0018 */
	u32 int_mask;				/*  0x001C */
	u32 int_alias_clr_en;			/*  0x0020 */
	u32 int_status_alias;			/*  0x0024 */
	u32 sw_reset;				/*  0x0028 */
	u32 slv_timer;				/*  0x002C */
	u32 msi_config;				/*  0x0030 */
	u32 loopback;			/*  0x0034 */
	u32 watchdog_timer;			/*  0x0038 */
};


/* START OF TXDMA REGISTER ADDRESS MAP */


#define ET_TXDMA_CSR_HALT	0x00000001
#define ET_TXDMA_DROP_TLP	0x00000002
#define ET_TXDMA_CACHE_THRS	0x000000F0
#define ET_TXDMA_CACHE_SHIFT	4
#define ET_TXDMA_SNGL_EPKT	0x00000100
#define ET_TXDMA_CLASS		0x00001E00




#define ET_DMA12_MASK		0x0FFF	/* 12 bit mask for DMA12W types */
#define ET_DMA12_WRAP		0x1000
#define ET_DMA10_MASK		0x03FF	/* 10 bit mask for DMA10W types */
#define ET_DMA10_WRAP		0x0400
#define ET_DMA4_MASK		0x000F	/* 4 bit mask for DMA4W types */
#define ET_DMA4_WRAP		0x0010

#define INDEX12(x)	((x) & ET_DMA12_MASK)
#define INDEX10(x)	((x) & ET_DMA10_MASK)
#define INDEX4(x)	((x) & ET_DMA4_MASK)

extern inline void add_10bit(u32 *v, int n)
{
	*v = INDEX10(*v + n) | (*v & ET_DMA10_WRAP);
}

extern inline void add_12bit(u32 *v, int n)
{
	*v = INDEX12(*v + n) | (*v & ET_DMA12_WRAP);
}


struct txdma_regs {			/* Location: */
	u32 csr;			/*  0x1000 */
	u32 pr_base_hi;			/*  0x1004 */
	u32 pr_base_lo;			/*  0x1008 */
	u32 pr_num_des;			/*  0x100C */
	u32 txq_wr_addr;		/*  0x1010 */
	u32 txq_wr_addr_ext;		/*  0x1014 */
	u32 txq_rd_addr;		/*  0x1018 */
	u32 dma_wb_base_hi;		/*  0x101C */
	u32 dma_wb_base_lo;		/*  0x1020 */
	u32 service_request;		/*  0x1024 */
	u32 service_complete;		/*  0x1028 */
	u32 cache_rd_index;		/*  0x102C */
	u32 cache_wr_index;		/*  0x1030 */
	u32 TxDmaError;			/*  0x1034 */
	u32 DescAbortCount;		/*  0x1038 */
	u32 PayloadAbortCnt;		/*  0x103c */
	u32 WriteBackAbortCnt;		/*  0x1040 */
	u32 DescTimeoutCnt;		/*  0x1044 */
	u32 PayloadTimeoutCnt;		/*  0x1048 */
	u32 WriteBackTimeoutCnt;	/*  0x104c */
	u32 DescErrorCount;		/*  0x1050 */
	u32 PayloadErrorCnt;		/*  0x1054 */
	u32 WriteBackErrorCnt;		/*  0x1058 */
	u32 DroppedTLPCount;		/*  0x105c */
	u32 NewServiceComplete;		/*  0x1060 */
	u32 EthernetPacketCount;	/*  0x1064 */
};

/* END OF TXDMA REGISTER ADDRESS MAP */


/* START OF RXDMA REGISTER ADDRESS MAP */






























struct rxdma_regs {					/* Location: */
	u32 csr;					/*  0x2000 */
	u32 dma_wb_base_lo;				/*  0x2004 */
	u32 dma_wb_base_hi;				/*  0x2008 */
	u32 num_pkt_done;				/*  0x200C */
	u32 max_pkt_time;				/*  0x2010 */
	u32 rxq_rd_addr;				/*  0x2014 */
	u32 rxq_rd_addr_ext;				/*  0x2018 */
	u32 rxq_wr_addr;				/*  0x201C */
	u32 psr_base_lo;				/*  0x2020 */
	u32 psr_base_hi;				/*  0x2024 */
	u32 psr_num_des;				/*  0x2028 */
	u32 psr_avail_offset;				/*  0x202C */
	u32 psr_full_offset;				/*  0x2030 */
	u32 psr_access_index;				/*  0x2034 */
	u32 psr_min_des;				/*  0x2038 */
	u32 fbr0_base_lo;				/*  0x203C */
	u32 fbr0_base_hi;				/*  0x2040 */
	u32 fbr0_num_des;				/*  0x2044 */
	u32 fbr0_avail_offset;				/*  0x2048 */
	u32 fbr0_full_offset;				/*  0x204C */
	u32 fbr0_rd_index;				/*  0x2050 */
	u32 fbr0_min_des;				/*  0x2054 */
	u32 fbr1_base_lo;				/*  0x2058 */
	u32 fbr1_base_hi;				/*  0x205C */
	u32 fbr1_num_des;				/*  0x2060 */
	u32 fbr1_avail_offset;				/*  0x2064 */
	u32 fbr1_full_offset;				/*  0x2068 */
	u32 fbr1_rd_index;				/*  0x206C */
	u32 fbr1_min_des;				/*  0x2070 */
};

/* END OF RXDMA REGISTER ADDRESS MAP */


/* START OF TXMAC REGISTER ADDRESS MAP */










struct txmac_regs {			/* Location: */
	u32 ctl;			/*  0x3000 */
	u32 shadow_ptr;			/*  0x3004 */
	u32 err_cnt;			/*  0x3008 */
	u32 max_fill;			/*  0x300C */
	u32 cf_param;			/*  0x3010 */
	u32 tx_test;			/*  0x3014 */
	u32 err;			/*  0x3018 */
	u32 err_int;			/*  0x301C */
	u32 bp_ctrl;			/*  0x3020 */
};

/* END OF TXMAC REGISTER ADDRESS MAP */

/* START OF RXMAC REGISTER ADDRESS MAP */





typedef union _RXMAC_WOL_SA_LO_t {
	u32 value;
	struct {
#ifdef _BIT_FIELDS_HTOL
		u32 sa3:8;	/* bits 24-31 */
		u32 sa4:8;	/* bits 16-23 */
		u32 sa5:8;	/* bits 8-15 */
		u32 sa6:8;	/* bits 0-7 */
#else
		u32 sa6:8;	/* bits 0-7 */
		u32 sa5:8;	/* bits 8-15 */
		u32 sa4:8;	/* bits 16-23 */
		u32 sa3:8;	/* bits 24-31 */
#endif
	} bits;
} RXMAC_WOL_SA_LO_t, *PRXMAC_WOL_SA_LO_t;

typedef union _RXMAC_WOL_SA_HI_t {
	u32 value;
	struct {
#ifdef _BIT_FIELDS_HTOL
		u32 reserved:16;	/* bits 16-31 */
		u32 sa1:8;		/* bits 8-15 */
		u32 sa2:8;		/* bits 0-7 */
#else
		u32 sa2:8;		/* bits 0-7 */
		u32 sa1:8;		/* bits 8-15 */
		u32 reserved:16;	/* bits 16-31 */
#endif
	} bits;
} RXMAC_WOL_SA_HI_t, *PRXMAC_WOL_SA_HI_t;


typedef union _RXMAC_UNI_PF_ADDR1_t {
	u32 value;
	struct {
#ifdef _BIT_FIELDS_HTOL
		u32 addr1_3:8;	/* bits 24-31 */
		u32 addr1_4:8;	/* bits 16-23 */
		u32 addr1_5:8;	/* bits 8-15 */
		u32 addr1_6:8;	/* bits 0-7 */
#else
		u32 addr1_6:8;	/* bits 0-7 */
		u32 addr1_5:8;	/* bits 8-15 */
		u32 addr1_4:8;	/* bits 16-23 */
		u32 addr1_3:8;	/* bits 24-31 */
#endif
	} bits;
} RXMAC_UNI_PF_ADDR1_t, *PRXMAC_UNI_PF_ADDR1_t;

typedef union _RXMAC_UNI_PF_ADDR2_t {
	u32 value;
	struct {
#ifdef _BIT_FIELDS_HTOL
		u32 addr2_3:8;	/* bits 24-31 */
		u32 addr2_4:8;	/* bits 16-23 */
		u32 addr2_5:8;	/* bits 8-15 */
		u32 addr2_6:8;	/* bits 0-7 */
#else
		u32 addr2_6:8;	/* bits 0-7 */
		u32 addr2_5:8;	/* bits 8-15 */
		u32 addr2_4:8;	/* bits 16-23 */
		u32 addr2_3:8;	/* bits 24-31 */
#endif
	} bits;
} RXMAC_UNI_PF_ADDR2_t, *PRXMAC_UNI_PF_ADDR2_t;

typedef union _RXMAC_UNI_PF_ADDR3_t {
	u32 value;
	struct {
#ifdef _BIT_FIELDS_HTOL
		u32 addr2_1:8;	/* bits 24-31 */
		u32 addr2_2:8;	/* bits 16-23 */
		u32 addr1_1:8;	/* bits 8-15 */
		u32 addr1_2:8;	/* bits 0-7 */
#else
		u32 addr1_2:8;	/* bits 0-7 */
		u32 addr1_1:8;	/* bits 8-15 */
		u32 addr2_2:8;	/* bits 16-23 */
		u32 addr2_1:8;	/* bits 24-31 */
#endif
	} bits;
} RXMAC_UNI_PF_ADDR3_t, *PRXMAC_UNI_PF_ADDR3_t;









typedef struct _RXMAC_t {				/* Location: */
	u32 ctrl;					/*  0x4000 */
	u32 crc0;					/*  0x4004 */
	u32 crc12;					/*  0x4008 */
	u32 crc34;					/*  0x400C */
	RXMAC_WOL_SA_LO_t sa_lo;			/*  0x4010 */
	RXMAC_WOL_SA_HI_t sa_hi;			/*  0x4014 */
	u32 mask0_word0;				/*  0x4018 */
	u32 mask0_word1;				/*  0x401C */
	u32 mask0_word2;				/*  0x4020 */
	u32 mask0_word3;				/*  0x4024 */
	u32 mask1_word0;				/*  0x4028 */
	u32 mask1_word1;				/*  0x402C */
	u32 mask1_word2;				/*  0x4030 */
	u32 mask1_word3;				/*  0x4034 */
	u32 mask2_word0;				/*  0x4038 */
	u32 mask2_word1;				/*  0x403C */
	u32 mask2_word2;				/*  0x4040 */
	u32 mask2_word3;				/*  0x4044 */
	u32 mask3_word0;				/*  0x4048 */
	u32 mask3_word1;				/*  0x404C */
	u32 mask3_word2;				/*  0x4050 */
	u32 mask3_word3;				/*  0x4054 */
	u32 mask4_word0;				/*  0x4058 */
	u32 mask4_word1;				/*  0x405C */
	u32 mask4_word2;				/*  0x4060 */
	u32 mask4_word3;				/*  0x4064 */
	RXMAC_UNI_PF_ADDR1_t uni_pf_addr1;		/*  0x4068 */
	RXMAC_UNI_PF_ADDR2_t uni_pf_addr2;		/*  0x406C */
	RXMAC_UNI_PF_ADDR3_t uni_pf_addr3;		/*  0x4070 */
	u32 multi_hash1;				/*  0x4074 */
	u32 multi_hash2;				/*  0x4078 */
	u32 multi_hash3;				/*  0x407C */
	u32 multi_hash4;				/*  0x4080 */
	u32 pf_ctrl;					/*  0x4084 */
	u32 mcif_ctrl_max_seg;				/*  0x4088 */
	u32 mcif_water_mark;				/*  0x408C */
	u32 rxq_diag;					/*  0x4090 */
	u32 space_avail;				/*  0x4094 */

	u32 mif_ctrl;					/*  0x4098 */
	u32 err_reg;					/*  0x409C */
} RXMAC_t, *PRXMAC_t;

/* END OF RXMAC REGISTER ADDRESS MAP */


/* START OF MAC REGISTER ADDRESS MAP */


#define CFG1_LOOPBACK	0x00000100
#define CFG1_RX_FLOW	0x00000020
#define CFG1_TX_FLOW	0x00000010
#define CFG1_RX_ENABLE	0x00000004
#define CFG1_TX_ENABLE	0x00000001
#define CFG1_WAIT	0x0000000A	/* RX & TX syncd */










#define MII_ADDR(phy, reg)	((phy) << 8 | (reg))




#define MGMT_BUSY	0x00000001	/* busy */
#define MGMT_WAIT	0x00000005	/* busy | not valid */



typedef union _MAC_STATION_ADDR1_t {
	u32 value;
	struct {
#ifdef _BIT_FIELDS_HTOL
		u32 Octet6:8;	/* bits 24-31 */
		u32 Octet5:8;	/* bits 16-23 */
		u32 Octet4:8;	/* bits 8-15 */
		u32 Octet3:8;	/* bits 0-7 */
#else
		u32 Octet3:8;	/* bits 0-7 */
		u32 Octet4:8;	/* bits 8-15 */
		u32 Octet5:8;	/* bits 16-23 */
		u32 Octet6:8;	/* bits 24-31 */
#endif
	} bits;
} MAC_STATION_ADDR1_t, *PMAC_STATION_ADDR1_t;

typedef union _MAC_STATION_ADDR2_t {
	u32 value;
	struct {
#ifdef _BIT_FIELDS_HTOL
		u32 Octet2:8;	/* bits 24-31 */
		u32 Octet1:8;	/* bits 16-23 */
		u32 reserved:16;	/* bits 0-15 */
#else
		u32 reserved:16;	/* bit 0-15 */
		u32 Octet1:8;	/* bits 16-23 */
		u32 Octet2:8;	/* bits 24-31 */
#endif
	} bits;
} MAC_STATION_ADDR2_t, *PMAC_STATION_ADDR2_t;

typedef struct _MAC_t {					/* Location: */
	u32 cfg1;					/*  0x5000 */
	u32 cfg2;					/*  0x5004 */
	u32 ipg;					/*  0x5008 */
	u32 hfdp;					/*  0x500C */
	u32 max_fm_len;					/*  0x5010 */
	u32 rsv1;					/*  0x5014 */
	u32 rsv2;					/*  0x5018 */
	u32 mac_test;					/*  0x501C */
	u32 mii_mgmt_cfg;				/*  0x5020 */
	u32 mii_mgmt_cmd;				/*  0x5024 */
	u32 mii_mgmt_addr;				/*  0x5028 */
	u32 mii_mgmt_ctrl;				/*  0x502C */
	u32 mii_mgmt_stat;				/*  0x5030 */
	u32 mii_mgmt_indicator;				/*  0x5034 */
	u32 if_ctrl;					/*  0x5038 */
	u32 if_stat;					/*  0x503C */
	MAC_STATION_ADDR1_t station_addr_1;		/*  0x5040 */
	MAC_STATION_ADDR2_t station_addr_2;		/*  0x5044 */
} MAC_t, *PMAC_t;

/* END OF MAC REGISTER ADDRESS MAP */

/* START OF MAC STAT REGISTER ADDRESS MAP */



struct macstat_regs {			/* Location: */
	u32 pad[32];			/*  0x6000 - 607C */

	/* Tx/Rx 0-64 Byte Frame Counter */
	u32 TR64;			/*  0x6080 */

	/* Tx/Rx 65-127 Byte Frame Counter */
	u32 TR127;			/*  0x6084 */

	/* Tx/Rx 128-255 Byte Frame Counter */
	u32 TR255;			/*  0x6088 */

	/* Tx/Rx 256-511 Byte Frame Counter */
	u32 TR511;			/*  0x608C */

	/* Tx/Rx 512-1023 Byte Frame Counter */
	u32 TR1K;			/*  0x6090 */

	/* Tx/Rx 1024-1518 Byte Frame Counter */
	u32 TRMax;			/*  0x6094 */

	/* Tx/Rx 1519-1522 Byte Good VLAN Frame Count */
	u32 TRMgv;			/*  0x6098 */

	/* Rx Byte Counter */
	u32 RByt;			/*  0x609C */

	/* Rx Packet Counter */
	u32 RPkt;			/*  0x60A0 */

	/* Rx FCS Error Counter */
	u32 RFcs;			/*  0x60A4 */

	/* Rx Multicast Packet Counter */
	u32 RMca;			/*  0x60A8 */

	/* Rx Broadcast Packet Counter */
	u32 RBca;			/*  0x60AC */

	/* Rx Control Frame Packet Counter */
	u32 RxCf;			/*  0x60B0 */

	/* Rx Pause Frame Packet Counter */
	u32 RxPf;			/*  0x60B4 */

	/* Rx Unknown OP Code Counter */
	u32 RxUo;			/*  0x60B8 */

	/* Rx Alignment Error Counter */
	u32 RAln;			/*  0x60BC */

	/* Rx Frame Length Error Counter */
	u32 RFlr;			/*  0x60C0 */

	/* Rx Code Error Counter */
	u32 RCde;			/*  0x60C4 */

	/* Rx Carrier Sense Error Counter */
	u32 RCse;			/*  0x60C8 */

	/* Rx Undersize Packet Counter */
	u32 RUnd;			/*  0x60CC */

	/* Rx Oversize Packet Counter */
	u32 ROvr;			/*  0x60D0 */

	/* Rx Fragment Counter */
	u32 RFrg;			/*  0x60D4 */

	/* Rx Jabber Counter */
	u32 RJbr;			/*  0x60D8 */

	/* Rx Drop */
	u32 RDrp;			/*  0x60DC */

	/* Tx Byte Counter */
	u32 TByt;			/*  0x60E0 */

	/* Tx Packet Counter */
	u32 TPkt;			/*  0x60E4 */

	/* Tx Multicast Packet Counter */
	u32 TMca;			/*  0x60E8 */

	/* Tx Broadcast Packet Counter */
	u32 TBca;			/*  0x60EC */

	/* Tx Pause Control Frame Counter */
	u32 TxPf;			/*  0x60F0 */

	/* Tx Deferral Packet Counter */
	u32 TDfr;			/*  0x60F4 */

	/* Tx Excessive Deferral Packet Counter */
	u32 TEdf;			/*  0x60F8 */

	/* Tx Single Collision Packet Counter */
	u32 TScl;			/*  0x60FC */

	/* Tx Multiple Collision Packet Counter */
	u32 TMcl;			/*  0x6100 */

	/* Tx Late Collision Packet Counter */
	u32 TLcl;			/*  0x6104 */

	/* Tx Excessive Collision Packet Counter */
	u32 TXcl;			/*  0x6108 */

	/* Tx Total Collision Packet Counter */
	u32 TNcl;			/*  0x610C */

	/* Tx Pause Frame Honored Counter */
	u32 TPfh;			/*  0x6110 */

	/* Tx Drop Frame Counter */
	u32 TDrp;			/*  0x6114 */

	/* Tx Jabber Frame Counter */
	u32 TJbr;			/*  0x6118 */

	/* Tx FCS Error Counter */
	u32 TFcs;			/*  0x611C */

	/* Tx Control Frame Counter */
	u32 TxCf;			/*  0x6120 */

	/* Tx Oversize Frame Counter */
	u32 TOvr;			/*  0x6124 */

	/* Tx Undersize Frame Counter */
	u32 TUnd;			/*  0x6128 */

	/* Tx Fragments Frame Counter */
	u32 TFrg;			/*  0x612C */

	/* Carry Register One Register */
	u32 Carry1;			/*  0x6130 */

	/* Carry Register Two Register */
	u32 Carry2;			/*  0x6134 */

	/* Carry Register One Mask Register */
	u32 Carry1M;			/*  0x6138 */

	/* Carry Register Two Mask Register */
	u32 Carry2M;			/*  0x613C */
};

/* END OF MAC STAT REGISTER ADDRESS MAP */


/* START OF MMC REGISTER ADDRESS MAP */


#define ET_MMC_ENABLE		1
#define ET_MMC_ARB_DISABLE	2
#define ET_MMC_RXMAC_DISABLE	4
#define ET_MMC_TXMAC_DISABLE	8
#define ET_MMC_TXDMA_DISABLE	16
#define ET_MMC_RXDMA_DISABLE	32
#define ET_MMC_FORCE_CE		64


#define ET_SRAM_REQ_ACCESS	1
#define ET_SRAM_WR_ACCESS	2
#define ET_SRAM_IS_CTRL		4


struct mmc_regs {		/* Location: */
	u32 mmc_ctrl;		/*  0x7000 */
	u32 sram_access;	/*  0x7004 */
	u32 sram_word1;		/*  0x7008 */
	u32 sram_word2;		/*  0x700C */
	u32 sram_word3;		/*  0x7010 */
	u32 sram_word4;		/*  0x7014 */
};

/* END OF MMC REGISTER ADDRESS MAP */


typedef struct _ADDRESS_MAP_t {
	struct global_regs global;
	/* unused section of global address map */
	u8 unused_global[4096 - sizeof(struct global_regs)];
	struct txdma_regs txdma;
	/* unused section of txdma address map */
	u8 unused_txdma[4096 - sizeof(struct txdma_regs)];
	struct rxdma_regs rxdma;
	/* unused section of rxdma address map */
	u8 unused_rxdma[4096 - sizeof(struct rxdma_regs)];
	struct txmac_regs txmac;
	/* unused section of txmac address map */
	u8 unused_txmac[4096 - sizeof(struct txmac_regs)];
	RXMAC_t rxmac;
	/* unused section of rxmac address map */
	u8 unused_rxmac[4096 - sizeof(RXMAC_t)];
	MAC_t mac;
	/* unused section of mac address map */
	u8 unused_mac[4096 - sizeof(MAC_t)];
	struct macstat_regs macstat;
	/* unused section of mac stat address map */
	u8 unused_mac_stat[4096 - sizeof(struct macstat_regs)];
	struct mmc_regs mmc;
	/* unused section of mmc address map */
	u8 unused_mmc[4096 - sizeof(struct mmc_regs)];
	/* unused section of address map */
	u8 unused_[1015808];

	u8 unused_exp_rom[4096];	/* MGS-size TBD */
	u8 unused__[524288];	/* unused section of address map */
} ADDRESS_MAP_t, *PADDRESS_MAP_t;

#endif /* _ET1310_ADDRESS_MAP_H_ */
