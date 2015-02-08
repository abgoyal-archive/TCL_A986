

#ifndef _CASSINI_H
#define _CASSINI_H

#define CAS_ID_REV2          0x02
#define CAS_ID_REVPLUS       0x10
#define CAS_ID_REVPLUS02u    0x11
#define CAS_ID_REVSATURNB2   0x30

/** global resources **/

#define  REG_CAWR	               0x0004  /* core arbitration weight */
#define    CAWR_RX_DMA_WEIGHT_SHIFT    0
#define    CAWR_RX_DMA_WEIGHT_MASK     0x03    /* [0:1] */
#define    CAWR_TX_DMA_WEIGHT_SHIFT    2
#define    CAWR_TX_DMA_WEIGHT_MASK     0x0C    /* [3:2] */
#define    CAWR_RR_DIS                 0x10    /* [4] */

#define  REG_INF_BURST                 0x0008  /* infinite burst enable reg */
#define    INF_BURST_EN                0x1     /* enable */

#define  REG_INTR_STATUS               0x000C  /* interrupt status register */
#define    INTR_TX_INTME               0x00000001  /* frame w/ INT ME desc bit set
						      xferred from host queue to
						      TX FIFO */
#define    INTR_TX_ALL                 0x00000002  /* all xmit frames xferred into
						      TX FIFO. i.e.,
						      TX Kick == TX complete. if
						      PACED_MODE set, then TX FIFO
						      also empty */
#define    INTR_TX_DONE                0x00000004  /* any frame xferred into tx
						      FIFO */
#define    INTR_TX_TAG_ERROR           0x00000008  /* TX FIFO tag framing
						      corrupted. FATAL ERROR */
#define    INTR_RX_DONE                0x00000010  /* at least 1 frame xferred
						      from RX FIFO to host mem.
						      RX completion reg updated.
						      may be delayed by recv
						      intr blanking. */
#define    INTR_RX_BUF_UNAVAIL         0x00000020  /* no more receive buffers.
						      RX Kick == RX complete */
#define    INTR_RX_TAG_ERROR           0x00000040  /* RX FIFO tag framing
						      corrupted. FATAL ERROR */
#define    INTR_RX_COMP_FULL           0x00000080  /* no more room in completion
						      ring to post descriptors.
						      RX complete head incr to
						      almost reach RX complete
						      tail */
#define    INTR_RX_BUF_AE              0x00000100  /* less than the
						      programmable threshold #
						      of free descr avail for
						      hw use */
#define    INTR_RX_COMP_AF             0x00000200  /* less than the
						      programmable threshold #
						      of descr spaces for hw
						      use in completion descr
						      ring */
#define    INTR_RX_LEN_MISMATCH        0x00000400  /* len field from MAC !=
						      len of non-reassembly pkt
						      from fifo during DMA or
						      header parser provides TCP
						      header and payload size >
						      MAC packet size.
						      FATAL ERROR */
#define    INTR_SUMMARY                0x00001000  /* summary interrupt bit. this
						      bit will be set if an interrupt
						      generated on the pci bus. useful
						      when driver is polling for
						      interrupts */
#define    INTR_PCS_STATUS             0x00002000  /* PCS interrupt status register */
#define    INTR_TX_MAC_STATUS          0x00004000  /* TX MAC status register has at
						      least 1 unmasked interrupt set */
#define    INTR_RX_MAC_STATUS          0x00008000  /* RX MAC status register has at
						      least 1 unmasked interrupt set */
#define    INTR_MAC_CTRL_STATUS        0x00010000  /* MAC control status register has
						      at least 1 unmasked interrupt
						      set */
#define    INTR_MIF_STATUS             0x00020000  /* MIF status register has at least
						      1 unmasked interrupt set */
#define    INTR_PCI_ERROR_STATUS       0x00040000  /* PCI error status register in the
						      BIF has at least 1 unmasked
						      interrupt set */
#define    INTR_TX_COMP_3_MASK         0xFFF80000  /* mask for TX completion
						      3 reg data */
#define    INTR_TX_COMP_3_SHIFT        19
#define    INTR_ERROR_MASK (INTR_MIF_STATUS | INTR_PCI_ERROR_STATUS | \
                            INTR_PCS_STATUS | INTR_RX_LEN_MISMATCH | \
                            INTR_TX_MAC_STATUS | INTR_RX_MAC_STATUS | \
                            INTR_TX_TAG_ERROR | INTR_RX_TAG_ERROR | \
                            INTR_MAC_CTRL_STATUS)

#define  REG_INTR_MASK                 0x0010  /* Interrupt mask */

#define  REG_ALIAS_CLEAR               0x0014  /* alias clear mask
						  (used w/ status alias) */
#define  REG_INTR_STATUS_ALIAS         0x001C  /* interrupt status alias
						  (selective clear) */

/* DEFAULT: 0x0, SIZE: 3 bits */
#define  REG_PCI_ERR_STATUS            0x1000  /* PCI error status */
#define    PCI_ERR_BADACK              0x01    /* reserved in Cassini+.
						  set if no ACK64# during ABS64 cycle
						  in Cassini. */
#define    PCI_ERR_DTRTO               0x02    /* delayed xaction timeout. set if
						  no read retry after 2^15 clocks */
#define    PCI_ERR_OTHER               0x04    /* other PCI errors */
#define    PCI_ERR_BIM_DMA_WRITE       0x08    /* BIM received 0 count DMA write req.
						  unused in Cassini. */
#define    PCI_ERR_BIM_DMA_READ        0x10    /* BIM received 0 count DMA read req.
						  unused in Cassini. */
#define    PCI_ERR_BIM_DMA_TIMEOUT     0x20    /* BIM received 255 retries during
						  DMA. unused in cassini. */

#define  REG_PCI_ERR_STATUS_MASK       0x1004  /* PCI Error status mask */

#define  REG_BIM_CFG                0x1008  /* BIM Configuration */
#define    BIM_CFG_RESERVED0        0x001   /* reserved */
#define    BIM_CFG_RESERVED1        0x002   /* reserved */
#define    BIM_CFG_64BIT_DISABLE    0x004   /* disable 64-bit mode */
#define    BIM_CFG_66MHZ            0x008   /* (ro) 1 = 66MHz, 0 = < 66MHz */
#define    BIM_CFG_32BIT            0x010   /* (ro) 1 = 32-bit slot, 0 = 64-bit */
#define    BIM_CFG_DPAR_INTR_ENABLE 0x020   /* detected parity err enable */
#define    BIM_CFG_RMA_INTR_ENABLE  0x040   /* master abort intr enable */
#define    BIM_CFG_RTA_INTR_ENABLE  0x080   /* target abort intr enable */
#define    BIM_CFG_RESERVED2        0x100   /* reserved */
#define    BIM_CFG_BIM_DISABLE      0x200   /* stop BIM DMA. use before global
					       reset. reserved in Cassini. */
#define    BIM_CFG_BIM_STATUS       0x400   /* (ro) 1 = BIM DMA suspended.
						  reserved in Cassini. */
#define    BIM_CFG_PERROR_BLOCK     0x800  /* block PERR# to pci bus. def: 0.
						 reserved in Cassini. */

/* DEFAULT: 0x00000000, SIZE: 32 bits */
#define  REG_BIM_DIAG                  0x100C  /* BIM Diagnostic */
#define    BIM_DIAG_MSTR_SM_MASK       0x3FFFFF00 /* PCI master controller state
						     machine bits [21:0] */
#define    BIM_DIAG_BRST_SM_MASK       0x7F    /* PCI burst controller state
						  machine bits [6:0] */

#define  REG_SW_RESET                  0x1010  /* Software reset */
#define    SW_RESET_TX                 0x00000001  /* reset TX DMA engine. poll until
						      cleared to 0.  */
#define    SW_RESET_RX                 0x00000002  /* reset RX DMA engine. poll until
						      cleared to 0. */
#define    SW_RESET_RSTOUT             0x00000004  /* force RSTOUT# pin active (low).
						      resets PHY and anything else
						      connected to RSTOUT#. RSTOUT#
						      is also activated by local PCI
						      reset when hot-swap is being
						      done. */
#define    SW_RESET_BLOCK_PCS_SLINK    0x00000008  /* if a global reset is done with
						      this bit set, PCS and SLINK
						      modules won't be reset.
						      i.e., link won't drop. */
#define    SW_RESET_BREQ_SM_MASK       0x00007F00  /* breq state machine [6:0] */
#define    SW_RESET_PCIARB_SM_MASK     0x00070000  /* pci arbitration state bits:
						      0b000: ARB_IDLE1
						      0b001: ARB_IDLE2
						      0b010: ARB_WB_ACK
						      0b011: ARB_WB_WAT
						      0b100: ARB_RB_ACK
						      0b101: ARB_RB_WAT
						      0b110: ARB_RB_END
						      0b111: ARB_WB_END */
#define    SW_RESET_RDPCI_SM_MASK      0x00300000  /* read pci state bits:
						      0b00: RD_PCI_WAT
						      0b01: RD_PCI_RDY
						      0b11: RD_PCI_ACK */
#define    SW_RESET_RDARB_SM_MASK      0x00C00000  /* read arbitration state bits:
						      0b00: AD_IDL_RX
						      0b01: AD_ACK_RX
						      0b10: AD_ACK_TX
						      0b11: AD_IDL_TX */
#define    SW_RESET_WRPCI_SM_MASK      0x06000000  /* write pci state bits
						      0b00: WR_PCI_WAT
						      0b01: WR_PCI_RDY
						      0b11: WR_PCI_ACK */
#define    SW_RESET_WRARB_SM_MASK      0x38000000  /* write arbitration state bits:
						      0b000: ARB_IDLE1
						      0b001: ARB_IDLE2
						      0b010: ARB_TX_ACK
						      0b011: ARB_TX_WAT
						      0b100: ARB_RX_ACK
						      0b110: ARB_RX_WAT */

#define  REG_MINUS_BIM_DATAPATH_TEST   0x1018  /* Cassini: BIM datapath test
						  Cassini+: reserved */

#define  REG_BIM_LOCAL_DEV_EN          0x1020  /* BIM local device
						  output EN. default: 0x7 */
#define    BIM_LOCAL_DEV_PAD           0x01    /* address bus, RW signal, and
						  OE signal output enable on the
						  local bus interface. these
						  are shared between both local
						  bus devices. tristate when 0. */
#define    BIM_LOCAL_DEV_PROM          0x02    /* PROM chip select */
#define    BIM_LOCAL_DEV_EXT           0x04    /* secondary local bus device chip
						  select output enable */
#define    BIM_LOCAL_DEV_SOFT_0        0x08    /* sw programmable ctrl bit 0 */
#define    BIM_LOCAL_DEV_SOFT_1        0x10    /* sw programmable ctrl bit 1 */
#define    BIM_LOCAL_DEV_HW_RESET      0x20    /* internal hw reset. Cassini+ only. */

#define  REG_BIM_BUFFER_ADDR           0x1024  /* BIM buffer address. for
						  purposes. */
#define    BIM_BUFFER_ADDR_MASK        0x3F    /* index (0 - 23) of buffer  */
#define    BIM_BUFFER_WR_SELECT        0x40    /* write buffer access = 1
						  read buffer access = 0 */
/* DEFAULT: undefined */
#define  REG_BIM_BUFFER_DATA_LOW       0x1028  /* BIM buffer data low */
#define  REG_BIM_BUFFER_DATA_HI        0x102C  /* BIM buffer data high */

#define  REG_BIM_RAM_BIST              0x102C  /* BIM RAM (read buffer) BIST
						  control/status */
#define    BIM_RAM_BIST_RD_START       0x01    /* start BIST for BIM read buffer */
#define    BIM_RAM_BIST_WR_START       0x02    /* start BIST for BIM write buffer.
						  Cassini only. reserved in
						  Cassini+. */
#define    BIM_RAM_BIST_RD_PASS        0x04    /* summary BIST pass status for read
						  buffer. */
#define    BIM_RAM_BIST_WR_PASS        0x08    /* summary BIST pass status for write
						  buffer. Cassini only. reserved
						  in Cassini+. */
#define    BIM_RAM_BIST_RD_LOW_PASS    0x10    /* read low bank passes BIST */
#define    BIM_RAM_BIST_RD_HI_PASS     0x20    /* read high bank passes BIST */
#define    BIM_RAM_BIST_WR_LOW_PASS    0x40    /* write low bank passes BIST.
						  Cassini only. reserved in
						  Cassini+. */
#define    BIM_RAM_BIST_WR_HI_PASS     0x80    /* write high bank passes BIST.
						  Cassini only. reserved in
						  Cassini+. */

#define  REG_BIM_DIAG_MUX              0x1030  /* BIM diagnostic probe mux
						  select register */

#define  REG_PLUS_PROBE_MUX_SELECT     0x1034 /* Cassini+: PROBE MUX SELECT */
#define    PROBE_MUX_EN                0x80000000 /* allow probe signals to be
						     driven on local bus P_A[15:0]
						     for debugging */
#define    PROBE_MUX_SUB_MUX_MASK      0x0000FF00 /* select sub module probe signals:
						     0x03 = mac[1:0]
						     0x0C = rx[1:0]
						     0x30 = tx[1:0]
						     0xC0 = hp[1:0] */
#define    PROBE_MUX_SEL_HI_MASK       0x000000F0 /* select which module to appear
						     on P_A[15:8]. see above for
						     values. */
#define    PROBE_MUX_SEL_LOW_MASK      0x0000000F /* select which module to appear
						     on P_A[7:0]. see above for
						     values. */

#define  REG_PLUS_INTR_MASK_1          0x1038 /* Cassini+: interrupt mask
						 register 2 for INTB */
#define  REG_PLUS_INTRN_MASK(x)       (REG_PLUS_INTR_MASK_1 + ((x) - 1)*16)
#define    INTR_RX_DONE_ALT              0x01
#define    INTR_RX_COMP_FULL_ALT         0x02
#define    INTR_RX_COMP_AF_ALT           0x04
#define    INTR_RX_BUF_UNAVAIL_1         0x08
#define    INTR_RX_BUF_AE_1              0x10 /* almost empty */
#define    INTRN_MASK_RX_EN              0x80
#define    INTRN_MASK_CLEAR_ALL          (INTR_RX_DONE_ALT | \
                                          INTR_RX_COMP_FULL_ALT | \
                                          INTR_RX_COMP_AF_ALT | \
                                          INTR_RX_BUF_UNAVAIL_1 | \
                                          INTR_RX_BUF_AE_1)
#define  REG_PLUS_INTR_STATUS_1        0x103C /* Cassini+: interrupt status
						 register 2 for INTB. default: 0x1F */
#define  REG_PLUS_INTRN_STATUS(x)       (REG_PLUS_INTR_STATUS_1 + ((x) - 1)*16)
#define    INTR_STATUS_ALT_INTX_EN     0x80   /* generate INTX when one of the
						 flags are set. enables desc ring. */

#define  REG_PLUS_ALIAS_CLEAR_1        0x1040 /* Cassini+: alias clear mask
						 register 2 for INTB */
#define  REG_PLUS_ALIASN_CLEAR(x)      (REG_PLUS_ALIAS_CLEAR_1 + ((x) - 1)*16)

#define  REG_PLUS_INTR_STATUS_ALIAS_1  0x1044 /* Cassini+: interrupt status
						 register alias 2 for INTB */
#define  REG_PLUS_INTRN_STATUS_ALIAS(x) (REG_PLUS_INTR_STATUS_ALIAS_1 + ((x) - 1)*16)

#define REG_SATURN_PCFG               0x106c /* pin configuration register for
						integrated macphy */

#define   SATURN_PCFG_TLA             0x00000001 /* 1 = phy actled */
#define   SATURN_PCFG_FLA             0x00000002 /* 1 = phy link10led */
#define   SATURN_PCFG_CLA             0x00000004 /* 1 = phy link100led */
#define   SATURN_PCFG_LLA             0x00000008 /* 1 = phy link1000led */
#define   SATURN_PCFG_RLA             0x00000010 /* 1 = phy duplexled */
#define   SATURN_PCFG_PDS             0x00000020 /* phy debug mode.
						    0 = normal */
#define   SATURN_PCFG_MTP             0x00000080 /* test point select */
#define   SATURN_PCFG_GMO             0x00000100 /* GMII observe. 1 =
						    GMII on SERDES pins for
						    monitoring. */
#define   SATURN_PCFG_FSI             0x00000200 /* 1 = freeze serdes/gmii. all
						    pins configed as outputs.
						    for power saving when using
						    internal phy. */
#define   SATURN_PCFG_LAD             0x00000800 /* 0 = mac core led ctrl
						    polarity from strapping
						    value.
						    1 = mac core led ctrl
						    polarity active low. */


/** transmit dma registers **/
#define MAX_TX_RINGS_SHIFT            2
#define MAX_TX_RINGS                  (1 << MAX_TX_RINGS_SHIFT)
#define MAX_TX_RINGS_MASK             (MAX_TX_RINGS - 1)

#define  REG_TX_CFG                    0x2004  /* TX config */
#define    TX_CFG_DMA_EN               0x00000001  /* enable TX DMA. if cleared, DMA
						      will stop after xfer of current
						      buffer has been completed. */
#define    TX_CFG_FIFO_PIO_SEL         0x00000002  /* TX DMA FIFO can be
						      accessed w/ FIFO addr
						      and data registers.
						      TX DMA should be
						      disabled. */
#define    TX_CFG_DESC_RING0_MASK      0x0000003C  /* # desc entries in
						      ring 1. */
#define    TX_CFG_DESC_RING0_SHIFT     2
#define    TX_CFG_DESC_RINGN_MASK(a)   (TX_CFG_DESC_RING0_MASK << (a)*4)
#define    TX_CFG_DESC_RINGN_SHIFT(a)  (TX_CFG_DESC_RING0_SHIFT + (a)*4)
#define    TX_CFG_PACED_MODE           0x00100000  /* TX_ALL only set after
						      TX FIFO becomes empty.
						      if 0, TX_ALL set
						      if descr queue empty. */
#define    TX_CFG_DMA_RDPIPE_DIS       0x01000000  /* always set to 1 */
#define    TX_CFG_COMPWB_Q1            0x02000000  /* completion writeback happens at
						      the end of every packet kicked
						      through Q1. */
#define    TX_CFG_COMPWB_Q2            0x04000000  /* completion writeback happens at
						      the end of every packet kicked
						      through Q2. */
#define    TX_CFG_COMPWB_Q3            0x08000000  /* completion writeback happens at
						      the end of every packet kicked
						      through Q3 */
#define    TX_CFG_COMPWB_Q4            0x10000000  /* completion writeback happens at
						      the end of every packet kicked
						      through Q4 */
#define    TX_CFG_INTR_COMPWB_DIS      0x20000000  /* disable pre-interrupt completion
						      writeback */
#define    TX_CFG_CTX_SEL_MASK         0xC0000000  /* selects tx test port
						      connection
						      0b00: tx mac req,
						            tx mac retry req,
							    tx ack and tx tag.
						      0b01: txdma rd req,
						            txdma rd ack,
							    txdma rd rdy,
							    txdma rd type0
						      0b11: txdma wr req,
						            txdma wr ack,
							    txdma wr rdy,
							    txdma wr xfr done. */
#define    TX_CFG_CTX_SEL_SHIFT        30

#define  REG_TX_FIFO_WRITE_PTR         0x2014  /* TX FIFO write pointer */
#define  REG_TX_FIFO_SHADOW_WRITE_PTR  0x2018  /* TX FIFO shadow write
						  pointer. temp hold reg.
					          diagnostics only. */
#define  REG_TX_FIFO_READ_PTR          0x201C  /* TX FIFO read pointer */
#define  REG_TX_FIFO_SHADOW_READ_PTR   0x2020  /* TX FIFO shadow read
						  pointer */

/* (ro) 11-bit up/down counter w/ # of frames currently in TX FIFO */
#define  REG_TX_FIFO_PKT_CNT           0x2024  /* TX FIFO packet counter */

/* current state of all state machines in TX */
#define  REG_TX_SM_1                   0x2028  /* TX state machine reg #1 */
#define    TX_SM_1_CHAIN_MASK          0x000003FF   /* chaining state machine */
#define    TX_SM_1_CSUM_MASK           0x00000C00   /* checksum state machine */
#define    TX_SM_1_FIFO_LOAD_MASK      0x0003F000   /* FIFO load state machine.
						       = 0x01 when TX disabled. */
#define    TX_SM_1_FIFO_UNLOAD_MASK    0x003C0000   /* FIFO unload state machine */
#define    TX_SM_1_CACHE_MASK          0x03C00000   /* desc. prefetch cache controller
						       state machine */
#define    TX_SM_1_CBQ_ARB_MASK        0xF8000000   /* CBQ arbiter state machine */

#define  REG_TX_SM_2                   0x202C  /* TX state machine reg #2 */
#define    TX_SM_2_COMP_WB_MASK        0x07    /* completion writeback sm */
#define	   TX_SM_2_SUB_LOAD_MASK       0x38    /* sub load state machine */
#define	   TX_SM_2_KICK_MASK           0xC0    /* kick state machine */

#define  REG_TX_DATA_PTR_LOW           0x2030  /* TX data pointer low */
#define  REG_TX_DATA_PTR_HI            0x2034  /* TX data pointer high */

#define  REG_TX_KICK0                  0x2038  /* TX kick reg #1 */
#define  REG_TX_KICKN(x)               (REG_TX_KICK0 + (x)*4)
#define  REG_TX_COMP0                  0x2048  /* TX completion reg #1 */
#define  REG_TX_COMPN(x)               (REG_TX_COMP0 + (x)*4)

#define  TX_COMPWB_SIZE             8
#define  REG_TX_COMPWB_DB_LOW       0x2058  /* TX completion write back
					       base low */
#define  REG_TX_COMPWB_DB_HI        0x205C  /* TX completion write back
					       base high */
#define    TX_COMPWB_MSB_MASK       0x00000000000000FFULL
#define    TX_COMPWB_MSB_SHIFT      0
#define    TX_COMPWB_LSB_MASK       0x000000000000FF00ULL
#define    TX_COMPWB_LSB_SHIFT      8
#define    TX_COMPWB_NEXT(x)        ((x) >> 16)

#define  REG_TX_DB0_LOW         0x2060  /* TX descriptor base low #1 */
#define  REG_TX_DB0_HI          0x2064  /* TX descriptor base hi #1 */
#define  REG_TX_DBN_LOW(x)      (REG_TX_DB0_LOW + (x)*8)
#define  REG_TX_DBN_HI(x)       (REG_TX_DB0_HI + (x)*8)

#define  REG_TX_MAXBURST_0             0x2080  /* TX MaxBurst #1 */
#define  REG_TX_MAXBURST_1             0x2084  /* TX MaxBurst #2 */
#define  REG_TX_MAXBURST_2             0x2088  /* TX MaxBurst #3 */
#define  REG_TX_MAXBURST_3             0x208C  /* TX MaxBurst #4 */

#define  REG_TX_FIFO_ADDR              0x2104  /* TX FIFO address */
#define  REG_TX_FIFO_TAG               0x2108  /* TX FIFO tag */
#define  REG_TX_FIFO_DATA_LOW          0x210C  /* TX FIFO data low */
#define  REG_TX_FIFO_DATA_HI_T1        0x2110  /* TX FIFO data high t1 */
#define  REG_TX_FIFO_DATA_HI_T0        0x2114  /* TX FIFO data high t0 */
#define  REG_TX_FIFO_SIZE              0x2118  /* (ro) TX FIFO size = 0x090 = 9KB */

#define  REG_TX_RAMBIST                0x211C /* TX RAMBIST control/status */
#define    TX_RAMBIST_STATE            0x01C0 /* progress state of RAMBIST
						 controller state machine */
#define    TX_RAMBIST_RAM33A_PASS      0x0020 /* RAM33A passed */
#define    TX_RAMBIST_RAM32A_PASS      0x0010 /* RAM32A passed */
#define    TX_RAMBIST_RAM33B_PASS      0x0008 /* RAM33B passed */
#define    TX_RAMBIST_RAM32B_PASS      0x0004 /* RAM32B passed */
#define    TX_RAMBIST_SUMMARY          0x0002 /* all RAM passed */
#define    TX_RAMBIST_START            0x0001 /* write 1 to start BIST. self
						 clears on completion. */

/** receive dma registers **/
#define MAX_RX_DESC_RINGS              2
#define MAX_RX_COMP_RINGS              4

#define  REG_RX_CFG                     0x4000  /* RX config */
#define    RX_CFG_DMA_EN                0x00000001 /* enable RX DMA. 0 stops
							 channel as soon as current
							 frame xfer has completed.
							 driver should disable MAC
							 for 200ms before disabling
							 RX */
#define    RX_CFG_DESC_RING_MASK        0x0000001E /* # desc entries in RX
							 free desc ring.
							 def: 0x8 = 8k */
#define    RX_CFG_DESC_RING_SHIFT       1
#define    RX_CFG_COMP_RING_MASK        0x000001E0 /* # desc entries in RX complete
							 ring. def: 0x8 = 32k */
#define    RX_CFG_COMP_RING_SHIFT       5
#define    RX_CFG_BATCH_DIS             0x00000200 /* disable receive desc
						      batching. def: 0x0 =
						      enabled */
#define    RX_CFG_SWIVEL_MASK           0x00001C00 /* byte offset of the 1st
						      data byte of the packet
						      w/in 8 byte boundares.
						      this swivels the data
						      DMA'ed to header
						      buffers, jumbo buffers
						      when header split is not
						      requested and MTU sized
						      buffers. def: 0x2 */
#define    RX_CFG_SWIVEL_SHIFT          10

/* cassini+ only */
#define    RX_CFG_DESC_RING1_MASK       0x000F0000 /* # of desc entries in
							 RX free desc ring 2.
							 def: 0x8 = 8k */
#define    RX_CFG_DESC_RING1_SHIFT      16


#define  REG_RX_PAGE_SIZE               0x4004  /* RX page size */
#define    RX_PAGE_SIZE_MASK            0x00000003 /* size of pages pointed to
						      by receive descriptors.
						      if jumbo buffers are
						      supported the page size
						      should not be < 8k.
						      0b00 = 2k, 0b01 = 4k
						      0b10 = 8k, 0b11 = 16k
						      DEFAULT: 8k */
#define    RX_PAGE_SIZE_SHIFT           0
#define    RX_PAGE_SIZE_MTU_COUNT_MASK  0x00007800 /* # of MTU buffers the hw
						      packs into a page.
						      DEFAULT: 4 */
#define    RX_PAGE_SIZE_MTU_COUNT_SHIFT 11
#define    RX_PAGE_SIZE_MTU_STRIDE_MASK 0x18000000 /* # of bytes that separate
							 each MTU buffer +
							 offset from each
							 other.
							 0b00 = 1k, 0b01 = 2k
							 0b10 = 4k, 0b11 = 8k
							 DEFAULT: 0x1 */
#define    RX_PAGE_SIZE_MTU_STRIDE_SHIFT 27
#define    RX_PAGE_SIZE_MTU_OFF_MASK    0xC0000000 /* offset in each page that
						      hw writes the MTU buffer
						      into.
						      0b00 = 0,
						      0b01 = 64 bytes
						      0b10 = 96, 0b11 = 128
						      DEFAULT: 0x1 */
#define    RX_PAGE_SIZE_MTU_OFF_SHIFT   30

#define  REG_RX_FIFO_WRITE_PTR             0x4008  /* RX FIFO write pointer */
#define  REG_RX_FIFO_READ_PTR              0x400C  /* RX FIFO read pointer */
#define  REG_RX_IPP_FIFO_SHADOW_WRITE_PTR  0x4010  /* RX IPP FIFO shadow write
						      pointer */
#define  REG_RX_IPP_FIFO_SHADOW_READ_PTR   0x4014  /* RX IPP FIFO shadow read
						      pointer */
#define  REG_RX_IPP_FIFO_READ_PTR          0x400C  /* RX IPP FIFO read
						      pointer. (8-bit counter) */

#define  REG_RX_DEBUG                      0x401C  /* RX debug */
#define    RX_DEBUG_LOAD_STATE_MASK        0x0000000F /* load state machine w/ MAC:
							 0x0 = idle,   0x1 = load_bop
							 0x2 = load 1, 0x3 = load 2
							 0x4 = load 3, 0x5 = load 4
							 0x6 = last detect
							 0x7 = wait req
							 0x8 = wait req statuss 1st
							 0x9 = load st
							 0xa = bubble mac
							 0xb = error */
#define    RX_DEBUG_LM_STATE_MASK          0x00000070 /* load state machine w/ HP and
							 RX FIFO:
							 0x0 = idle,   0x1 = hp xfr
							 0x2 = wait hp ready
							 0x3 = wait flow code
							 0x4 = fifo xfer
							 0x5 = make status
							 0x6 = csum ready
							 0x7 = error */
#define    RX_DEBUG_FC_STATE_MASK          0x000000180 /* flow control state machine
							 w/ MAC:
							 0x0 = idle
							 0x1 = wait xoff ack
							 0x2 = wait xon
							 0x3 = wait xon ack */
#define    RX_DEBUG_DATA_STATE_MASK        0x000001E00 /* unload data state machine
							 states:
							 0x0 = idle data
							 0x1 = header begin
							 0x2 = xfer header
							 0x3 = xfer header ld
							 0x4 = mtu begin
							 0x5 = xfer mtu
							 0x6 = xfer mtu ld
							 0x7 = jumbo begin
							 0x8 = xfer jumbo
							 0x9 = xfer jumbo ld
							 0xa = reas begin
							 0xb = xfer reas
							 0xc = flush tag
							 0xd = xfer reas ld
							 0xe = error
							 0xf = bubble idle */
#define    RX_DEBUG_DESC_STATE_MASK        0x0001E000 /* unload desc state machine
							 states:
							 0x0 = idle desc
							 0x1 = wait ack
							 0x9 = wait ack 2
							 0x2 = fetch desc 1
							 0xa = fetch desc 2
							 0x3 = load ptrs
							 0x4 = wait dma
							 0x5 = wait ack batch
							 0x6 = post batch
							 0x7 = xfr done */
#define    RX_DEBUG_INTR_READ_PTR_MASK     0x30000000 /* interrupt read ptr of the
							 interrupt queue */
#define    RX_DEBUG_INTR_WRITE_PTR_MASK    0xC0000000 /* interrupt write pointer
							 of the interrupt queue */

#define  REG_RX_PAUSE_THRESH               0x4020  /* RX pause thresholds */
#define    RX_PAUSE_THRESH_QUANTUM         64
#define    RX_PAUSE_THRESH_OFF_MASK        0x000001FF /* XOFF PAUSE emitted when
							 RX FIFO occupancy >
							 value*64B */
#define    RX_PAUSE_THRESH_OFF_SHIFT       0
#define    RX_PAUSE_THRESH_ON_MASK         0x001FF000 /* XON PAUSE emitted after
							 emitting XOFF PAUSE when RX
							 FIFO occupancy falls below
							 this value*64B. must be
							 < XOFF threshold. if =
							 RX_FIFO_SIZE< XON frames are
							 never emitted. */
#define    RX_PAUSE_THRESH_ON_SHIFT        12

#define  REG_RX_KICK                    0x4024  /* RX kick reg */

#define  REG_RX_DB_LOW                     0x4028  /* RX descriptor ring
							 base low */
#define  REG_RX_DB_HI                      0x402C  /* RX descriptor ring
							 base hi */
#define  REG_RX_CB_LOW                     0x4030  /* RX completion ring
							 base low */
#define  REG_RX_CB_HI                      0x4034  /* RX completion ring
							 base hi */
#define  REG_RX_COMP                       0x4038  /* (ro) RX completion */

#define  REG_RX_COMP_HEAD                  0x403C  /* RX completion head */
#define  REG_RX_COMP_TAIL                  0x4040  /* RX completion tail */

#define  REG_RX_BLANK                      0x4044  /* RX blanking register
							 for ISR read */
#define    RX_BLANK_INTR_PKT_MASK          0x000001FF /* RX_DONE intr asserted if
							 this many sets of completion
							 writebacks (up to 2 packets)
							 occur since the last time
							 the ISR was read. 0 = no
							 packet blanking */
#define    RX_BLANK_INTR_PKT_SHIFT         0
#define    RX_BLANK_INTR_TIME_MASK         0x3FFFF000 /* RX_DONE interrupt asserted
							 if that many clocks were
							 counted since last time the
							 ISR was read.
							 each count is 512 core
							 clocks (125MHz). 0 = no
							 time blanking */
#define    RX_BLANK_INTR_TIME_SHIFT        12

#define  REG_RX_AE_THRESH                  0x4048  /* RX almost empty
							 thresholds */
#define    RX_AE_THRESH_FREE_MASK          0x00001FFF /* RX_BUF_AE will be
							 generated if # desc
							 avail for hw use <=
							 # */
#define    RX_AE_THRESH_FREE_SHIFT         0
#define    RX_AE_THRESH_COMP_MASK          0x0FFFE000 /* RX_COMP_AE will be
							 generated if # of
							 completion entries
							 avail for hw use <=
							 # */
#define    RX_AE_THRESH_COMP_SHIFT         13

#define  REG_RX_RED                      0x404C  /* RX random early detect enable */
#define    RX_RED_4K_6K_FIFO_MASK        0x000000FF /*  4KB < FIFO thresh < 6KB */
#define    RX_RED_6K_8K_FIFO_MASK        0x0000FF00 /*  6KB < FIFO thresh < 8KB */
#define    RX_RED_8K_10K_FIFO_MASK       0x00FF0000 /*  8KB < FIFO thresh < 10KB */
#define    RX_RED_10K_12K_FIFO_MASK      0xFF000000 /* 10KB < FIFO thresh < 12KB */

#define  REG_RX_FIFO_FULLNESS              0x4050  /* (ro) RX FIFO fullness */
#define    RX_FIFO_FULLNESS_RX_FIFO_MASK   0x3FF80000 /* level w/ 8B granularity */
#define    RX_FIFO_FULLNESS_IPP_FIFO_MASK  0x0007FF00 /* level w/ 8B granularity */
#define    RX_FIFO_FULLNESS_RX_PKT_MASK    0x000000FF /* # packets in RX FIFO */
#define  REG_RX_IPP_PACKET_COUNT           0x4054  /* RX IPP packet counter */
#define  REG_RX_WORK_DMA_PTR_LOW           0x4058  /* RX working DMA ptr low */
#define  REG_RX_WORK_DMA_PTR_HI            0x405C  /* RX working DMA ptr
						      high */

#define  REG_RX_BIST                       0x4060  /* (ro) RX BIST */
#define    RX_BIST_32A_PASS                0x80000000 /* RX FIFO 32A passed */
#define    RX_BIST_33A_PASS                0x40000000 /* RX FIFO 33A passed */
#define    RX_BIST_32B_PASS                0x20000000 /* RX FIFO 32B passed */
#define    RX_BIST_33B_PASS                0x10000000 /* RX FIFO 33B passed */
#define    RX_BIST_32C_PASS                0x08000000 /* RX FIFO 32C passed */
#define    RX_BIST_33C_PASS                0x04000000 /* RX FIFO 33C passed */
#define    RX_BIST_IPP_32A_PASS            0x02000000 /* RX IPP FIFO 33B passed */
#define    RX_BIST_IPP_33A_PASS            0x01000000 /* RX IPP FIFO 33A passed */
#define    RX_BIST_IPP_32B_PASS            0x00800000 /* RX IPP FIFO 32B passed */
#define    RX_BIST_IPP_33B_PASS            0x00400000 /* RX IPP FIFO 33B passed */
#define    RX_BIST_IPP_32C_PASS            0x00200000 /* RX IPP FIFO 32C passed */
#define    RX_BIST_IPP_33C_PASS            0x00100000 /* RX IPP FIFO 33C passed */
#define    RX_BIST_CTRL_32_PASS            0x00800000 /* RX CTRL FIFO 32 passed */
#define    RX_BIST_CTRL_33_PASS            0x00400000 /* RX CTRL FIFO 33 passed */
#define    RX_BIST_REAS_26A_PASS           0x00200000 /* RX Reas 26A passed */
#define    RX_BIST_REAS_26B_PASS           0x00100000 /* RX Reas 26B passed */
#define    RX_BIST_REAS_27_PASS            0x00080000 /* RX Reas 27 passed */
#define    RX_BIST_STATE_MASK              0x00078000 /* BIST state machine */
#define    RX_BIST_SUMMARY                 0x00000002 /* when BIST complete,
							 summary pass bit
							 contains AND of BIST
							 results of all 16
							 RAMS */
#define    RX_BIST_START                   0x00000001 /* write 1 to start
							 BIST. self clears
							 on completion. */

#define  REG_RX_CTRL_FIFO_WRITE_PTR        0x4064  /* (ro) RX control FIFO
						      write ptr */
#define  REG_RX_CTRL_FIFO_READ_PTR         0x4068  /* (ro) RX control FIFO read
						      ptr */

#define  REG_RX_BLANK_ALIAS_READ           0x406C  /* RX blanking register for
						      alias read */
#define    RX_BAR_INTR_PACKET_MASK         0x000001FF /* assert RX_DONE if #
							 completion writebacks
							 > # since last ISR
							 read. 0 = no
							 blanking. up to 2
							 packets per
							 completion wb. */
#define    RX_BAR_INTR_TIME_MASK           0x3FFFF000 /* assert RX_DONE if #
							 clocks > # since last
							 ISR read. each count
							 is 512 core clocks
							 (125MHz). 0 = no
							 blanking. */

#define  REG_RX_FIFO_ADDR                  0x4080  /* RX FIFO address */
#define  REG_RX_FIFO_TAG                   0x4084  /* RX FIFO tag */
#define  REG_RX_FIFO_DATA_LOW              0x4088  /* RX FIFO data low */
#define  REG_RX_FIFO_DATA_HI_T0            0x408C  /* RX FIFO data high T0 */
#define  REG_RX_FIFO_DATA_HI_T1            0x4090  /* RX FIFO data high T1 */

#define  REG_RX_CTRL_FIFO_ADDR             0x4094  /* RX Control FIFO and
						      Batching FIFO addr */
#define  REG_RX_CTRL_FIFO_DATA_LOW         0x4098  /* RX Control FIFO data
						      low */
#define  REG_RX_CTRL_FIFO_DATA_MID         0x409C  /* RX Control FIFO data
						      mid */
#define  REG_RX_CTRL_FIFO_DATA_HI          0x4100  /* RX Control FIFO data
						      hi and flow id */
#define    RX_CTRL_FIFO_DATA_HI_CTRL       0x0001  /* upper bit of ctrl word */
#define    RX_CTRL_FIFO_DATA_HI_FLOW_MASK  0x007E  /* flow id */

#define  REG_RX_IPP_FIFO_ADDR              0x4104  /* RX IPP FIFO address */
#define  REG_RX_IPP_FIFO_TAG               0x4108  /* RX IPP FIFO tag */
#define  REG_RX_IPP_FIFO_DATA_LOW          0x410C  /* RX IPP FIFO data low */
#define  REG_RX_IPP_FIFO_DATA_HI_T0        0x4110  /* RX IPP FIFO data high
						      T0 */
#define  REG_RX_IPP_FIFO_DATA_HI_T1        0x4114  /* RX IPP FIFO data high
						      T1 */

#define  REG_RX_HEADER_PAGE_PTR_LOW        0x4118  /* (ro) RX header page ptr
						      low */
#define  REG_RX_HEADER_PAGE_PTR_HI         0x411C  /* (ro) RX header page ptr
						      high */
#define  REG_RX_MTU_PAGE_PTR_LOW           0x4120  /* (ro) RX MTU page pointer
						      low */
#define  REG_RX_MTU_PAGE_PTR_HI            0x4124  /* (ro) RX MTU page pointer
						      high */

#define  REG_RX_TABLE_ADDR             0x4128  /* RX reassembly DMA table
						  address */
#define    RX_TABLE_ADDR_MASK          0x0000003F /* address mask */

#define  REG_RX_TABLE_DATA_LOW         0x412C  /* RX reassembly DMA table
						  data low */
#define  REG_RX_TABLE_DATA_MID         0x4130  /* RX reassembly DMA table
						  data mid */
#define  REG_RX_TABLE_DATA_HI          0x4134  /* RX reassembly DMA table
						  data high */

/* cassini+ only */
#define  REG_PLUS_RX_DB1_LOW            0x4200  /* RX descriptor ring
						   2 base low */
#define  REG_PLUS_RX_DB1_HI             0x4204  /* RX descriptor ring
						   2 base high */
#define  REG_PLUS_RX_CB1_LOW            0x4208  /* RX completion ring
						   2 base low. 4 total */
#define  REG_PLUS_RX_CB1_HI             0x420C  /* RX completion ring
						   2 base high. 4 total */
#define  REG_PLUS_RX_CBN_LOW(x)        (REG_PLUS_RX_CB1_LOW + 8*((x) - 1))
#define  REG_PLUS_RX_CBN_HI(x)         (REG_PLUS_RX_CB1_HI + 8*((x) - 1))
#define  REG_PLUS_RX_KICK1             0x4220  /* RX Kick 2 register */
#define  REG_PLUS_RX_COMP1             0x4224  /* (ro) RX completion 2
						  reg */
#define  REG_PLUS_RX_COMP1_HEAD        0x4228  /* (ro) RX completion 2
						  head reg. 4 total. */
#define  REG_PLUS_RX_COMP1_TAIL        0x422C  /* RX completion 2
						  tail reg. 4 total. */
#define  REG_PLUS_RX_COMPN_HEAD(x)    (REG_PLUS_RX_COMP1_HEAD + 8*((x) - 1))
#define  REG_PLUS_RX_COMPN_TAIL(x)    (REG_PLUS_RX_COMP1_TAIL + 8*((x) - 1))
#define  REG_PLUS_RX_AE1_THRESH        0x4240  /* RX almost empty 2
						  thresholds */
#define    RX_AE1_THRESH_FREE_MASK     RX_AE_THRESH_FREE_MASK
#define    RX_AE1_THRESH_FREE_SHIFT    RX_AE_THRESH_FREE_SHIFT

/** header parser registers **/

#define  REG_HP_CFG                       0x4140  /* header parser
						     configuration reg */
#define    HP_CFG_PARSE_EN                0x00000001 /* enab header parsing */
#define    HP_CFG_NUM_CPU_MASK            0x000000FC /* # processors
						      0 = 64. 0x3f = 63 */
#define    HP_CFG_NUM_CPU_SHIFT           2
#define    HP_CFG_SYN_INC_MASK            0x00000100 /* SYN bit won't increment
							TCP seq # by one when
							stored in FDBM */
#define    HP_CFG_TCP_THRESH_MASK         0x000FFE00 /* # bytes of TCP data
							needed to be considered
							for reassembly */
#define    HP_CFG_TCP_THRESH_SHIFT        9

#define  REG_HP_INSTR_RAM_ADDR             0x4144  /* HP instruction RAM
						      address */
#define    HP_INSTR_RAM_ADDR_MASK          0x01F   /* 5-bit mask */
#define  REG_HP_INSTR_RAM_DATA_LOW         0x4148  /* HP instruction RAM
						      data low */
#define    HP_INSTR_RAM_LOW_OUTMASK_MASK   0x0000FFFF
#define    HP_INSTR_RAM_LOW_OUTMASK_SHIFT  0
#define    HP_INSTR_RAM_LOW_OUTSHIFT_MASK  0x000F0000
#define    HP_INSTR_RAM_LOW_OUTSHIFT_SHIFT 16
#define    HP_INSTR_RAM_LOW_OUTEN_MASK     0x00300000
#define    HP_INSTR_RAM_LOW_OUTEN_SHIFT    20
#define    HP_INSTR_RAM_LOW_OUTARG_MASK    0xFFC00000
#define    HP_INSTR_RAM_LOW_OUTARG_SHIFT   22
#define  REG_HP_INSTR_RAM_DATA_MID         0x414C  /* HP instruction RAM
						      data mid */
#define    HP_INSTR_RAM_MID_OUTARG_MASK    0x00000003
#define    HP_INSTR_RAM_MID_OUTARG_SHIFT   0
#define    HP_INSTR_RAM_MID_OUTOP_MASK     0x0000003C
#define    HP_INSTR_RAM_MID_OUTOP_SHIFT    2
#define    HP_INSTR_RAM_MID_FNEXT_MASK     0x000007C0
#define    HP_INSTR_RAM_MID_FNEXT_SHIFT    6
#define    HP_INSTR_RAM_MID_FOFF_MASK      0x0003F800
#define    HP_INSTR_RAM_MID_FOFF_SHIFT     11
#define    HP_INSTR_RAM_MID_SNEXT_MASK     0x007C0000
#define    HP_INSTR_RAM_MID_SNEXT_SHIFT    18
#define    HP_INSTR_RAM_MID_SOFF_MASK      0x3F800000
#define    HP_INSTR_RAM_MID_SOFF_SHIFT     23
#define    HP_INSTR_RAM_MID_OP_MASK        0xC0000000
#define    HP_INSTR_RAM_MID_OP_SHIFT       30
#define  REG_HP_INSTR_RAM_DATA_HI          0x4150  /* HP instruction RAM
						      data high */
#define    HP_INSTR_RAM_HI_VAL_MASK        0x0000FFFF
#define    HP_INSTR_RAM_HI_VAL_SHIFT       0
#define    HP_INSTR_RAM_HI_MASK_MASK       0xFFFF0000
#define    HP_INSTR_RAM_HI_MASK_SHIFT      16

#define  REG_HP_DATA_RAM_FDB_ADDR          0x4154  /* HP data and FDB
						      RAM address */
#define    HP_DATA_RAM_FDB_DATA_MASK       0x001F  /* select 1 of 86 byte
						      locations in header
						      parser data ram to
						      read/write */
#define    HP_DATA_RAM_FDB_FDB_MASK        0x3F00  /* 1 of 64 353-bit locations
						      in the flow database */
#define  REG_HP_DATA_RAM_DATA              0x4158  /* HP data RAM data */

#define  REG_HP_FLOW_DB0                   0x415C  /* HP flow database 1 reg */
#define  REG_HP_FLOW_DBN(x)                (REG_HP_FLOW_DB0 + (x)*4)

#define  REG_HP_STATE_MACHINE              0x418C  /* (ro) HP state machine */
#define  REG_HP_STATUS0                    0x4190  /* (ro) HP status 1 */
#define    HP_STATUS0_SAP_MASK             0xFFFF0000 /* SAP */
#define    HP_STATUS0_L3_OFF_MASK          0x0000FE00 /* L3 offset */
#define    HP_STATUS0_LB_CPUNUM_MASK       0x000001F8 /* load balancing CPU
							 number */
#define    HP_STATUS0_HRP_OPCODE_MASK      0x00000007 /* HRP opcode */

#define  REG_HP_STATUS1                    0x4194  /* (ro) HP status 2 */
#define    HP_STATUS1_ACCUR2_MASK          0xE0000000 /* accu R2[6:4] */
#define    HP_STATUS1_FLOWID_MASK          0x1F800000 /* flow id */
#define    HP_STATUS1_TCP_OFF_MASK         0x007F0000 /* tcp payload offset */
#define    HP_STATUS1_TCP_SIZE_MASK        0x0000FFFF /* tcp payload size */

#define  REG_HP_STATUS2                    0x4198  /* (ro) HP status 3 */
#define    HP_STATUS2_ACCUR2_MASK          0xF0000000 /* accu R2[3:0] */
#define    HP_STATUS2_CSUM_OFF_MASK        0x07F00000 /* checksum start
							 start offset */
#define    HP_STATUS2_ACCUR1_MASK          0x000FE000 /* accu R1 */
#define    HP_STATUS2_FORCE_DROP           0x00001000 /* force drop */
#define    HP_STATUS2_BWO_REASSM           0x00000800 /* batching w/o
							 reassembly */
#define    HP_STATUS2_JH_SPLIT_EN          0x00000400 /* jumbo header split
							 enable */
#define    HP_STATUS2_FORCE_TCP_NOCHECK    0x00000200 /* force tcp no payload
							 check */
#define    HP_STATUS2_DATA_MASK_ZERO       0x00000100 /* mask of data length
							 equal to zero */
#define    HP_STATUS2_FORCE_TCP_CHECK      0x00000080 /* force tcp payload
							 chk */
#define    HP_STATUS2_MASK_TCP_THRESH      0x00000040 /* mask of payload
							 threshold */
#define    HP_STATUS2_NO_ASSIST            0x00000020 /* no assist */
#define    HP_STATUS2_CTRL_PACKET_FLAG     0x00000010 /* control packet flag */
#define    HP_STATUS2_TCP_FLAG_CHECK       0x00000008 /* tcp flag check */
#define    HP_STATUS2_SYN_FLAG             0x00000004 /* syn flag */
#define    HP_STATUS2_TCP_CHECK            0x00000002 /* tcp payload chk */
#define    HP_STATUS2_TCP_NOCHECK          0x00000001 /* tcp no payload chk */

#define  REG_HP_RAM_BIST                   0x419C  /* HP RAM BIST reg */
#define    HP_RAM_BIST_HP_DATA_PASS        0x80000000 /* HP data ram */
#define    HP_RAM_BIST_HP_INSTR0_PASS      0x40000000 /* HP instr ram 0 */
#define    HP_RAM_BIST_HP_INSTR1_PASS      0x20000000 /* HP instr ram 1 */
#define    HP_RAM_BIST_HP_INSTR2_PASS      0x10000000 /* HP instr ram 2 */
#define    HP_RAM_BIST_FDBM_AGE0_PASS      0x08000000 /* FDBM aging RAM0 */
#define    HP_RAM_BIST_FDBM_AGE1_PASS      0x04000000 /* FDBM aging RAM1 */
#define    HP_RAM_BIST_FDBM_FLOWID00_PASS  0x02000000 /* FDBM flowid RAM0
							 bank 0 */
#define    HP_RAM_BIST_FDBM_FLOWID10_PASS  0x01000000 /* FDBM flowid RAM1
							 bank 0 */
#define    HP_RAM_BIST_FDBM_FLOWID20_PASS  0x00800000 /* FDBM flowid RAM2
							 bank 0 */
#define    HP_RAM_BIST_FDBM_FLOWID30_PASS  0x00400000 /* FDBM flowid RAM3
							 bank 0 */
#define    HP_RAM_BIST_FDBM_FLOWID01_PASS  0x00200000 /* FDBM flowid RAM0
							 bank 1 */
#define    HP_RAM_BIST_FDBM_FLOWID11_PASS  0x00100000 /* FDBM flowid RAM1
							 bank 2 */
#define    HP_RAM_BIST_FDBM_FLOWID21_PASS  0x00080000 /* FDBM flowid RAM2
							 bank 1 */
#define    HP_RAM_BIST_FDBM_FLOWID31_PASS  0x00040000 /* FDBM flowid RAM3
							 bank 1 */
#define    HP_RAM_BIST_FDBM_TCPSEQ_PASS    0x00020000 /* FDBM tcp sequence
							 RAM */
#define    HP_RAM_BIST_SUMMARY             0x00000002 /* all BIST tests */
#define    HP_RAM_BIST_START               0x00000001 /* start/stop BIST */


/** MAC registers.  **/
#define  REG_MAC_TX_RESET                  0x6000  /* TX MAC software reset
						      command (default: 0x0) */
#define  REG_MAC_RX_RESET                  0x6004  /* RX MAC software reset
						      command (default: 0x0) */
#define  REG_MAC_SEND_PAUSE                0x6008  /* send pause command reg */
#define    MAC_SEND_PAUSE_TIME_MASK        0x0000FFFF /* value of pause time
							 to be sent on network
							 in units of slot
							 times */
#define    MAC_SEND_PAUSE_SEND             0x00010000 /* send pause flow ctrl
							 frame on network */

#define  REG_MAC_TX_STATUS                 0x6010  /* TX MAC status reg */
#define    MAC_TX_FRAME_XMIT               0x0001  /* successful frame
						      transmision */
#define    MAC_TX_UNDERRUN                 0x0002  /* terminated frame
						      transmission due to
						      data starvation in the
						      xmit data path */
#define    MAC_TX_MAX_PACKET_ERR           0x0004  /* frame exceeds max allowed
						      length passed to TX MAC
						      by the DMA engine */
#define    MAC_TX_COLL_NORMAL              0x0008  /* rollover of the normal
						      collision counter */
#define    MAC_TX_COLL_EXCESS              0x0010  /* rollover of the excessive
						      collision counter */
#define    MAC_TX_COLL_LATE                0x0020  /* rollover of the late
						      collision counter */
#define    MAC_TX_COLL_FIRST               0x0040  /* rollover of the first
						      collision counter */
#define    MAC_TX_DEFER_TIMER              0x0080  /* rollover of the defer
						      timer */
#define    MAC_TX_PEAK_ATTEMPTS            0x0100  /* rollover of the peak
						      attempts counter */

#define  REG_MAC_RX_STATUS                 0x6014  /* RX MAC status reg */
#define    MAC_RX_FRAME_RECV               0x0001  /* successful receipt of
						      a frame */
#define    MAC_RX_OVERFLOW                 0x0002  /* dropped frame due to
						      RX FIFO overflow */
#define    MAC_RX_FRAME_COUNT              0x0004  /* rollover of receive frame
						      counter */
#define    MAC_RX_ALIGN_ERR                0x0008  /* rollover of alignment
						      error counter */
#define    MAC_RX_CRC_ERR                  0x0010  /* rollover of crc error
						      counter */
#define    MAC_RX_LEN_ERR                  0x0020  /* rollover of length
						      error counter */
#define    MAC_RX_VIOL_ERR                 0x0040  /* rollover of code
						      violation error */

/* DEFAULT: 0xXXXX0000 on reset */
#define  REG_MAC_CTRL_STATUS               0x6018  /* MAC control status reg */
#define    MAC_CTRL_PAUSE_RECEIVED         0x00000001  /* successful
							  reception of a
							  pause control
							  frame */
#define    MAC_CTRL_PAUSE_STATE            0x00000002  /* MAC has made a
							  transition from
							  "not paused" to
							  "paused" */
#define    MAC_CTRL_NOPAUSE_STATE          0x00000004  /* MAC has made a
							  transition from
							  "paused" to "not
							  paused" */
#define    MAC_CTRL_PAUSE_TIME_MASK        0xFFFF0000  /* value of pause time
							  operand that was
							  received in the last
							  pause flow control
							  frame */

/* layout identical to TX MAC[8:0] */
#define  REG_MAC_TX_MASK                   0x6020  /* TX MAC mask reg */
/* layout identical to RX MAC[6:0] */
#define  REG_MAC_RX_MASK                   0x6024  /* RX MAC mask reg */
/* layout identical to CTRL MAC[2:0] */
#define  REG_MAC_CTRL_MASK                 0x6028  /* MAC control mask reg */

#define  REG_MAC_TX_CFG                 0x6030  /* TX MAC config reg */
#define    MAC_TX_CFG_EN                0x0001  /* enable TX MAC. 0 will
						      force TXMAC state
						      machine to remain in
						      idle state or to
						      transition to idle state
						      on completion of an
						      ongoing packet. */
#define    MAC_TX_CFG_IGNORE_CARRIER    0x0002  /* disable CSMA/CD deferral
						   process. set to 1 when
						   full duplex and 0 when
						   half duplex */
#define    MAC_TX_CFG_IGNORE_COLL       0x0004  /* disable CSMA/CD backoff
						   algorithm. set to 1 when
						   full duplex and 0 when
						   half duplex */
#define    MAC_TX_CFG_IPG_EN            0x0008  /* enable extension of the
						   Rx-to-TX IPG. after
						   receiving a frame, TX
						   MAC will reset its
						   deferral process to
						   carrier sense for the
						   amount of time = IPG0 +
						   IPG1 and commit to
						   transmission for time
						   specified in IPG2. when
						   0 or when xmitting frames
						   back-to-pack (Tx-to-Tx
						   IPG), TX MAC ignores
						   IPG0 and will only use
						   IPG1 for deferral time.
						   IPG2 still used. */
#define    MAC_TX_CFG_NEVER_GIVE_UP_EN  0x0010  /* TX MAC will not easily
						   give up on frame
						   xmission. if backoff
						   algorithm reaches the
						   ATTEMPT_LIMIT, it will
						   clear attempts counter
						   and continue trying to
						   send the frame as
						   specified by
						   GIVE_UP_LIM. when 0,
						   TX MAC will execute
						   standard CSMA/CD prot. */
#define    MAC_TX_CFG_NEVER_GIVE_UP_LIM 0x0020  /* when set, TX MAC will
						   continue to try to xmit
						   until successful. when
						   0, TX MAC will continue
						   to try xmitting until
						   successful or backoff
						   algorithm reaches
						   ATTEMPT_LIMIT*16 */
#define    MAC_TX_CFG_NO_BACKOFF        0x0040  /* modify CSMA/CD to disable
						   backoff algorithm. TX
						   MAC will not back off
						   after a xmission attempt
						   that resulted in a
						   collision. */
#define    MAC_TX_CFG_SLOW_DOWN         0x0080  /* modify CSMA/CD so that
						   deferral process is reset
						   in response to carrier
						   sense during the entire
						   duration of IPG. TX MAC
						   will only commit to frame
						   xmission after frame
						   xmission has actually
						   begun. */
#define    MAC_TX_CFG_NO_FCS            0x0100  /* TX MAC will not generate
						   CRC for all xmitted
						   packets. when clear, CRC
						   generation is dependent
						   upon NO_CRC bit in the
						   xmit control word from
						   TX DMA */
#define    MAC_TX_CFG_CARRIER_EXTEND    0x0200  /* enables xmit part of the
						   carrier extension
						   feature. this allows for
						   longer collision domains
						   by extending the carrier
						   and collision window
						   from the end of FCS until
						   the end of the slot time
						   if necessary. Required
						   for half-duplex at 1Gbps,
						   clear otherwise. */

#define  REG_MAC_RX_CFG                 0x6034  /* RX MAC config reg */
#define    MAC_RX_CFG_EN                0x0001  /* enable RX MAC */
#define    MAC_RX_CFG_STRIP_PAD         0x0002  /* always program to 0.
						   feature not supported */
#define    MAC_RX_CFG_STRIP_FCS         0x0004  /* RX MAC will strip the
						   last 4 bytes of a
						   received frame. */
#define    MAC_RX_CFG_PROMISC_EN        0x0008  /* promiscuous mode */
#define    MAC_RX_CFG_PROMISC_GROUP_EN  0x0010  /* accept all valid
						   multicast frames (group
						   bit in DA field set) */
#define    MAC_RX_CFG_HASH_FILTER_EN    0x0020  /* use hash table to filter
						   multicast addresses */
#define    MAC_RX_CFG_ADDR_FILTER_EN    0x0040  /* cause RX MAC to use
						   address filtering regs
						   to filter both unicast
						   and multicast
						   addresses */
#define    MAC_RX_CFG_DISABLE_DISCARD   0x0080  /* pass errored frames to
						   RX DMA by setting BAD
						   bit but not Abort bit
						   in the status. CRC,
						   framing, and length errs
						   will not increment
						   error counters. frames
						   which don't match dest
						   addr will be passed up
						   w/ BAD bit set. */
#define    MAC_RX_CFG_CARRIER_EXTEND    0x0100  /* enable reception of
						   packet bursts generated
						   by carrier extension
						   with packet bursting
						   senders. only applies
						   to half-duplex 1Gbps */

/* DEFAULT: 0x0 */
#define  REG_MAC_CTRL_CFG               0x6038  /* MAC control config reg */
#define    MAC_CTRL_CFG_SEND_PAUSE_EN   0x0001  /* respond to requests for
						   sending pause flow ctrl
						   frames */
#define    MAC_CTRL_CFG_RECV_PAUSE_EN   0x0002  /* respond to received
						   pause flow ctrl frames */
#define    MAC_CTRL_CFG_PASS_CTRL       0x0004  /* pass valid MAC ctrl
						   packets to RX DMA */

#define  REG_MAC_XIF_CFG                0x603C  /* XIF config reg */
#define    MAC_XIF_TX_MII_OUTPUT_EN        0x0001  /* enable output drivers
						      on MII xmit bus */
#define    MAC_XIF_MII_INT_LOOPBACK        0x0002  /* loopback GMII xmit data
						      path to GMII recv data
						      path. phy mode register
						      clock selection must be
						      set to GMII mode and
						      GMII_MODE should be set
						      to 1. in loopback mode,
						      REFCLK will drive the
						      entire mac core. 0 for
						      normal operation. */
#define    MAC_XIF_DISABLE_ECHO            0x0004  /* disables receive data
						      path during packet
						      xmission. clear to 0
						      in any full duplex mode,
						      in any loopback mode,
						      or in half-duplex SERDES
						      or SLINK modes. set when
						      in half-duplex when
						      using external phy. */
#define    MAC_XIF_GMII_MODE               0x0008  /* MAC operates with GMII
						      clocks and datapath */
#define    MAC_XIF_MII_BUFFER_OUTPUT_EN    0x0010  /* MII_BUF_EN pin. enable
						      external tristate buffer
						      on the MII receive
						      bus. */
#define    MAC_XIF_LINK_LED                0x0020  /* LINKLED# active (low) */
#define    MAC_XIF_FDPLX_LED               0x0040  /* FDPLXLED# active (low) */

#define  REG_MAC_IPG0                      0x6040  /* inter-packet gap0 reg.
						      recommended: 0x00 */
#define  REG_MAC_IPG1                      0x6044  /* inter-packet gap1 reg
						      recommended: 0x08 */
#define  REG_MAC_IPG2                      0x6048  /* inter-packet gap2 reg
						      recommended: 0x04 */
#define  REG_MAC_SLOT_TIME                 0x604C  /* slot time reg
						      recommended: 0x40 */
#define  REG_MAC_FRAMESIZE_MIN             0x6050  /* min frame size reg
						      recommended: 0x40 */

#define  REG_MAC_FRAMESIZE_MAX             0x6054  /* max frame size reg */
#define    MAC_FRAMESIZE_MAX_BURST_MASK    0x3FFF0000 /* max burst size */
#define    MAC_FRAMESIZE_MAX_BURST_SHIFT   16
#define    MAC_FRAMESIZE_MAX_FRAME_MASK    0x00007FFF /* max frame size */
#define    MAC_FRAMESIZE_MAX_FRAME_SHIFT   0
#define  REG_MAC_PA_SIZE                   0x6058  /* PA size reg. number of
						      preamble bytes that the
						      TX MAC will xmit at the
						      beginning of each frame
						      value should be 2 or
						      greater. recommended
						      value: 0x07 */
#define  REG_MAC_JAM_SIZE                  0x605C  /* jam size reg. duration
						      of jam in units of media
						      byte time. recommended
						      value: 0x04 */
#define  REG_MAC_ATTEMPT_LIMIT             0x6060  /* attempt limit reg. #
						      of attempts TX MAC will
						      make to xmit a frame
						      before it resets its
						      attempts counter. after
						      the limit has been
						      reached, TX MAC may or
						      may not drop the frame
						      dependent upon value
						      in TX_MAC_CFG.
						      recommended
						      value: 0x10 */
#define  REG_MAC_CTRL_TYPE                 0x6064  /* MAC control type reg.
						      type field of a MAC
						      ctrl frame. recommended
						      value: 0x8808 */

#define  REG_MAC_ADDR0                     0x6080  /* MAC address 0 reg */
#define  REG_MAC_ADDRN(x)                  (REG_MAC_ADDR0 + (x)*4)
#define  REG_MAC_ADDR_FILTER0              0x614C  /* address filter 0 reg
						      [47:32] */
#define  REG_MAC_ADDR_FILTER1              0x6150  /* address filter 1 reg
						      [31:16] */
#define  REG_MAC_ADDR_FILTER2              0x6154  /* address filter 2 reg
						      [15:0] */
#define  REG_MAC_ADDR_FILTER2_1_MASK       0x6158  /* address filter 2 and 1
						      mask reg. 8-bit reg
						      contains nibble mask for
						      reg 2 and 1. */
#define  REG_MAC_ADDR_FILTER0_MASK         0x615C  /* address filter 0 mask
						      reg */

#define  REG_MAC_HASH_TABLE0               0x6160  /* hash table 0 reg */
#define  REG_MAC_HASH_TABLEN(x)            (REG_MAC_HASH_TABLE0 + (x)*4)

#define  REG_MAC_COLL_NORMAL               0x61A0 /* normal collision
						     counter. */
#define  REG_MAC_COLL_FIRST                0x61A4 /* first attempt
						     successful collision
						     counter */
#define  REG_MAC_COLL_EXCESS               0x61A8 /* excessive collision
						     counter */
#define  REG_MAC_COLL_LATE                 0x61AC /* late collision counter */
#define  REG_MAC_TIMER_DEFER               0x61B0 /* defer timer. time base
						     is the media byte
						     clock/256 */
#define  REG_MAC_ATTEMPTS_PEAK             0x61B4 /* peak attempts reg */
#define  REG_MAC_RECV_FRAME                0x61B8 /* receive frame counter */
#define  REG_MAC_LEN_ERR                   0x61BC /* length error counter */
#define  REG_MAC_ALIGN_ERR                 0x61C0 /* alignment error counter */
#define  REG_MAC_FCS_ERR                   0x61C4 /* FCS error counter */
#define  REG_MAC_RX_CODE_ERR               0x61C8 /* RX code violation
						     error counter */

/* misc registers */
#define  REG_MAC_RANDOM_SEED               0x61CC /* random number seed reg.
						   10-bit register used as a
						   seed  for the random number
						   generator for the CSMA/CD
						   backoff algorithm. only
						   programmed after power-on
						   reset and should be a
						   random value which has a
						   high likelihood of being
						   unique for each MAC
						   attached to a network
						   segment (e.g., 10 LSB of
						   MAC address) */


/* 27-bit register has the current state for key state machines in the MAC */
#define  REG_MAC_STATE_MACHINE             0x61D0 /* (ro) state machine reg */
#define    MAC_SM_RLM_MASK                 0x07800000
#define    MAC_SM_RLM_SHIFT                23
#define    MAC_SM_RX_FC_MASK               0x00700000
#define    MAC_SM_RX_FC_SHIFT              20
#define    MAC_SM_TLM_MASK                 0x000F0000
#define    MAC_SM_TLM_SHIFT                16
#define    MAC_SM_ENCAP_SM_MASK            0x0000F000
#define    MAC_SM_ENCAP_SM_SHIFT           12
#define    MAC_SM_TX_REQ_MASK              0x00000C00
#define    MAC_SM_TX_REQ_SHIFT             10
#define    MAC_SM_TX_FC_MASK               0x000003C0
#define    MAC_SM_TX_FC_SHIFT              6
#define    MAC_SM_FIFO_WRITE_SEL_MASK      0x00000038
#define    MAC_SM_FIFO_WRITE_SEL_SHIFT     3
#define    MAC_SM_TX_FIFO_EMPTY_MASK       0x00000007
#define    MAC_SM_TX_FIFO_EMPTY_SHIFT      0

#define  REG_MIF_BIT_BANG_CLOCK            0x6200 /* MIF bit-bang clock.
						   1 -> 0 will generate a
						   rising edge. 0 -> 1 will
						   generate a falling edge. */
#define  REG_MIF_BIT_BANG_DATA             0x6204 /* MIF bit-bang data. 1-bit
						     register generates data */
#define  REG_MIF_BIT_BANG_OUTPUT_EN        0x6208 /* MIF bit-bang output
						     enable. enable when
						     xmitting data from MIF to
						     transceiver. */

#define  REG_MIF_FRAME                     0x620C /* MIF frame/output reg */
#define    MIF_FRAME_START_MASK            0xC0000000 /* start of frame.
							 load w/ 01 when
							 issuing an instr */
#define    MIF_FRAME_ST                    0x40000000 /* STart of frame */
#define    MIF_FRAME_OPCODE_MASK           0x30000000 /* opcode. 01 for a
							 write. 10 for a
							 read */
#define    MIF_FRAME_OP_READ               0x20000000 /* read OPcode */
#define    MIF_FRAME_OP_WRITE              0x10000000 /* write OPcode */
#define    MIF_FRAME_PHY_ADDR_MASK         0x0F800000 /* phy address. when
							 issuing an instr,
							 this field should be
							 loaded w/ the XCVR
							 addr */
#define    MIF_FRAME_PHY_ADDR_SHIFT        23
#define    MIF_FRAME_REG_ADDR_MASK         0x007C0000 /* register address.
							 when issuing an instr,
							 addr of register
							 to be read/written */
#define    MIF_FRAME_REG_ADDR_SHIFT        18
#define    MIF_FRAME_TURN_AROUND_MSB       0x00020000 /* turn around, MSB.
							 when issuing an instr,
							 set this bit to 1 */
#define    MIF_FRAME_TURN_AROUND_LSB       0x00010000 /* turn around, LSB.
							 when issuing an instr,
							 set this bit to 0.
							 when polling for
							 completion, 1 means
							 that instr execution
							 has been completed */
#define    MIF_FRAME_DATA_MASK             0x0000FFFF /* instruction payload
							 load with 16-bit data
							 to be written in
							 transceiver reg for a
							 write. doesn't matter
							 in a read. when
							 polling for
							 completion, field is
							 "don't care" for write
							 and 16-bit data
							 returned by the
							 transceiver for a
							 read (if valid bit
							 is set) */
#define  REG_MIF_CFG                    0x6210 /* MIF config reg */
#define    MIF_CFG_PHY_SELECT           0x0001 /* 1 -> select MDIO_1
						  0 -> select MDIO_0 */
#define    MIF_CFG_POLL_EN              0x0002 /* enable polling
						  mechanism. if set,
						  BB_MODE should be 0 */
#define    MIF_CFG_BB_MODE              0x0004 /* 1 -> bit-bang mode
						  0 -> frame mode */
#define    MIF_CFG_POLL_REG_MASK        0x00F8 /* register address to be
						  used by polling mode.
						  only meaningful if POLL_EN
						  is set to 1 */
#define    MIF_CFG_POLL_REG_SHIFT       3
#define    MIF_CFG_MDIO_0               0x0100 /* (ro) dual purpose.
						  when MDIO_0 is idle,
						  1 -> tranceiver is
						  connected to MDIO_0.
						  when MIF is communicating
						  w/ MDIO_0 in bit-bang
						  mode, this bit indicates
						  the incoming bit stream
						  during a read op */
#define    MIF_CFG_MDIO_1               0x0200 /* (ro) dual purpose.
						  when MDIO_1 is idle,
						  1 -> transceiver is
						  connected to MDIO_1.
						  when MIF is communicating
						  w/ MDIO_1 in bit-bang
						  mode, this bit indicates
						  the incoming bit stream
						  during a read op */
#define    MIF_CFG_POLL_PHY_MASK        0x7C00 /* tranceiver address to
						  be polled */
#define    MIF_CFG_POLL_PHY_SHIFT       10

#define  REG_MIF_MASK                      0x6214 /* MIF mask reg */

/* 32-bit register used when in poll mode. auto-cleared after being read */
#define  REG_MIF_STATUS                    0x6218 /* MIF status reg */
#define    MIF_STATUS_POLL_DATA_MASK       0xFFFF0000 /* poll data contains
							 the "latest image"
							 update of the XCVR
							 reg being read */
#define    MIF_STATUS_POLL_DATA_SHIFT      16
#define    MIF_STATUS_POLL_STATUS_MASK     0x0000FFFF /* poll status indicates
							 which bits in the
							 POLL_DATA field have
							 changed since the
							 MIF_STATUS reg was
							 last read */
#define    MIF_STATUS_POLL_STATUS_SHIFT    0

/* 7-bit register has current state for all state machines in the MIF */
#define  REG_MIF_STATE_MACHINE             0x621C /* MIF state machine reg */
#define    MIF_SM_CONTROL_MASK             0x07   /* control state machine
						     state */
#define    MIF_SM_EXECUTION_MASK           0x60   /* execution state machine
						     state */


#define  REG_PCS_MII_CTRL                  0x9000 /* PCS MII control reg */
#define    PCS_MII_CTRL_1000_SEL           0x0040 /* reads 1. ignored on
						     writes */
#define    PCS_MII_CTRL_COLLISION_TEST     0x0080 /* COL signal at the PCS
						     to MAC interface is
						     activated regardless
						     of activity */
#define    PCS_MII_CTRL_DUPLEX             0x0100 /* forced 0x0. PCS
						     behaviour same for
						     half and full dplx */
#define    PCS_MII_RESTART_AUTONEG         0x0200 /* self clearing.
						     restart auto-
						     negotiation */
#define    PCS_MII_ISOLATE                 0x0400 /* read as 0. ignored
						     on writes */
#define    PCS_MII_POWER_DOWN              0x0800 /* read as 0. ignored
						     on writes */
#define    PCS_MII_AUTONEG_EN              0x1000 /* default 1. PCS goes
						     through automatic
						     link config before it
						     can be used. when 0,
						     link can be used
						     w/out any link config
						     phase */
#define    PCS_MII_10_100_SEL              0x2000 /* read as 0. ignored on
						     writes */
#define    PCS_MII_RESET                   0x8000 /* reset PCS. self-clears
						     when done */

/* DEFAULT: 0x0108 */
#define  REG_PCS_MII_STATUS                0x9004 /* PCS MII status reg */
#define    PCS_MII_STATUS_EXTEND_CAP       0x0001 /* reads 0 */
#define    PCS_MII_STATUS_JABBER_DETECT    0x0002 /* reads 0 */
#define    PCS_MII_STATUS_LINK_STATUS      0x0004 /* 1 -> link up.
						     0 -> link down. 0 is
						     latched so that 0 is
						     kept until read. read
						     2x to determine if the
						     link has gone up again */
#define    PCS_MII_STATUS_AUTONEG_ABLE     0x0008 /* reads 1 (able to perform
						     auto-neg) */
#define    PCS_MII_STATUS_REMOTE_FAULT     0x0010 /* 1 -> remote fault detected
						     from received link code
						     word. only valid after
						     auto-neg completed */
#define    PCS_MII_STATUS_AUTONEG_COMP     0x0020 /* 1 -> auto-negotiation
						          completed
						     0 -> auto-negotiation not
						     completed */
#define    PCS_MII_STATUS_EXTEND_STATUS    0x0100 /* reads as 1. used as an
						     indication that this is
						     a 1000 Base-X PHY. writes
						     to it are ignored */

#define  REG_PCS_MII_ADVERT                0x9008 /* PCS MII advertisement
						     reg */
#define    PCS_MII_ADVERT_FD               0x0020  /* advertise full duplex
						      1000 Base-X */
#define    PCS_MII_ADVERT_HD               0x0040  /* advertise half-duplex
						      1000 Base-X */
#define    PCS_MII_ADVERT_SYM_PAUSE        0x0080  /* advertise PAUSE
						      symmetric capability */
#define    PCS_MII_ADVERT_ASYM_PAUSE       0x0100  /* advertises PAUSE
						      asymmetric capability */
#define    PCS_MII_ADVERT_RF_MASK          0x3000 /* remote fault. write bit13
						     to optionally indicate to
						     link partner that chip is
						     going off-line. bit12 will
						     get set when signal
						     detect == FAIL and will
						     remain set until
						     successful negotiation */
#define    PCS_MII_ADVERT_ACK              0x4000 /* (ro) */
#define    PCS_MII_ADVERT_NEXT_PAGE        0x8000 /* (ro) forced 0x0 */

#define  REG_PCS_MII_LPA                   0x900C /* PCS MII link partner
						     ability reg */
#define    PCS_MII_LPA_FD             PCS_MII_ADVERT_FD
#define    PCS_MII_LPA_HD             PCS_MII_ADVERT_HD
#define    PCS_MII_LPA_SYM_PAUSE      PCS_MII_ADVERT_SYM_PAUSE
#define    PCS_MII_LPA_ASYM_PAUSE     PCS_MII_ADVERT_ASYM_PAUSE
#define    PCS_MII_LPA_RF_MASK        PCS_MII_ADVERT_RF_MASK
#define    PCS_MII_LPA_ACK            PCS_MII_ADVERT_ACK
#define    PCS_MII_LPA_NEXT_PAGE      PCS_MII_ADVERT_NEXT_PAGE

/* DEFAULT: 0x0 */
#define  REG_PCS_CFG                       0x9010 /* PCS config reg */
#define    PCS_CFG_EN                      0x01   /* enable PCS. must be
						     0 when modifying
						     PCS_MII_ADVERT */
#define    PCS_CFG_SD_OVERRIDE             0x02   /* sets signal detect to
						     OK. bit is
						     non-resettable */
#define    PCS_CFG_SD_ACTIVE_LOW           0x04   /* changes interpretation
						     of optical signal to make
						     signal detect okay when
						     signal is low */
#define    PCS_CFG_JITTER_STUDY_MASK       0x18   /* used to make jitter
						     measurements. a single
						     code group is xmitted
						     regularly.
						     0x0 = normal operation
						     0x1 = high freq test
						           pattern, D21.5
						     0x2 = low freq test
						           pattern, K28.7
						     0x3 = reserved */
#define    PCS_CFG_10MS_TIMER_OVERRIDE     0x20   /* shortens 10-20ms auto-
						     negotiation timer to
						     a few cycles for test
						     purposes */

/* used for diagnostic purposes. bits 20-22 autoclear on read */
#define  REG_PCS_STATE_MACHINE             0x9014 /* (ro) PCS state machine
						     and diagnostic reg */
#define    PCS_SM_TX_STATE_MASK            0x0000000F /* 0 and 1 indicate
							 xmission of idle.
							 otherwise, xmission of
							 a packet */
#define    PCS_SM_RX_STATE_MASK            0x000000F0 /* 0 indicates reception
							 of idle. otherwise,
							 reception of packet */
#define    PCS_SM_WORD_SYNC_STATE_MASK     0x00000700 /* 0 indicates loss of
							 sync */
#define    PCS_SM_SEQ_DETECT_STATE_MASK    0x00001800 /* cycling through 0-3
							 indicates reception of
							 Config codes. cycling
							 through 0-1 indicates
							 reception of idles */
#define    PCS_SM_LINK_STATE_MASK          0x0001E000
#define        SM_LINK_STATE_UP            0x00016000 /* link state is up */

#define    PCS_SM_LOSS_LINK_C              0x00100000 /* loss of link due to
							 recept of Config
							 codes */
#define    PCS_SM_LOSS_LINK_SYNC           0x00200000 /* loss of link due to
							 loss of sync */
#define    PCS_SM_LOSS_SIGNAL_DETECT       0x00400000 /* signal detect goes
							 from OK to FAIL. bit29
							 will also be set if
							 this is set */
#define    PCS_SM_NO_LINK_BREAKLINK        0x01000000 /* link not up due to
							receipt of breaklink
							C codes from partner.
							C codes w/ 0 content
							received triggering
							start/restart of
							autonegotiation.
							should be sent for
							no longer than 20ms */
#define    PCS_SM_NO_LINK_SERDES           0x02000000 /* serdes being
							initialized. see serdes
							state reg */
#define    PCS_SM_NO_LINK_C                0x04000000 /* C codes not stable or
							 not received */
#define    PCS_SM_NO_LINK_SYNC             0x08000000 /* word sync not
							 achieved */
#define    PCS_SM_NO_LINK_WAIT_C           0x10000000 /* waiting for C codes
							 w/ ack bit set */
#define    PCS_SM_NO_LINK_NO_IDLE          0x20000000 /* link partner continues
							 to send C codes
							 instead of idle
							 symbols or pkt data */

#define  REG_PCS_INTR_STATUS               0x9018 /* PCS interrupt status */
#define    PCS_INTR_STATUS_LINK_CHANGE     0x04   /* link status has changed
						     since last read */

#define  REG_PCS_DATAPATH_MODE             0x9050 /* datapath mode reg */
#define    PCS_DATAPATH_MODE_MII           0x00 /* PCS is not used and
						   MII/GMII is selected.
						   selection between MII and
						   GMII is controlled by
						   XIF_CFG */
#define    PCS_DATAPATH_MODE_SERDES        0x02 /* PCS is used via the
						   10-bit interface */

/* input to serdes chip or serialink block */
#define  REG_PCS_SERDES_CTRL              0x9054 /* serdes control reg */
#define    PCS_SERDES_CTRL_LOOPBACK       0x01   /* enable loopback on
						    serdes interface */
#define    PCS_SERDES_CTRL_SYNCD_EN       0x02   /* enable sync carrier
						    detection. should be
						    0x0 for normal
						    operation */
#define    PCS_SERDES_CTRL_LOCKREF       0x04   /* frequency-lock RBC[0:1]
						   to REFCLK when set.
						   when clear, receiver
						   clock locks to incoming
						   serial data */

#define  REG_PCS_SHARED_OUTPUT_SEL         0x9058 /* shared output select */
#define    PCS_SOS_PROM_ADDR_MASK          0x0007

#define  REG_PCS_SERDES_STATE              0x905C /* (ro) serdes state */
#define    PCS_SERDES_STATE_MASK           0x03

#define  REG_PCS_PACKET_COUNT              0x9060 /* (ro) PCS packet counter */
#define    PCS_PACKET_COUNT_TX             0x000007FF /* pkts xmitted by PCS */
#define    PCS_PACKET_COUNT_RX             0x07FF0000 /* pkts recvd by PCS
							 whether they
							 encountered an error
							 or not */

#define  REG_EXPANSION_ROM_RUN_START       0x100000 /* expansion rom run time
						       access */
#define  REG_EXPANSION_ROM_RUN_END         0x17FFFF

#define  REG_SECOND_LOCALBUS_START         0x180000 /* secondary local bus
						       device */
#define  REG_SECOND_LOCALBUS_END           0x1FFFFF

/* entropy device */
#define  REG_ENTROPY_START                 REG_SECOND_LOCALBUS_START
#define  REG_ENTROPY_DATA                  (REG_ENTROPY_START + 0x00)
#define  REG_ENTROPY_STATUS                (REG_ENTROPY_START + 0x04)
#define      ENTROPY_STATUS_DRDY           0x01
#define      ENTROPY_STATUS_BUSY           0x02
#define      ENTROPY_STATUS_CIPHER         0x04
#define      ENTROPY_STATUS_BYPASS_MASK    0x18
#define  REG_ENTROPY_MODE                  (REG_ENTROPY_START + 0x05)
#define      ENTROPY_MODE_KEY_MASK         0x07
#define      ENTROPY_MODE_ENCRYPT          0x40
#define  REG_ENTROPY_RAND_REG              (REG_ENTROPY_START + 0x06)
#define  REG_ENTROPY_RESET                 (REG_ENTROPY_START + 0x07)
#define      ENTROPY_RESET_DES_IO          0x01
#define      ENTROPY_RESET_STC_MODE        0x02
#define      ENTROPY_RESET_KEY_CACHE       0x04
#define      ENTROPY_RESET_IV              0x08
#define  REG_ENTROPY_IV                    (REG_ENTROPY_START + 0x08)
#define  REG_ENTROPY_KEY0                  (REG_ENTROPY_START + 0x10)
#define  REG_ENTROPY_KEYN(x)               (REG_ENTROPY_KEY0 + 4*(x))

/* phys of interest w/ their special mii registers */
#define PHY_LUCENT_B0     0x00437421
#define   LUCENT_MII_REG      0x1F

#define PHY_NS_DP83065    0x20005c78
#define   DP83065_MII_MEM     0x16
#define   DP83065_MII_REGD    0x1D
#define   DP83065_MII_REGE    0x1E

#define PHY_BROADCOM_5411 0x00206071
#define PHY_BROADCOM_B0   0x00206050
#define   BROADCOM_MII_REG4   0x14
#define   BROADCOM_MII_REG5   0x15
#define   BROADCOM_MII_REG7   0x17
#define   BROADCOM_MII_REG8   0x18

#define   CAS_MII_ANNPTR          0x07
#define   CAS_MII_ANNPRR          0x08
#define   CAS_MII_1000_CTRL       0x09
#define   CAS_MII_1000_STATUS     0x0A
#define   CAS_MII_1000_EXTEND     0x0F

#define   CAS_BMSR_1000_EXTEND    0x0100 /* supports 1000Base-T extended status */
#define   CAS_BMCR_SPEED1000      0x0040  /* Select 1000Mbps */

#define   CAS_ADVERTISE_1000HALF   0x0100
#define   CAS_ADVERTISE_1000FULL   0x0200
#define   CAS_ADVERTISE_PAUSE      0x0400
#define   CAS_ADVERTISE_ASYM_PAUSE 0x0800

/* regular lpa register */
#define   CAS_LPA_PAUSE	           CAS_ADVERTISE_PAUSE
#define   CAS_LPA_ASYM_PAUSE       CAS_ADVERTISE_ASYM_PAUSE

/* 1000_STATUS register */
#define   CAS_LPA_1000HALF        0x0400
#define   CAS_LPA_1000FULL        0x0800

#define   CAS_EXTEND_1000XFULL    0x8000
#define   CAS_EXTEND_1000XHALF    0x4000
#define   CAS_EXTEND_1000TFULL    0x2000
#define   CAS_EXTEND_1000THALF    0x1000

/* cassini header parser firmware */
typedef struct cas_hp_inst {
	const char *note;

	u16 mask, val;

	u8 op;
	u8 soff, snext;	/* if match succeeds, new offset and match */
	u8 foff, fnext;	/* if match fails, new offset and match */
	/* output info */
	u8 outop;    /* output opcode */

	u16 outarg;  /* output argument */
	u8 outenab;  /* output enable: 0 = not, 1 = if match
			 2 = if !match, 3 = always */
	u8 outshift; /* barrel shift right, 4 bits */
	u16 outmask;
} cas_hp_inst_t;

/* comparison */
#define OP_EQ     0 /* packet == value */
#define OP_LT     1 /* packet < value */
#define OP_GT     2 /* packet > value */
#define OP_NP     3 /* new packet */

/* output opcodes */
#define	CL_REG	0
#define	LD_FID	1
#define	LD_SEQ	2
#define	LD_CTL	3
#define	LD_SAP	4
#define	LD_R1	5
#define	LD_L3	6
#define	LD_SUM	7
#define	LD_HDR	8
#define	IM_FID	9
#define	IM_SEQ	10
#define	IM_SAP	11
#define	IM_R1	12
#define	IM_CTL	13
#define	LD_LEN	14
#define	ST_FLG	15

/* match setp #s for IP4TCP4 */
#define S1_PCKT         0
#define S1_VLAN         1
#define S1_CFI          2
#define S1_8023         3
#define S1_LLC          4
#define S1_LLCc         5
#define S1_IPV4         6
#define S1_IPV4c        7
#define S1_IPV4F        8
#define S1_TCP44        9
#define S1_IPV6         10
#define S1_IPV6L        11
#define S1_IPV6c        12
#define S1_TCP64        13
#define S1_TCPSQ        14
#define S1_TCPFG        15
#define	S1_TCPHL	16
#define	S1_TCPHc	17
#define	S1_CLNP		18
#define	S1_CLNP2	19
#define	S1_DROP		20
#define	S2_HTTP		21
#define	S1_ESP4		22
#define	S1_AH4		23
#define	S1_ESP6		24
#define	S1_AH6		25

#define CAS_PROG_IP46TCP4_PREAMBLE \
{ "packet arrival?", 0xffff, 0x0000, OP_NP,  6, S1_VLAN,  0, S1_PCKT,  \
  CL_REG, 0x3ff, 1, 0x0, 0x0000}, \
{ "VLAN?", 0xffff, 0x8100, OP_EQ,  1, S1_CFI,   0, S1_8023,  \
  IM_CTL, 0x00a,  3, 0x0, 0xffff}, \
{ "CFI?", 0x1000, 0x1000, OP_EQ,  0, S1_DROP,  1, S1_8023, \
  CL_REG, 0x000,  0, 0x0, 0x0000}, \
{ "8023?", 0xffff, 0x0600, OP_LT,  1, S1_LLC,   0, S1_IPV4, \
  CL_REG, 0x000,  0, 0x0, 0x0000}, \
{ "LLC?", 0xffff, 0xaaaa, OP_EQ,  1, S1_LLCc,  0, S1_CLNP, \
  CL_REG, 0x000,  0, 0x0, 0x0000}, \
{ "LLCc?", 0xff00, 0x0300, OP_EQ,  2, S1_IPV4,  0, S1_CLNP, \
  CL_REG, 0x000,  0, 0x0, 0x0000}, \
{ "IPV4?", 0xffff, 0x0800, OP_EQ,  1, S1_IPV4c, 0, S1_IPV6, \
  LD_SAP, 0x100,  3, 0x0, 0xffff}, \
{ "IPV4 cont?", 0xff00, 0x4500, OP_EQ,  3, S1_IPV4F, 0, S1_CLNP, \
  LD_SUM, 0x00a,  1, 0x0, 0x0000}, \
{ "IPV4 frag?", 0x3fff, 0x0000, OP_EQ,  1, S1_TCP44, 0, S1_CLNP, \
  LD_LEN, 0x03e,  1, 0x0, 0xffff}, \
{ "TCP44?", 0x00ff, 0x0006, OP_EQ,  7, S1_TCPSQ, 0, S1_CLNP, \
  LD_FID, 0x182,  1, 0x0, 0xffff}, /* FID IP4&TCP src+dst */ \
{ "IPV6?", 0xffff, 0x86dd, OP_EQ,  1, S1_IPV6L, 0, S1_CLNP,  \
  LD_SUM, 0x015,  1, 0x0, 0x0000}, \
{ "IPV6 len", 0xf000, 0x6000, OP_EQ,  0, S1_IPV6c, 0, S1_CLNP, \
  IM_R1,  0x128,  1, 0x0, 0xffff}, \
{ "IPV6 cont?", 0x0000, 0x0000, OP_EQ,  3, S1_TCP64, 0, S1_CLNP, \
  LD_FID, 0x484,  1, 0x0, 0xffff}, /* FID IP6&TCP src+dst */ \
{ "TCP64?", 0xff00, 0x0600, OP_EQ, 18, S1_TCPSQ, 0, S1_CLNP, \
  LD_LEN, 0x03f,  1, 0x0, 0xffff}

#ifdef USE_HP_IP46TCP4
static cas_hp_inst_t cas_prog_ip46tcp4tab[] = {
	CAS_PROG_IP46TCP4_PREAMBLE,
	{ "TCP seq", /* DADDR should point to dest port */
	  0x0000, 0x0000, OP_EQ, 0, S1_TCPFG, 4, S1_TCPFG, LD_SEQ,
	  0x081,  3, 0x0, 0xffff}, /* Load TCP seq # */
	{ "TCP control flags", 0x0000, 0x0000, OP_EQ,  0, S1_TCPHL, 0,
	  S1_TCPHL, ST_FLG, 0x045,  3, 0x0, 0x002f}, /* Load TCP flags */
	{ "TCP length", 0x0000, 0x0000, OP_EQ,  0, S1_TCPHc, 0,
	  S1_TCPHc, LD_R1,  0x205,  3, 0xB, 0xf000},
	{ "TCP length cont", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0,
	  S1_PCKT,  LD_HDR, 0x0ff,  3, 0x0, 0xffff},
	{ "Cleanup", 0x0000, 0x0000, OP_EQ,  0, S1_CLNP2,  0, S1_CLNP2,
	  IM_CTL, 0x001,  3, 0x0, 0x0001},
	{ "Cleanup 2", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0, S1_PCKT,
	  IM_CTL, 0x000,  0, 0x0, 0x0000},
	{ "Drop packet", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0, S1_PCKT,
	  IM_CTL, 0x080,  3, 0x0, 0xffff},
	{ NULL },
};
#ifdef HP_IP46TCP4_DEFAULT
#define CAS_HP_FIRMWARE               cas_prog_ip46tcp4tab
#endif
#endif

#ifdef USE_HP_IP46TCP4NOHTTP
static cas_hp_inst_t cas_prog_ip46tcp4nohttptab[] = {
	CAS_PROG_IP46TCP4_PREAMBLE,
	{ "TCP seq", /* DADDR should point to dest port */
	  0xFFFF, 0x0080, OP_EQ,  0, S2_HTTP,  0, S1_TCPFG, LD_SEQ,
	  0x081,  3, 0x0, 0xffff} , /* Load TCP seq # */
	{ "TCP control flags", 0xFFFF, 0x8080, OP_EQ,  0, S2_HTTP,  0,
	  S1_TCPHL, ST_FLG, 0x145,  2, 0x0, 0x002f, }, /* Load TCP flags */
	{ "TCP length", 0x0000, 0x0000, OP_EQ,  0, S1_TCPHc, 0, S1_TCPHc,
	  LD_R1,  0x205,  3, 0xB, 0xf000},
	{ "TCP length cont", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0, S1_PCKT,
	  LD_HDR, 0x0ff,  3, 0x0, 0xffff},
	{ "Cleanup", 0x0000, 0x0000, OP_EQ,  0, S1_CLNP2,  0, S1_CLNP2,
	  IM_CTL, 0x001,  3, 0x0, 0x0001},
	{ "Cleanup 2", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0, S1_PCKT,
	  CL_REG, 0x002,  3, 0x0, 0x0000},
	{ "Drop packet", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0, S1_PCKT,
	  IM_CTL, 0x080,  3, 0x0, 0xffff},
	{ "No HTTP", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0, S1_PCKT,
	  IM_CTL, 0x044,  3, 0x0, 0xffff},
	{ NULL },
};
#ifdef HP_IP46TCP4NOHTTP_DEFAULT
#define CAS_HP_FIRMWARE               cas_prog_ip46tcp4nohttptab
#endif
#endif

/* match step #s for IP4FRAG */
#define	S3_IPV6c	11
#define	S3_TCP64	12
#define	S3_TCPSQ	13
#define	S3_TCPFG	14
#define	S3_TCPHL	15
#define	S3_TCPHc	16
#define	S3_FRAG		17
#define	S3_FOFF		18
#define	S3_CLNP		19

#ifdef USE_HP_IP4FRAG
static cas_hp_inst_t cas_prog_ip4fragtab[] = {
	{ "packet arrival?", 0xffff, 0x0000, OP_NP,  6, S1_VLAN,  0, S1_PCKT,
	  CL_REG, 0x3ff, 1, 0x0, 0x0000},
	{ "VLAN?", 0xffff, 0x8100, OP_EQ,  1, S1_CFI,   0, S1_8023,
	  IM_CTL, 0x00a,  3, 0x0, 0xffff},
	{ "CFI?", 0x1000, 0x1000, OP_EQ,  0, S3_CLNP,  1, S1_8023,
	  CL_REG, 0x000,  0, 0x0, 0x0000},
	{ "8023?", 0xffff, 0x0600, OP_LT,  1, S1_LLC,   0, S1_IPV4,
	  CL_REG, 0x000,  0, 0x0, 0x0000},
	{ "LLC?", 0xffff, 0xaaaa, OP_EQ,  1, S1_LLCc,  0, S3_CLNP,
	  CL_REG, 0x000,  0, 0x0, 0x0000},
	{ "LLCc?",0xff00, 0x0300, OP_EQ,  2, S1_IPV4,  0, S3_CLNP,
	  CL_REG, 0x000,  0, 0x0, 0x0000},
	{ "IPV4?", 0xffff, 0x0800, OP_EQ,  1, S1_IPV4c, 0, S1_IPV6,
	  LD_SAP, 0x100,  3, 0x0, 0xffff},
	{ "IPV4 cont?", 0xff00, 0x4500, OP_EQ,  3, S1_IPV4F, 0, S3_CLNP,
	  LD_SUM, 0x00a,  1, 0x0, 0x0000},
	{ "IPV4 frag?", 0x3fff, 0x0000, OP_EQ,  1, S1_TCP44, 0, S3_FRAG,
	  LD_LEN, 0x03e,  3, 0x0, 0xffff},
	{ "TCP44?", 0x00ff, 0x0006, OP_EQ,  7, S3_TCPSQ, 0, S3_CLNP,
	  LD_FID, 0x182,  3, 0x0, 0xffff}, /* FID IP4&TCP src+dst */
	{ "IPV6?", 0xffff, 0x86dd, OP_EQ,  1, S3_IPV6c, 0, S3_CLNP,
	  LD_SUM, 0x015,  1, 0x0, 0x0000},
	{ "IPV6 cont?", 0xf000, 0x6000, OP_EQ,  3, S3_TCP64, 0, S3_CLNP,
	  LD_FID, 0x484,  1, 0x0, 0xffff}, /* FID IP6&TCP src+dst */
	{ "TCP64?", 0xff00, 0x0600, OP_EQ, 18, S3_TCPSQ, 0, S3_CLNP,
	  LD_LEN, 0x03f,  1, 0x0, 0xffff},
	{ "TCP seq",	/* DADDR should point to dest port */
	  0x0000, 0x0000, OP_EQ,  0, S3_TCPFG, 4, S3_TCPFG, LD_SEQ,
	  0x081,  3, 0x0, 0xffff}, /* Load TCP seq # */
	{ "TCP control flags", 0x0000, 0x0000, OP_EQ,  0, S3_TCPHL, 0,
	  S3_TCPHL, ST_FLG, 0x045,  3, 0x0, 0x002f}, /* Load TCP flags */
	{ "TCP length", 0x0000, 0x0000, OP_EQ,  0, S3_TCPHc, 0, S3_TCPHc,
	  LD_R1,  0x205,  3, 0xB, 0xf000},
	{ "TCP length cont", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0, S1_PCKT,
	  LD_HDR, 0x0ff,  3, 0x0, 0xffff},
	{ "IP4 Fragment", 0x0000, 0x0000, OP_EQ,  0, S3_FOFF,  0, S3_FOFF,
	  LD_FID, 0x103,  3, 0x0, 0xffff}, /* FID IP4 src+dst */
	{ "IP4 frag offset", 0x0000, 0x0000, OP_EQ,  0, S3_FOFF,  0, S3_FOFF,
	  LD_SEQ, 0x040,  1, 0xD, 0xfff8},
	{ "Cleanup", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0, S1_PCKT,
	  IM_CTL, 0x001,  3, 0x0, 0x0001},
	{ NULL },
};
#ifdef HP_IP4FRAG_DEFAULT
#define CAS_HP_FIRMWARE               cas_prog_ip4fragtab
#endif
#endif

#ifdef USE_HP_IP46TCP4BATCH
static cas_hp_inst_t cas_prog_ip46tcp4batchtab[] = {
	CAS_PROG_IP46TCP4_PREAMBLE,
	{ "TCP seq",	/* DADDR should point to dest port */
	  0x0000, 0x0000, OP_EQ,  0, S1_TCPFG, 0, S1_TCPFG, LD_SEQ,
	  0x081,  3, 0x0, 0xffff}, /* Load TCP seq # */
	{ "TCP control flags", 0x0000, 0x0000, OP_EQ,  0, S1_TCPHL, 0,
	  S1_TCPHL, ST_FLG, 0x000,  3, 0x0, 0x0000}, /* Load TCP flags */
	{ "TCP length", 0x0000, 0x0000, OP_EQ,  0, S1_TCPHc, 0,
	  S1_TCPHc, LD_R1,  0x205,  3, 0xB, 0xf000},
	{ "TCP length cont", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0,
	  S1_PCKT,  IM_CTL, 0x040,  3, 0x0, 0xffff}, /* set batch bit */
	{ "Cleanup", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0, S1_PCKT,
	  IM_CTL, 0x001,  3, 0x0, 0x0001},
	{ "Drop packet", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0,
	  S1_PCKT,  IM_CTL, 0x080,  3, 0x0, 0xffff},
	{ NULL },
};
#ifdef HP_IP46TCP4BATCH_DEFAULT
#define CAS_HP_FIRMWARE               cas_prog_ip46tcp4batchtab
#endif
#endif

#ifdef USE_HP_WORKAROUND
static cas_hp_inst_t  cas_prog_workaroundtab[] = {
	{ "packet arrival?", 0xffff, 0x0000, OP_NP,  6, S1_VLAN,  0,
	  S1_PCKT,  CL_REG, 0x3ff,  1, 0x0, 0x0000} ,
	{ "VLAN?", 0xffff, 0x8100, OP_EQ,  1, S1_CFI, 0, S1_8023,
	  IM_CTL, 0x04a,  3, 0x0, 0xffff},
	{ "CFI?", 0x1000, 0x1000, OP_EQ,  0, S1_CLNP,  1, S1_8023,
	  CL_REG, 0x000,  0, 0x0, 0x0000},
	{ "8023?", 0xffff, 0x0600, OP_LT,  1, S1_LLC,   0, S1_IPV4,
	  CL_REG, 0x000,  0, 0x0, 0x0000},
	{ "LLC?", 0xffff, 0xaaaa, OP_EQ,  1, S1_LLCc,  0, S1_CLNP,
	  CL_REG, 0x000,  0, 0x0, 0x0000},
	{ "LLCc?", 0xff00, 0x0300, OP_EQ,  2, S1_IPV4,  0, S1_CLNP,
	  CL_REG, 0x000,  0, 0x0, 0x0000},
	{ "IPV4?", 0xffff, 0x0800, OP_EQ,  1, S1_IPV4c, 0, S1_IPV6,
	  IM_SAP, 0x6AE,  3, 0x0, 0xffff},
	{ "IPV4 cont?", 0xff00, 0x4500, OP_EQ,  3, S1_IPV4F, 0, S1_CLNP,
	  LD_SUM, 0x00a,  1, 0x0, 0x0000},
	{ "IPV4 frag?", 0x3fff, 0x0000, OP_EQ,  1, S1_TCP44, 0, S1_CLNP,
	  LD_LEN, 0x03e,  1, 0x0, 0xffff},
	{ "TCP44?", 0x00ff, 0x0006, OP_EQ,  7, S1_TCPSQ, 0, S1_CLNP,
	  LD_FID, 0x182,  3, 0x0, 0xffff}, /* FID IP4&TCP src+dst */
	{ "IPV6?", 0xffff, 0x86dd, OP_EQ,  1, S1_IPV6L, 0, S1_CLNP,
	  LD_SUM, 0x015,  1, 0x0, 0x0000},
	{ "IPV6 len", 0xf000, 0x6000, OP_EQ,  0, S1_IPV6c, 0, S1_CLNP,
	  IM_R1,  0x128,  1, 0x0, 0xffff},
	{ "IPV6 cont?", 0x0000, 0x0000, OP_EQ,  3, S1_TCP64, 0, S1_CLNP,
	  LD_FID, 0x484,  1, 0x0, 0xffff}, /* FID IP6&TCP src+dst */
	{ "TCP64?", 0xff00, 0x0600, OP_EQ, 18, S1_TCPSQ, 0, S1_CLNP,
	  LD_LEN, 0x03f,  1, 0x0, 0xffff},
	{ "TCP seq",      /* DADDR should point to dest port */
	  0x0000, 0x0000, OP_EQ,  0, S1_TCPFG, 4, S1_TCPFG, LD_SEQ,
	  0x081,  3, 0x0, 0xffff}, /* Load TCP seq # */
	{ "TCP control flags", 0x0000, 0x0000, OP_EQ,  0, S1_TCPHL, 0,
	  S1_TCPHL, ST_FLG, 0x045,  3, 0x0, 0x002f}, /* Load TCP flags */
	{ "TCP length", 0x0000, 0x0000, OP_EQ,  0, S1_TCPHc, 0, S1_TCPHc,
	  LD_R1,  0x205,  3, 0xB, 0xf000},
	{ "TCP length cont", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0,
	  S1_PCKT,  LD_HDR, 0x0ff,  3, 0x0, 0xffff},
	{ "Cleanup", 0x0000, 0x0000, OP_EQ,  0, S1_CLNP2, 0, S1_CLNP2,
	  IM_SAP, 0x6AE,  3, 0x0, 0xffff} ,
	{ "Cleanup 2", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0, S1_PCKT,
	  IM_CTL, 0x001,  3, 0x0, 0x0001},
	{ NULL },
};
#ifdef HP_WORKAROUND_DEFAULT
#define CAS_HP_FIRMWARE               cas_prog_workaroundtab
#endif
#endif

#ifdef USE_HP_ENCRYPT
static cas_hp_inst_t  cas_prog_encryptiontab[] = {
	{ "packet arrival?", 0xffff, 0x0000, OP_NP,  6, S1_VLAN,  0,
	  S1_PCKT,  CL_REG, 0x3ff,  1, 0x0, 0x0000},
	{ "VLAN?", 0xffff, 0x8100, OP_EQ,  1, S1_CFI,   0, S1_8023,
	  IM_CTL, 0x00a,  3, 0x0, 0xffff},
#if 0
//"CFI?", /* 02 FIND CFI and If FIND go to S1_DROP */
//0x1000, 0x1000, OP_EQ,  0, S1_DROP,  1, S1_8023,  CL_REG, 0x000,  0, 0x0, 0x00
	00,
#endif
	{ "CFI?", /* FIND CFI and If FIND go to CleanUP1 (ignore and send to host) */
	  0x1000, 0x1000, OP_EQ,  0, S1_CLNP,  1, S1_8023,
	  CL_REG, 0x000,  0, 0x0, 0x0000},
	{ "8023?", 0xffff, 0x0600, OP_LT,  1, S1_LLC,   0, S1_IPV4,
	  CL_REG, 0x000,  0, 0x0, 0x0000},
	{ "LLC?", 0xffff, 0xaaaa, OP_EQ,  1, S1_LLCc,  0, S1_CLNP,
	  CL_REG, 0x000,  0, 0x0, 0x0000},
	{ "LLCc?", 0xff00, 0x0300, OP_EQ,  2, S1_IPV4,  0, S1_CLNP,
	  CL_REG, 0x000,  0, 0x0, 0x0000},
	{ "IPV4?", 0xffff, 0x0800, OP_EQ,  1, S1_IPV4c, 0, S1_IPV6,
	  LD_SAP, 0x100,  3, 0x0, 0xffff},
	{ "IPV4 cont?", 0xff00, 0x4500, OP_EQ,  3, S1_IPV4F, 0, S1_CLNP,
	  LD_SUM, 0x00a,  1, 0x0, 0x0000},
	{ "IPV4 frag?", 0x3fff, 0x0000, OP_EQ,  1, S1_TCP44, 0, S1_CLNP,
	  LD_LEN, 0x03e,  1, 0x0, 0xffff},
	{ "TCP44?", 0x00ff, 0x0006, OP_EQ,  7, S1_TCPSQ, 0, S1_ESP4,
	  LD_FID, 0x182,  1, 0x0, 0xffff}, /* FID IP4&TCP src+dst */
	{ "IPV6?", 0xffff, 0x86dd, OP_EQ,  1, S1_IPV6L, 0, S1_CLNP,
	  LD_SUM, 0x015,  1, 0x0, 0x0000},
	{ "IPV6 len", 0xf000, 0x6000, OP_EQ,  0, S1_IPV6c, 0, S1_CLNP,
	  IM_R1,  0x128,  1, 0x0, 0xffff},
	{ "IPV6 cont?", 0x0000, 0x0000, OP_EQ,  3, S1_TCP64, 0, S1_CLNP,
	  LD_FID, 0x484,  1, 0x0, 0xffff}, /*  FID IP6&TCP src+dst */
	{ "TCP64?",
#if 0
//@@@0xff00, 0x0600, OP_EQ, 18, S1_TCPSQ, 0, S1_ESP6,  LD_LEN, 0x03f,  1, 0x0, 0xffff,
#endif
	  0xff00, 0x0600, OP_EQ, 12, S1_TCPSQ, 0, S1_ESP6,  LD_LEN,
	  0x03f,  1, 0x0, 0xffff},
	{ "TCP seq", /* 14:DADDR should point to dest port */
	  0xFFFF, 0x0080, OP_EQ,  0, S2_HTTP,  0, S1_TCPFG, LD_SEQ,
	  0x081,  3, 0x0, 0xffff}, /* Load TCP seq # */
	{ "TCP control flags", 0xFFFF, 0x8080, OP_EQ,  0, S2_HTTP,  0,
	  S1_TCPHL, ST_FLG, 0x145,  2, 0x0, 0x002f}, /* Load TCP flags */
	{ "TCP length", 0x0000, 0x0000, OP_EQ,  0, S1_TCPHc, 0, S1_TCPHc,
	  LD_R1,  0x205,  3, 0xB, 0xf000} ,
	{ "TCP length cont", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0,
	  S1_PCKT,  LD_HDR, 0x0ff,  3, 0x0, 0xffff},
	{ "Cleanup", 0x0000, 0x0000, OP_EQ,  0, S1_CLNP2,  0, S1_CLNP2,
	  IM_CTL, 0x001,  3, 0x0, 0x0001},
	{ "Cleanup 2", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0, S1_PCKT,
	  CL_REG, 0x002,  3, 0x0, 0x0000},
	{ "Drop packet", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0, S1_PCKT,
	  IM_CTL, 0x080,  3, 0x0, 0xffff},
	{ "No HTTP", 0x0000, 0x0000, OP_EQ,  0, S1_PCKT,  0, S1_PCKT,
	  IM_CTL, 0x044,  3, 0x0, 0xffff},
	{ "IPV4 ESP encrypted?",  /* S1_ESP4 */
	  0x00ff, 0x0032, OP_EQ,  0, S1_CLNP2, 0, S1_AH4, IM_CTL,
	  0x021, 1,  0x0, 0xffff},
	{ "IPV4 AH encrypted?",   /* S1_AH4 */
	  0x00ff, 0x0033, OP_EQ,  0, S1_CLNP2, 0, S1_CLNP, IM_CTL,
	  0x021, 1,  0x0, 0xffff},
	{ "IPV6 ESP encrypted?",  /* S1_ESP6 */
#if 0
//@@@0x00ff, 0x0032, OP_EQ,  0, S1_CLNP2, 0, S1_AH6, IM_CTL, 0x021, 1,  0x0, 0xffff,
#endif
	  0xff00, 0x3200, OP_EQ,  0, S1_CLNP2, 0, S1_AH6, IM_CTL,
	  0x021, 1,  0x0, 0xffff},
	{ "IPV6 AH encrypted?",   /* S1_AH6 */
#if 0
//@@@0x00ff, 0x0033, OP_EQ,  0, S1_CLNP2, 0, S1_CLNP, IM_CTL, 0x021, 1,  0x0, 0xffff,
#endif
	  0xff00, 0x3300, OP_EQ,  0, S1_CLNP2, 0, S1_CLNP, IM_CTL,
	  0x021, 1,  0x0, 0xffff},
	{ NULL },
};
#ifdef HP_ENCRYPT_DEFAULT
#define CAS_HP_FIRMWARE               cas_prog_encryptiontab
#endif
#endif

static cas_hp_inst_t cas_prog_null[] = { {NULL} };
#ifdef HP_NULL_DEFAULT
#define CAS_HP_FIRMWARE               cas_prog_null
#endif

/* phy types */
#define   CAS_PHY_UNKNOWN       0x00
#define   CAS_PHY_SERDES        0x01
#define   CAS_PHY_MII_MDIO0     0x02
#define   CAS_PHY_MII_MDIO1     0x04
#define   CAS_PHY_MII(x)        ((x) & (CAS_PHY_MII_MDIO0 | CAS_PHY_MII_MDIO1))


#define DESC_RING_I_TO_S(x)  (32*(1 << (x)))
#define COMP_RING_I_TO_S(x)  (128*(1 << (x)))
#define TX_DESC_RING_INDEX 4  /* 512 = 8k */
#define RX_DESC_RING_INDEX 4  /* 512 = 8k */
#define RX_COMP_RING_INDEX 4  /* 2048 = 64k: should be 4x rx ring size */

#if (TX_DESC_RING_INDEX > 8) || (TX_DESC_RING_INDEX < 0)
#error TX_DESC_RING_INDEX must be between 0 and 8
#endif

#if (RX_DESC_RING_INDEX > 8) || (RX_DESC_RING_INDEX < 0)
#error RX_DESC_RING_INDEX must be between 0 and 8
#endif

#if (RX_COMP_RING_INDEX > 8) || (RX_COMP_RING_INDEX < 0)
#error RX_COMP_RING_INDEX must be between 0 and 8
#endif

#define N_TX_RINGS                    MAX_TX_RINGS      /* for QoS */
#define N_TX_RINGS_MASK               MAX_TX_RINGS_MASK
#define N_RX_DESC_RINGS               MAX_RX_DESC_RINGS /* 1 for ipsec */
#define N_RX_COMP_RINGS               0x1 /* for mult. PCI interrupts */

/* number of flows that can go through re-assembly */
#define N_RX_FLOWS                    64

#define TX_DESC_RING_SIZE  DESC_RING_I_TO_S(TX_DESC_RING_INDEX)
#define RX_DESC_RING_SIZE  DESC_RING_I_TO_S(RX_DESC_RING_INDEX)
#define RX_COMP_RING_SIZE  COMP_RING_I_TO_S(RX_COMP_RING_INDEX)
#define TX_DESC_RINGN_INDEX(x) TX_DESC_RING_INDEX
#define RX_DESC_RINGN_INDEX(x) RX_DESC_RING_INDEX
#define RX_COMP_RINGN_INDEX(x) RX_COMP_RING_INDEX
#define TX_DESC_RINGN_SIZE(x)  TX_DESC_RING_SIZE
#define RX_DESC_RINGN_SIZE(x)  RX_DESC_RING_SIZE
#define RX_COMP_RINGN_SIZE(x)  RX_COMP_RING_SIZE

/* convert values */
#define CAS_BASE(x, y)                (((y) << (x ## _SHIFT)) & (x ## _MASK))
#define CAS_VAL(x, y)                 (((y) & (x ## _MASK)) >> (x ## _SHIFT))
#define CAS_TX_RINGN_BASE(y)          ((TX_DESC_RINGN_INDEX(y) << \
                                        TX_CFG_DESC_RINGN_SHIFT(y)) & \
                                        TX_CFG_DESC_RINGN_MASK(y))

/* min is 2k, but we can't do jumbo frames unless it's at least 8k */
#define CAS_MIN_PAGE_SHIFT            11 /* 2048 */
#define CAS_JUMBO_PAGE_SHIFT          13 /* 8192 */
#define CAS_MAX_PAGE_SHIFT            14 /* 16384 */

#define TX_DESC_BUFLEN_MASK         0x0000000000003FFFULL /* buffer length in
							     bytes. 0 - 9256 */
#define TX_DESC_BUFLEN_SHIFT        0
#define TX_DESC_CSUM_START_MASK     0x00000000001F8000ULL /* checksum start. #
							     of bytes to be
							     skipped before
							     csum calc begins.
							     value must be
							     even */
#define TX_DESC_CSUM_START_SHIFT    15
#define TX_DESC_CSUM_STUFF_MASK     0x000000001FE00000ULL /* checksum stuff.
							     byte offset w/in
							     the pkt for the
							     1st csum byte.
							     must be > 8 */
#define TX_DESC_CSUM_STUFF_SHIFT    21
#define TX_DESC_CSUM_EN             0x0000000020000000ULL /* enable checksum */
#define TX_DESC_EOF                 0x0000000040000000ULL /* end of frame */
#define TX_DESC_SOF                 0x0000000080000000ULL /* start of frame */
#define TX_DESC_INTME               0x0000000100000000ULL /* interrupt me */
#define TX_DESC_NO_CRC              0x0000000200000000ULL /* debugging only.
							     CRC will not be
							     inserted into
							     outgoing frame. */
struct cas_tx_desc {
	__le64     control;
	__le64     buffer;
};

struct cas_rx_desc {
	__le64     index;
	__le64     buffer;
};

/* received packets are put on the completion ring. */
/* word 1 */
#define RX_COMP1_DATA_SIZE_MASK           0x0000000007FFE000ULL
#define RX_COMP1_DATA_SIZE_SHIFT          13
#define RX_COMP1_DATA_OFF_MASK            0x000001FFF8000000ULL
#define RX_COMP1_DATA_OFF_SHIFT           27
#define RX_COMP1_DATA_INDEX_MASK          0x007FFE0000000000ULL
#define RX_COMP1_DATA_INDEX_SHIFT         41
#define RX_COMP1_SKIP_MASK                0x0180000000000000ULL
#define RX_COMP1_SKIP_SHIFT               55
#define RX_COMP1_RELEASE_NEXT             0x0200000000000000ULL
#define RX_COMP1_SPLIT_PKT                0x0400000000000000ULL
#define RX_COMP1_RELEASE_FLOW             0x0800000000000000ULL
#define RX_COMP1_RELEASE_DATA             0x1000000000000000ULL
#define RX_COMP1_RELEASE_HDR              0x2000000000000000ULL
#define RX_COMP1_TYPE_MASK                0xC000000000000000ULL
#define RX_COMP1_TYPE_SHIFT               62

/* word 2 */
#define RX_COMP2_NEXT_INDEX_MASK          0x00000007FFE00000ULL
#define RX_COMP2_NEXT_INDEX_SHIFT         21
#define RX_COMP2_HDR_SIZE_MASK            0x00000FF800000000ULL
#define RX_COMP2_HDR_SIZE_SHIFT           35
#define RX_COMP2_HDR_OFF_MASK             0x0003F00000000000ULL
#define RX_COMP2_HDR_OFF_SHIFT            44
#define RX_COMP2_HDR_INDEX_MASK           0xFFFC000000000000ULL
#define RX_COMP2_HDR_INDEX_SHIFT          50

/* word 3 */
#define RX_COMP3_SMALL_PKT                0x0000000000000001ULL
#define RX_COMP3_JUMBO_PKT                0x0000000000000002ULL
#define RX_COMP3_JUMBO_HDR_SPLIT_EN       0x0000000000000004ULL
#define RX_COMP3_CSUM_START_MASK          0x000000000007F000ULL
#define RX_COMP3_CSUM_START_SHIFT         12
#define RX_COMP3_FLOWID_MASK              0x0000000001F80000ULL
#define RX_COMP3_FLOWID_SHIFT             19
#define RX_COMP3_OPCODE_MASK              0x000000000E000000ULL
#define RX_COMP3_OPCODE_SHIFT             25
#define RX_COMP3_FORCE_FLAG               0x0000000010000000ULL
#define RX_COMP3_NO_ASSIST                0x0000000020000000ULL
#define RX_COMP3_LOAD_BAL_MASK            0x000001F800000000ULL
#define RX_COMP3_LOAD_BAL_SHIFT           35
#define RX_PLUS_COMP3_ENC_PKT             0x0000020000000000ULL /* cas+ */
#define RX_COMP3_L3_HEAD_OFF_MASK         0x0000FE0000000000ULL /* cas */
#define RX_COMP3_L3_HEAD_OFF_SHIFT        41
#define RX_PLUS_COMP_L3_HEAD_OFF_MASK     0x0000FC0000000000ULL /* cas+ */
#define RX_PLUS_COMP_L3_HEAD_OFF_SHIFT    42
#define RX_COMP3_SAP_MASK                 0xFFFF000000000000ULL
#define RX_COMP3_SAP_SHIFT                48

/* word 4 */
#define RX_COMP4_TCP_CSUM_MASK            0x000000000000FFFFULL
#define RX_COMP4_TCP_CSUM_SHIFT           0
#define RX_COMP4_PKT_LEN_MASK             0x000000003FFF0000ULL
#define RX_COMP4_PKT_LEN_SHIFT            16
#define RX_COMP4_PERFECT_MATCH_MASK       0x00000003C0000000ULL
#define RX_COMP4_PERFECT_MATCH_SHIFT      30
#define RX_COMP4_ZERO                     0x0000080000000000ULL
#define RX_COMP4_HASH_VAL_MASK            0x0FFFF00000000000ULL
#define RX_COMP4_HASH_VAL_SHIFT           44
#define RX_COMP4_HASH_PASS                0x1000000000000000ULL
#define RX_COMP4_BAD                      0x4000000000000000ULL
#define RX_COMP4_LEN_MISMATCH             0x8000000000000000ULL

#define RX_INDEX_NUM_MASK                 0x0000000000000FFFULL
#define RX_INDEX_NUM_SHIFT                0
#define RX_INDEX_RING_MASK                0x0000000000001000ULL
#define RX_INDEX_RING_SHIFT               12
#define RX_INDEX_RELEASE                  0x0000000000002000ULL

struct cas_rx_comp {
	__le64     word1;
	__le64     word2;
	__le64     word3;
	__le64     word4;
};

enum link_state {
	link_down = 0,	/* No link, will retry */
	link_aneg,	/* Autoneg in progress */
	link_force_try,	/* Try Forced link speed */
	link_force_ret,	/* Forced mode worked, retrying autoneg */
	link_force_ok,	/* Stay in forced mode */
	link_up		/* Link is up */
};

typedef struct cas_page {
	struct list_head list;
	struct page *buffer;
	dma_addr_t dma_addr;
	int used;
} cas_page_t;


#define INIT_BLOCK_TX           (TX_DESC_RING_SIZE)
#define INIT_BLOCK_RX_DESC      (RX_DESC_RING_SIZE)
#define INIT_BLOCK_RX_COMP      (RX_COMP_RING_SIZE)

struct cas_init_block {
	struct cas_rx_comp rxcs[N_RX_COMP_RINGS][INIT_BLOCK_RX_COMP];
	struct cas_rx_desc rxds[N_RX_DESC_RINGS][INIT_BLOCK_RX_DESC];
	struct cas_tx_desc txds[N_TX_RINGS][INIT_BLOCK_TX];
	__le64 tx_compwb;
};

#define TX_TINY_BUF_LEN    0x100
#define TX_TINY_BUF_BLOCK  ((INIT_BLOCK_TX + 1)*TX_TINY_BUF_LEN)

struct cas_tiny_count {
	int nbufs;
	int used;
};

struct cas {
	spinlock_t lock; /* for most bits */
	spinlock_t tx_lock[N_TX_RINGS]; /* tx bits */
	spinlock_t stat_lock[N_TX_RINGS + 1]; /* for stat gathering */
	spinlock_t rx_inuse_lock; /* rx inuse list */
	spinlock_t rx_spare_lock; /* rx spare list */

	void __iomem *regs;
	int tx_new[N_TX_RINGS], tx_old[N_TX_RINGS];
	int rx_old[N_RX_DESC_RINGS];
	int rx_cur[N_RX_COMP_RINGS], rx_new[N_RX_COMP_RINGS];
	int rx_last[N_RX_DESC_RINGS];

	struct napi_struct napi;

	/* Set when chip is actually in operational state
	 * (ie. not power managed) */
	int hw_running;
	int opened;
	struct mutex pm_mutex; /* open/close/suspend/resume */

	struct cas_init_block *init_block;
	struct cas_tx_desc *init_txds[MAX_TX_RINGS];
	struct cas_rx_desc *init_rxds[MAX_RX_DESC_RINGS];
	struct cas_rx_comp *init_rxcs[MAX_RX_COMP_RINGS];

	/* we use sk_buffs for tx and pages for rx. the rx skbuffs
	 * are there for flow re-assembly. */
	struct sk_buff      *tx_skbs[N_TX_RINGS][TX_DESC_RING_SIZE];
	struct sk_buff_head  rx_flows[N_RX_FLOWS];
	cas_page_t          *rx_pages[N_RX_DESC_RINGS][RX_DESC_RING_SIZE];
	struct list_head     rx_spare_list, rx_inuse_list;
	int                  rx_spares_needed;

	/* for small packets when copying would be quicker than
	   mapping */
	struct cas_tiny_count tx_tiny_use[N_TX_RINGS][TX_DESC_RING_SIZE];
	u8 *tx_tiny_bufs[N_TX_RINGS];

	u32			msg_enable;

	/* N_TX_RINGS must be >= N_RX_DESC_RINGS */
	struct net_device_stats net_stats[N_TX_RINGS + 1];

	u32			pci_cfg[64 >> 2];
	u8                      pci_revision;

	int                     phy_type;
	int			phy_addr;
	u32                     phy_id;
#define CAS_FLAG_1000MB_CAP     0x00000001
#define CAS_FLAG_REG_PLUS       0x00000002
#define CAS_FLAG_TARGET_ABORT   0x00000004
#define CAS_FLAG_SATURN         0x00000008
#define CAS_FLAG_RXD_POST_MASK  0x000000F0
#define CAS_FLAG_RXD_POST_SHIFT 4
#define CAS_FLAG_RXD_POST(x)    ((1 << (CAS_FLAG_RXD_POST_SHIFT + (x))) & \
                                 CAS_FLAG_RXD_POST_MASK)
#define CAS_FLAG_ENTROPY_DEV    0x00000100
#define CAS_FLAG_NO_HW_CSUM     0x00000200
	u32                     cas_flags;
	int                     packet_min; /* minimum packet size */
	int			tx_fifo_size;
	int			rx_fifo_size;
	int			rx_pause_off;
	int			rx_pause_on;
	int                     crc_size;      /* 4 if half-duplex */

	int                     pci_irq_INTC;
	int                     min_frame_size; /* for tx fifo workaround */

	/* page size allocation */
	int                     page_size;
	int                     page_order;
	int                     mtu_stride;

	u32			mac_rx_cfg;

	/* Autoneg & PHY control */
	int			link_cntl;
	int			link_fcntl;
	enum link_state		lstate;
	struct timer_list	link_timer;
	int			timer_ticks;
	struct work_struct	reset_task;
#if 0
	atomic_t		reset_task_pending;
#else
	atomic_t		reset_task_pending;
	atomic_t		reset_task_pending_mtu;
	atomic_t		reset_task_pending_spare;
	atomic_t		reset_task_pending_all;
#endif

#ifdef CONFIG_CASSINI_QGE_DEBUG
	atomic_t interrupt_seen; /* 1 if any interrupts are getting through */
#endif

	/* Link-down problem workaround */
#define LINK_TRANSITION_UNKNOWN 	0
#define LINK_TRANSITION_ON_FAILURE 	1
#define LINK_TRANSITION_STILL_FAILED 	2
#define LINK_TRANSITION_LINK_UP 	3
#define LINK_TRANSITION_LINK_CONFIG	4
#define LINK_TRANSITION_LINK_DOWN	5
#define LINK_TRANSITION_REQUESTED_RESET	6
	int			link_transition;
	int 			link_transition_jiffies_valid;
	unsigned long		link_transition_jiffies;

	/* Tuning */
	u8 orig_cacheline_size;	/* value when loaded */
#define CAS_PREF_CACHELINE_SIZE	 0x20	/* Minimum desired */

	/* Diagnostic counters and state. */
	int 			casreg_len; /* reg-space size for dumping */
	u64			pause_entered;
	u16			pause_last_time_recvd;

	dma_addr_t block_dvma, tx_tiny_dvma[N_TX_RINGS];
	struct pci_dev *pdev;
	struct net_device *dev;

	/* Firmware Info */
	u16			fw_load_addr;
	u32			fw_size;
	u8			*fw_data;
};

#define TX_DESC_NEXT(r, x)  (((x) + 1) & (TX_DESC_RINGN_SIZE(r) - 1))
#define RX_DESC_ENTRY(r, x) ((x) & (RX_DESC_RINGN_SIZE(r) - 1))
#define RX_COMP_ENTRY(r, x) ((x) & (RX_COMP_RINGN_SIZE(r) - 1))

#define TX_BUFF_COUNT(r, x, y)    ((x) <= (y) ? ((y) - (x)) : \
        (TX_DESC_RINGN_SIZE(r) - (x) + (y)))

#define TX_BUFFS_AVAIL(cp, i)	((cp)->tx_old[(i)] <= (cp)->tx_new[(i)] ? \
        (cp)->tx_old[(i)] + (TX_DESC_RINGN_SIZE(i) - 1) - (cp)->tx_new[(i)] : \
        (cp)->tx_old[(i)] - (cp)->tx_new[(i)] - 1)

#define CAS_ALIGN(addr, align) \
     (((unsigned long) (addr) + ((align) - 1UL)) & ~((align) - 1))

#define RX_FIFO_SIZE                  16384
#define EXPANSION_ROM_SIZE            65536

#define CAS_MC_EXACT_MATCH_SIZE       15
#define CAS_MC_HASH_SIZE              256
#define CAS_MC_HASH_MAX              (CAS_MC_EXACT_MATCH_SIZE + \
                                      CAS_MC_HASH_SIZE)

#define TX_TARGET_ABORT_LEN           0x20
#define RX_SWIVEL_OFF_VAL             0x2
#define RX_AE_FREEN_VAL(x)            (RX_DESC_RINGN_SIZE(x) >> 1)
#define RX_AE_COMP_VAL                (RX_COMP_RING_SIZE >> 1)
#define RX_BLANK_INTR_PKT_VAL         0x05
#define RX_BLANK_INTR_TIME_VAL        0x0F
#define HP_TCP_THRESH_VAL             1530 /* reduce to enable reassembly */

#define RX_SPARE_COUNT                (RX_DESC_RING_SIZE >> 1)
#define RX_SPARE_RECOVER_VAL          (RX_SPARE_COUNT >> 2)

#endif /* _CASSINI_H */
