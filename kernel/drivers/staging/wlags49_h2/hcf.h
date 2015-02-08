

//   vim:tw=110:ts=4:
#ifndef HCF_H
#define HCF_H 1



#include "hcfcfg.h"	// System Constants to be defined by the MSF-programmer to tailor the HCF
#include "mdd.h"	// Include file common for HCF, MSF


/************************************************************************************************/
/**************************************  MACROS  ************************************************/
/************************************************************************************************/

#define LOF(x) 			(sizeof(x)/sizeof(hcf_16)-1)



#define CNV_END_SHORT(w)  (hcf_16)( ((hcf_16)(w) & 0x00FF) << 8 | ((hcf_16)(w) & 0xFF00) >> 8 )
#define CNV_END_LONG(dw)  (hcf_32)( (dw >> 24) | ((dw >> 8) & 0xff00) | ((dw << 8) & 0xff0000) | (dw << 24) )

#if HCF_BIG_ENDIAN
//******************************************** B I G   E N D I A N *******************************************
#define CNV_LITTLE_TO_SHORT(w)	CNV_END_SHORT(w)	//    endianess conversion needed
#define CNV_BIG_TO_SHORT(w)		(w)				// no endianess conversion needed
#define CNV_LITTLE_TO_LONG(dw)	CNV_END_LONG(dw)
#define CNV_LONG_TO_LITTLE(dw)	CNV_END_LONG(dw)
#else
//****************************************** L I T T L E   E N D I A N ****************************************
#define CNV_LITTLE_TO_SHORT(w) 	(w)				// no endianess conversion needed
#define CNV_BIG_TO_SHORT(w)		CNV_END_SHORT(w)	//    endianess conversion needed
#define CNV_LITTLE_TO_LONG(dw)	(dw)
#define CNV_LONG_TO_LITTLE(dw)	(dw)

#if defined HCF_ALIGN && HCF_ALIGN > 1
#define CNV_SHORTP_TO_LITTLE(pw)	((hcf_16)(*(hcf_8 *)pw)) 			| ((hcf_16)(*((hcf_8 *)pw+1)) << 8)
#define CNV_LONGP_TO_LITTLE(pdw)	((hcf_32)(*(hcf_8 *)pdw)) 			| ((hcf_32)(*((hcf_8 *)pdw+1)) << 8) 	| \
									((hcf_32)(*((hcf_8 *)pdw+2)) << 16) | ((hcf_32)(*((hcf_8 *)pdw+3)) << 24)
#else
#define CNV_LONGP_TO_LITTLE(pdw)	(*(hcf_32 *)pdw)
#define CNV_SHORTP_TO_LITTLE(pw)	(*(hcf_16 *)pw)
#endif

#endif // HCF_BIG_ENDIAN

// conversion macros which can be expressed in other macros
#define CNV_SHORT_TO_LITTLE(w)	CNV_LITTLE_TO_SHORT(w)
#define CNV_SHORT_TO_BIG(w)		CNV_BIG_TO_SHORT(w)

/************************************************************************************************/
/**************************************  END OF MACROS  *****************************************/
/************************************************************************************************/

/***********************************************************************************************************/
/*****************                                                  ****************************************/
/***********************************************************************************************************/

// offsets Transmit/Receive Frame Structure
#define HFS_STAT				0x0000
#define HFS_SWSUP				0x0006					//SW Support
#define HFS_Q_INFO				0x0006					//Signal/Silence level
#define HFS_RATE				0x0008					//RxFlow/Rate
#define 	HFS_STAT_ERR		RX_STAT_ERR				//link "natural" HCF name to "natural" MSF name
#define HFS_TX_CNTL				0x0036
														// H-I     H-II
#define HFS_DAT_LEN				(HFS_ADDR_DEST - 2)		// 0x002C  0x0038
#define HFS_ADDR_DEST			0x003A					// 0x002E  0x003A
#define HFS_ADDR_SRC			(HFS_ADDR_DEST + 6)		// 0x0034  0x0040
#define HFS_LEN					(HFS_ADDR_SRC  + 6)		// 0x003A  0x0046
#define HFS_DAT					(HFS_LEN       + 2)		// 0x003C  0x0048
#define HFS_TYPE				(HFS_DAT       + 6)		// 0x0042  0x004E


//=============================  D E S C R I P T O R   S T R U C T U R E  ==============================
//;?MDD.H stuff ;?

#if HCF_BIG_ENDIAN
#define DESC_STRCT_CNT		0
#define DESC_STRCT_SIZE		1
#else
#define DESC_STRCT_CNT		1
#define DESC_STRCT_SIZE		0
#endif // HCF_BIG_ENDIAN

#define	BUF_CNT				buf_dim[DESC_STRCT_CNT]
#define	BUF_SIZE			buf_dim[DESC_STRCT_SIZE]

typedef struct DESC_STRCT {
	hcf_16					buf_dim[2];
	hcf_32					buf_phys_addr;
	hcf_32					next_desc_phys_addr;	// physical address of next descriptor
	hcf_32					desc_phys_addr;			// physical address of this descriptor
	struct DESC_STRCT 		*next_desc_addr;
	hcf_8	FAR				*buf_addr;
#if (HCF_EXT) & HCF_EXT_DESC_STRCT
	void FAR			   *DESC_MSFSup;			// pointer for arbitrary use by the MSF
#endif // HCF_DESC_STRCT_EXT
} DESC_STRCT;

#define HCF_DASA_SIZE			12							//size in bytes for DA/SA

#define DESC_CNT_MASK 			0x0FFF

#define GET_BUF_SIZE(descp)       ((descp)->BUF_SIZE)
#define GET_BUF_CNT(descp)        ((descp)->BUF_CNT)
#define SET_BUF_SIZE(descp, size) (descp)->BUF_SIZE = size;
#define SET_BUF_CNT(descp, count) (descp)->BUF_CNT = count;

//=========================================  T A L L I E S  ===================================================

typedef struct  {  //Hermes Tallies (IFB substructure)
  hcf_32	TxUnicastFrames;
  hcf_32	TxMulticastFrames;
  hcf_32	TxFragments;
  hcf_32	TxUnicastOctets;
  hcf_32	TxMulticastOctets;
  hcf_32	TxDeferredTransmissions;
  hcf_32	TxSingleRetryFrames;
  hcf_32	TxMultipleRetryFrames;
  hcf_32	TxRetryLimitExceeded;
  hcf_32	TxDiscards;
  hcf_32	RxUnicastFrames;
  hcf_32	RxMulticastFrames;
  hcf_32	RxFragments;
  hcf_32	RxUnicastOctets;
  hcf_32	RxMulticastOctets;
  hcf_32	RxFCSErrors;
  hcf_32	RxDiscardsNoBuffer;
  hcf_32	TxDiscardsWrongSA;
  hcf_32	RxWEPUndecryptable;
  hcf_32	RxMsgInMsgFragments;
  hcf_32	RxMsgInBadMsgFragments;
  hcf_32	RxDiscardsWEPICVError;
  hcf_32	RxDiscardsWEPExcluded;
#if (HCF_EXT) & HCF_EXT_TALLIES_FW
  hcf_32	TalliesExtra[32];
#endif // HCF_EXT_TALLIES_FW
} CFG_HERMES_TALLIES_STRCT;

typedef struct  {  //HCF Tallies (IFB substructure)
  hcf_32	NoBufInfo;  				//No buffer available for unsolicited Notify frame
  hcf_32	NoBufMB;					//No space available in MailBox
  hcf_32	MiscErr;					/* Command errors
  										 *  - time out on completion synchronous part Hermes Command
  										 *  - completed Hermes Command doesn't match original command
  										 *  - status of completed Hermes Command contains error bits
  										 */
#if (HCF_EXT) & HCF_EXT_TALLIES_FW
  hcf_32	EngCnt[8];
#endif // HCF_EXT_TALLIES_FW
} CFG_HCF_TALLIES_STRCT;

//Note this way to define ..._TAL_CNT implies that all tallies must keep the same (hcf_32) size
#if (HCF_TALLIES) & ( HCF_TALLIES_NIC | HCF_TALLIES_HCF )
#if (HCF_TALLIES) & HCF_TALLIES_NIC	//Hermes tally support
#define		HCF_NIC_TAL_CNT	(sizeof(CFG_HERMES_TALLIES_STRCT)/ sizeof(hcf_32))
#else
#define		HCF_NIC_TAL_CNT	0
#endif // HCF_TALLIES
#if (HCF_TALLIES) & HCF_TALLIES_HCF	//HCF tally support
#define		HCF_HCF_TAL_CNT	(sizeof(CFG_HCF_TALLIES_STRCT)   / sizeof(hcf_32))
#else
#define		HCF_HCF_TAL_CNT	0
#endif // HCF_TALLIES
#define HCF_TOT_TAL_CNT ( HCF_NIC_TAL_CNT + HCF_NIC_TAL_CNT )
#endif // HCF_TALLIES_NIC / HCF_TALLIES_HCF


/***********************************************************************************************************/
/********************************** I N T E R F A C E   B L O C K ******************************************/
/***********************************************************************************************************/

#define IFB_VERSION 0x0E	 			// initially 0, to be incremented by every IFB layout change

typedef struct  {
  hcf_io		IFB_IOBase;				// I/O address of Hermes chip as passed by MSF at hcf_connect call
  hcf_16		IFB_IORange;			// I/O Range used by Hermes chip
  hcf_16		IFB_DLMode;				// Download Mode state
  hcf_16		IFB_Cmd;				// cmd in progress flag, to be ack-ed before next cmd can be issued
  hcf_16		IFB_RxFID;				// FID of "current" RxFS (non-DMA mode)
//;?#if tx_delay option
  hcf_16		IFB_TxFID;				// fid storage during "delayed" send
//;?#endif tx_delay option
  hcf_16		IFB_RxLen;				//
  hcf_16		IFB_DefunctStat;		// BAP initialization or Cmd Completion failed
  hcf_16		IFB_ErrCmd;				// contents Status reg when error bits and/or mismatch in cmd_wait
  hcf_16		IFB_ErrQualifier;		// contents Resp0  reg when error bits and/or mismatch in cmd_wait
  hcf_16		IFB_lal;				// LookAhead Length
  wci_bufp		IFB_lap;				// LookAhead Buffer pointer
  hcf_16		IFB_LinkStat;			// Link Status
  hcf_16		IFB_DSLinkStat;			// Link Status, new strategy introduced for DeepSleep
  hcf_16		IFB_CarryIn;			// carry and carry-flag to move 1 byte from one get_frag to the next
  hcf_16		IFB_CarryOut;			// carry and carry-flag to move 1 byte from one put_frag to the next
  hcf_16		IFB_Version;			// IFB_VERSION, incremented by every SIGNIFICANT IFB layout change
  hcf_16		IFB_CardStat;			// NIC error / F/W incompatibility status
  hcf_16  		IFB_RscInd;				// non-DMA: TxFID available, DMA: always 1
  hcf_16		IFB_CntlOpt;			// flags: 32 bits I/O, DMA available, DMA enabled
  hcf_16		IFB_BusType;			// BusType, derived via CFG_NIC_BUS_TYPE
  CFG_FW_IDENTITY_STRCT	 IFB_FWIdentity; /* keep FWIdentity/Sup and PRIIdentity/Sup in sequence
										  * because of the (dumb) copy in init() */
#if defined MSF_COMPONENT_ID
  CFG_SUP_RANGE_STRCT	 IFB_FWSup;
  CFG_PRI_IDENTITY_STRCT IFB_PRIIdentity;
  CFG_SUP_RANGE_STRCT	 IFB_PRISup;
  CFG_SUP_RANGE_STRCT	 IFB_HSISup;
#endif // MSF_COMPONENT_ID
#if (HCF_EXT) & HCF_EXT_INFO_LOG
  RID_LOGP		IFB_RIDLogp;			// pointer to RID_LOG structure
#endif // HCF_EXT_INFO_LOG
#if HCF_PROT_TIME
  hcf_32	 	IFB_TickIni;			// initialization of S/W counter based protection loop
#endif // HCF_PROT_TIME
#if (HCF_EXT) & HCF_EXT_INT_TICK
  int			IFB_TickCnt;			// Hermes Timer Tick Counter
#endif // HCF_EXT_INT_TICK
#if (HCF_EXT) & HCF_EXT_MB
  hcf_16 	   *IFB_MBp;				// pointer to the MailBox
  hcf_16		IFB_MBSize;				// size of the MailBox
  hcf_16		IFB_MBWp;				// zero-based write index into the MailBox
  hcf_16		IFB_MBRp;				// zero-based read  index into the MailBox
  hcf_16		IFB_MBInfoLen;			// contents of L-field of the oldest available MailBoxInfoBlock
#endif // HCF_EXT_MB
#if (HCF_TYPE) & HCF_TYPE_WPA
  hcf_16		IFB_MICTxCntl;			// MIC bit and Key index in TxControl field of TxFS
  hcf_32		IFB_MICTxKey[2];		// calculating key
  hcf_32		IFB_MICTx[2];   	  	// Tx MIC calculation Engine state
  hcf_16		IFB_MICTxCarry;			// temp length, carries over from one Tx fragment to another
  hcf_16		IFB_MICRxCarry;			// temp length, carries over from one Rx fragment to another
  hcf_32		IFB_MICRxKey[4*2];		// 4 checking keys
  hcf_32		IFB_MICRx[2]; 	 		// Rx MIC calculation Engine state
#endif // HCF_TYPE_WPA
#if HCF_ASSERT
#if (HCF_ASSERT) & HCF_ASSERT_MB
  CFG_MB_INFO_RANGE1_STRCT	IFB_AssertStrct; // Add some complication to the HCF as prize for the new MSF I/F
#endif // HCF_ASSERT_MB
  										// target of above IFB_AssertStrct
  hcf_16		IFB_AssertLine;			//  - line number ( + encoded module name )
  hcf_16		IFB_AssertTrace;		//  - bit based trace of all hcf_.... invocations
  hcf_32		IFB_AssertQualifier;	//  - qualifier
  hcf_16		IFB_AssertLvl;			// Assert Filtering, Not yet implemented
  hcf_16		IFB_AssertWhere;		// Where parameter of the Assert macro
#if (HCF_ASSERT) & ( HCF_ASSERT_LNK_MSF_RTN | HCF_ASSERT_RT_MSF_RTN )
  MSF_ASSERT_RTNP	IFB_AssertRtn;		// MSF Assert Call back routine (inspired by GEF, DrDobbs Nov 1998 )
#endif // HCF_ASSERT_LNK_MSF_RTN
#if (HCF_ASSERT) & HCF_ASSERT_PRINTF	// engineering facilty intended as F/W debugging aid
   hcf_16       IFB_DbgPrintF_Cnt;
   CFG_FW_PRINTF_BUFFER_LOCATION_STRCT IFB_FwPfBuff;
#endif // HCF_ASSERT_PRINTF
#endif // HCF_ASSERT
#if ! defined HCF_INT_OFF
  hcf_16 volatile IFB_IntOffCnt;		// 0xFFFF based HCF_ACT_INT_OFF nesting counter, DeepSleep flag
#endif // HCF_INT_OFF
#if (HCF_TYPE) & HCF_TYPE_CCX
  hcf_16         IFB_CKIPStat;			// CKIP Status flag
#endif // HCF_TYPE_CCX
#if (HCF_TALLIES) & ( HCF_TALLIES_NIC | HCF_TALLIES_HCF )	//Hermes and/or HCF tally support
  hcf_32		IFB_Silly_you_should_align;	//;?
  hcf_16		IFB_TallyLen;			// Tally length (to build an LTV)
  hcf_16		IFB_TallyTyp;			// Tally Type (to build an LTV)
#endif // HCF_TALLIES_NIC / HCF_TALLIES_HCF
#if (HCF_TALLIES) & HCF_TALLIES_NIC		//Hermes tally support
  CFG_HERMES_TALLIES_STRCT	IFB_NIC_Tallies;
#endif // HCF_TALLIES_NIC
#if (HCF_TALLIES) & HCF_TALLIES_HCF		//HCF tally support
  CFG_HCF_TALLIES_STRCT		IFB_HCF_Tallies;
#endif // HCF_TALLIES_HCF
#if HCF_DMA
  //used for a pool of destination_address descriptor/buffers, used during tx encapsulation points to the
  //first/last descriptor in the descriptor chain, so we can easily remove and append a packet.
  DESC_STRCT	*IFB_FirstDesc[2];
  DESC_STRCT	*IFB_LastDesc[2];
  DESC_STRCT	*IFB_ConfinedDesc[2];	// pointers to descriptor used for host reclaim purposes.
  hcf_16		IFB_DmaPackets;			// HREG_EV_[TX/RX]DMA_DONE flags, reports DMA Frame availability to MSF
#endif // HCF_DMA
#if (HCF_EXT) & HCF_EXT_INT_TX_EX
  hcf_16		IFB_TxFsStat;			// Tx message monitoring
  hcf_16		IFB_TxFsGap[2];			//;?make this robust
  hcf_16		IFB_TxFsSwSup;
#endif // HCF_EXT_INT_TX_EX
  hcf_16		IFB_Magic;				/* "Magic" signature, to help the debugger interpret a memory dump
										 * also the last field cleared at hcf_connect
										 */
#if (HCF_EXT) & HCF_EXT_IFB_STRCT		// for usage by the MSF
  void FAR	   *IFB_MSFSup;				// pointer for arbitrary use by the MSF
#endif // HCF_EXT_IFB_STRCT_EXT
} IFB_STRCT;

typedef IFB_STRCT*	IFBP;


/***********************************************************************************************************/
/**********************   W C I    F U N C T I O N S    P R O T O T Y P E S   ******************************/
/***********************************************************************************************************/

EXTERN_C int		 hcf_action			(IFBP ifbp, hcf_16 cmd );
EXTERN_C int		 hcf_connect		(IFBP ifbp, hcf_io io_base );
#if (HCF_ENCAP) & HCF_ENC_SUP
EXTERN_C hcf_8 		 hcf_encap			(wci_bufp type );
#endif // HCF_ENC_SUP
EXTERN_C int		 hcf_get_info		(IFBP ifbp, LTVP ltvp );
EXTERN_C int		 hcf_service_nic	(IFBP ifbp, wci_bufp bufp, unsigned int len );
EXTERN_C int		 hcf_cntl			(IFBP ifbp, hcf_16 cmd );
EXTERN_C int		 hcf_put_info		(IFBP ifbp, LTVP ltvp );
EXTERN_C int		 hcf_rcv_msg		(IFBP ifbp, DESC_STRCT *descp, unsigned int offset );
EXTERN_C int		 hcf_send_msg       (IFBP ifbp, DESC_STRCT *dp, hcf_16 tx_cntl );
#if HCF_DMA
EXTERN_C void		 hcf_dma_tx_put 	(IFBP ifbp, DESC_STRCT *d, hcf_16 tx_cntl );
EXTERN_C DESC_STRCT* hcf_dma_tx_get		(IFBP ifbp );
EXTERN_C DESC_STRCT* hcf_dma_rx_get		(IFBP ifbp );
EXTERN_C void		 hcf_dma_rx_put		(IFBP ifbp, DESC_STRCT *d );
#endif // HCF_DMA
#if (HCF_ASSERT) & HCF_ASSERT_LNK_MSF_RTN
EXTERN_C void		 msf_assert	 		(unsigned int line_number, hcf_16 trace, hcf_32 qual );
#endif // HCF_ASSERT_LNK_MSF_RTN

#endif  // HCF_H

