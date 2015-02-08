

#define ROCKET_TYPE_NORMAL	0
#define ROCKET_TYPE_MODEM	1
#define ROCKET_TYPE_MODEMII	2
#define ROCKET_TYPE_MODEMIII	3
#define ROCKET_TYPE_PC104       4

#include <linux/mutex.h>

#include <asm/io.h>
#include <asm/byteorder.h>

typedef unsigned char Byte_t;
typedef unsigned int ByteIO_t;

typedef unsigned int Word_t;
typedef unsigned int WordIO_t;

typedef unsigned int DWordIO_t;


static inline void sOutB(unsigned short port, unsigned char value)
{
#ifdef ROCKET_DEBUG_IO
	printk(KERN_DEBUG "sOutB(%x, %x)...\n", port, value);
#endif
	outb_p(value, port);
}

static inline void sOutW(unsigned short port, unsigned short value)
{
#ifdef ROCKET_DEBUG_IO
	printk(KERN_DEBUG "sOutW(%x, %x)...\n", port, value);
#endif
	outw_p(value, port);
}

static inline void out32(unsigned short port, Byte_t *p)
{
	u32 value = get_unaligned_le32(p);
#ifdef ROCKET_DEBUG_IO
	printk(KERN_DEBUG "out32(%x, %lx)...\n", port, value);
#endif
	outl_p(value, port);
}

static inline unsigned char sInB(unsigned short port)
{
	return inb_p(port);
}

static inline unsigned short sInW(unsigned short port)
{
	return inw_p(port);
}

/* This is used to move arrays of bytes so byte swapping isn't appropriate. */
#define sOutStrW(port, addr, count) if (count) outsw(port, addr, count)
#define sInStrW(port, addr, count) if (count) insw(port, addr, count)

#define CTL_SIZE 8
#define AIOP_CTL_SIZE 4
#define CHAN_AIOP_SIZE 8
#define MAX_PORTS_PER_AIOP 8
#define MAX_AIOPS_PER_BOARD 4
#define MAX_PORTS_PER_BOARD 32

/* Bus type ID */
#define	isISA	0
#define	isPCI	1
#define	isMC	2

/* Controller ID numbers */
#define CTLID_NULL  -1		/* no controller exists */
#define CTLID_0001  0x0001	/* controller release 1 */

/* AIOP ID numbers, identifies AIOP type implementing channel */
#define AIOPID_NULL -1		/* no AIOP or channel exists */
#define AIOPID_0001 0x0001	/* AIOP release 1 */


#define _CMD_REG   0x38		/* Command Register            8    Write */
#define _INT_CHAN  0x39		/* Interrupt Channel Register  8    Read */
#define _INT_MASK  0x3A		/* Interrupt Mask Register     8    Read / Write */
#define _UNUSED    0x3B		/* Unused                      8 */
#define _INDX_ADDR 0x3C		/* Index Register Address      16   Write */
#define _INDX_DATA 0x3E		/* Index Register Data         8/16 Read / Write */

#define _TD0       0x00		/* Transmit Data               16   Write */
#define _RD0       0x00		/* Receive Data                16   Read */
#define _CHN_STAT0 0x20		/* Channel Status              8/16 Read / Write */
#define _FIFO_CNT0 0x10		/* Transmit/Receive FIFO Count 16   Read */
#define _INT_ID0   0x30		/* Interrupt Identification    8    Read */

#define _TX_ENBLS  0x980	/* Tx Processor Enables Register 8 Read / Write */
#define _TXCMP1    0x988	/* Transmit Compare Value #1     8 Read / Write */
#define _TXCMP2    0x989	/* Transmit Compare Value #2     8 Read / Write */
#define _TXREP1B1  0x98A	/* Tx Replace Value #1 - Byte 1  8 Read / Write */
#define _TXREP1B2  0x98B	/* Tx Replace Value #1 - Byte 2  8 Read / Write */
#define _TXREP2    0x98C	/* Transmit Replace Value #2     8 Read / Write */

#define _RX_FIFO    0x000	/* Rx FIFO */
#define _TX_FIFO    0x800	/* Tx FIFO */
#define _RXF_OUTP   0x990	/* Rx FIFO OUT pointer        16 Read / Write */
#define _RXF_INP    0x992	/* Rx FIFO IN pointer         16 Read / Write */
#define _TXF_OUTP   0x994	/* Tx FIFO OUT pointer        8  Read / Write */
#define _TXF_INP    0x995	/* Tx FIFO IN pointer         8  Read / Write */
#define _TXP_CNT    0x996	/* Tx Priority Count          8  Read / Write */
#define _TXP_PNTR   0x997	/* Tx Priority Pointer        8  Read / Write */

#define PRI_PEND    0x80	/* Priority data pending (bit7, Tx pri cnt) */
#define TXFIFO_SIZE 255		/* size of Tx FIFO */
#define RXFIFO_SIZE 1023	/* size of Rx FIFO */

#define _TXP_BUF    0x9C0	/* Tx Priority Buffer  32  Bytes   Read / Write */
#define TXP_SIZE    0x20	/* 32 bytes */


#define _TX_CTRL    0xFF0	/* Transmit Control               16  Write */
#define _RX_CTRL    0xFF2	/* Receive Control                 8  Write */
#define _BAUD       0xFF4	/* Baud Rate                      16  Write */
#define _CLK_PRE    0xFF6	/* Clock Prescaler                 8  Write */

#define STMBREAK   0x08		/* BREAK */
#define STMFRAME   0x04		/* framing error */
#define STMRCVROVR 0x02		/* receiver over run error */
#define STMPARITY  0x01		/* parity error */
#define STMERROR   (STMBREAK | STMFRAME | STMPARITY)
#define STMBREAKH   0x800	/* BREAK */
#define STMFRAMEH   0x400	/* framing error */
#define STMRCVROVRH 0x200	/* receiver over run error */
#define STMPARITYH  0x100	/* parity error */
#define STMERRORH   (STMBREAKH | STMFRAMEH | STMPARITYH)

#define CTS_ACT   0x20		/* CTS input asserted */
#define DSR_ACT   0x10		/* DSR input asserted */
#define CD_ACT    0x08		/* CD input asserted */
#define TXFIFOMT  0x04		/* Tx FIFO is empty */
#define TXSHRMT   0x02		/* Tx shift register is empty */
#define RDA       0x01		/* Rx data available */
#define DRAINED (TXFIFOMT | TXSHRMT)	/* indicates Tx is drained */

#define STATMODE  0x8000	/* status mode enable bit */
#define RXFOVERFL 0x2000	/* receive FIFO overflow */
#define RX2MATCH  0x1000	/* receive compare byte 2 match */
#define RX1MATCH  0x0800	/* receive compare byte 1 match */
#define RXBREAK   0x0400	/* received BREAK */
#define RXFRAME   0x0200	/* received framing error */
#define RXPARITY  0x0100	/* received parity error */
#define STATERROR (RXBREAK | RXFRAME | RXPARITY)

#define CTSFC_EN  0x80		/* CTS flow control enable bit */
#define RTSTOG_EN 0x40		/* RTS toggle enable bit */
#define TXINT_EN  0x10		/* transmit interrupt enable */
#define STOP2     0x08		/* enable 2 stop bits (0 = 1 stop) */
#define PARITY_EN 0x04		/* enable parity (0 = no parity) */
#define EVEN_PAR  0x02		/* even parity (0 = odd parity) */
#define DATA8BIT  0x01		/* 8 bit data (0 = 7 bit data) */

#define SETBREAK  0x10		/* send break condition (must clear) */
#define LOCALLOOP 0x08		/* local loopback set for test */
#define SET_DTR   0x04		/* assert DTR */
#define SET_RTS   0x02		/* assert RTS */
#define TX_ENABLE 0x01		/* enable transmitter */

#define RTSFC_EN  0x40		/* RTS flow control enable */
#define RXPROC_EN 0x20		/* receive processor enable */
#define TRIG_NO   0x00		/* Rx FIFO trigger level 0 (no trigger) */
#define TRIG_1    0x08		/* trigger level 1 char */
#define TRIG_1_2  0x10		/* trigger level 1/2 */
#define TRIG_7_8  0x18		/* trigger level 7/8 */
#define TRIG_MASK 0x18		/* trigger level mask */
#define SRCINT_EN 0x04		/* special Rx condition interrupt enable */
#define RXINT_EN  0x02		/* Rx interrupt enable */
#define MCINT_EN  0x01		/* modem change interrupt enable */

#define RXF_TRIG  0x20		/* Rx FIFO trigger level interrupt */
#define TXFIFO_MT 0x10		/* Tx FIFO empty interrupt */
#define SRC_INT   0x08		/* special receive condition interrupt */
#define DELTA_CD  0x04		/* CD change interrupt */
#define DELTA_CTS 0x02		/* CTS change interrupt */
#define DELTA_DSR 0x01		/* DSR change interrupt */

#define REP1W2_EN 0x10		/* replace byte 1 with 2 bytes enable */
#define IGN2_EN   0x08		/* ignore byte 2 enable */
#define IGN1_EN   0x04		/* ignore byte 1 enable */
#define COMP2_EN  0x02		/* compare byte 2 enable */
#define COMP1_EN  0x01		/* compare byte 1 enable */

#define RESET_ALL 0x80		/* reset AIOP (all channels) */
#define TXOVERIDE 0x40		/* Transmit software off override */
#define RESETUART 0x20		/* reset channel's UART */
#define RESTXFCNT 0x10		/* reset channel's Tx FIFO count register */
#define RESRXFCNT 0x08		/* reset channel's Rx FIFO count register */

#define INTSTAT0  0x01		/* AIOP 0 interrupt status */
#define INTSTAT1  0x02		/* AIOP 1 interrupt status */
#define INTSTAT2  0x04		/* AIOP 2 interrupt status */
#define INTSTAT3  0x08		/* AIOP 3 interrupt status */

#define INTR_EN   0x08		/* allow interrupts to host */
#define INT_STROB 0x04		/* strobe and clear interrupt line (EOI) */


#define _CFG_INT_PCI  0x40
#define _PCI_INT_FUNC 0x3A

#define PCI_STROB 0x2000	/* bit 13 of int aiop register */
#define INTR_EN_PCI   0x0010	/* allow interrupts to host */

#define _PCI_9030_INT_CTRL	0x4c          /* Offsets from BAR1 */
#define _PCI_9030_GPIO_CTRL	0x54
#define PCI_INT_CTRL_AIOP	0x0001
#define PCI_GPIO_CTRL_8PORT	0x4000
#define _PCI_9030_RING_IND	0xc0          /* Offsets from BAR1 */

#define CHAN3_EN  0x08		/* enable AIOP 3 */
#define CHAN2_EN  0x04		/* enable AIOP 2 */
#define CHAN1_EN  0x02		/* enable AIOP 1 */
#define CHAN0_EN  0x01		/* enable AIOP 0 */
#define FREQ_DIS  0x00
#define FREQ_274HZ 0x60
#define FREQ_137HZ 0x50
#define FREQ_69HZ  0x40
#define FREQ_34HZ  0x30
#define FREQ_17HZ  0x20
#define FREQ_9HZ   0x10
#define PERIODIC_ONLY 0x80	/* only PERIODIC interrupt */

#define CHANINT_EN 0x0100	/* flags to enable/disable channel ints */

#define RDATASIZE 72
#define RREGDATASIZE 52

#define AIOP_INTR_BIT_0		0x0001
#define AIOP_INTR_BIT_1		0x0002
#define AIOP_INTR_BIT_2		0x0004
#define AIOP_INTR_BIT_3		0x0008

#define AIOP_INTR_BITS ( \
	AIOP_INTR_BIT_0 \
	| AIOP_INTR_BIT_1 \
	| AIOP_INTR_BIT_2 \
	| AIOP_INTR_BIT_3)

#define UPCI_AIOP_INTR_BIT_0	0x0004
#define UPCI_AIOP_INTR_BIT_1	0x0020
#define UPCI_AIOP_INTR_BIT_2	0x0100
#define UPCI_AIOP_INTR_BIT_3	0x0800

#define UPCI_AIOP_INTR_BITS ( \
	UPCI_AIOP_INTR_BIT_0 \
	| UPCI_AIOP_INTR_BIT_1 \
	| UPCI_AIOP_INTR_BIT_2 \
	| UPCI_AIOP_INTR_BIT_3)

/* Controller level information structure */
typedef struct {
	int CtlID;
	int CtlNum;
	int BusType;
	int boardType;
	int isUPCI;
	WordIO_t PCIIO;
	WordIO_t PCIIO2;
	ByteIO_t MBaseIO;
	ByteIO_t MReg1IO;
	ByteIO_t MReg2IO;
	ByteIO_t MReg3IO;
	Byte_t MReg2;
	Byte_t MReg3;
	int NumAiop;
	int AltChanRingIndicator;
	ByteIO_t UPCIRingInd;
	WordIO_t AiopIO[AIOP_CTL_SIZE];
	ByteIO_t AiopIntChanIO[AIOP_CTL_SIZE];
	int AiopID[AIOP_CTL_SIZE];
	int AiopNumChan[AIOP_CTL_SIZE];
	Word_t *AiopIntrBits;
} CONTROLLER_T;

typedef CONTROLLER_T CONTROLLER_t;

/* Channel level information structure */
typedef struct {
	CONTROLLER_T *CtlP;
	int AiopNum;
	int ChanID;
	int ChanNum;
	int rtsToggle;

	ByteIO_t Cmd;
	ByteIO_t IntChan;
	ByteIO_t IntMask;
	DWordIO_t IndexAddr;
	WordIO_t IndexData;

	WordIO_t TxRxData;
	WordIO_t ChanStat;
	WordIO_t TxRxCount;
	ByteIO_t IntID;

	Word_t TxFIFO;
	Word_t TxFIFOPtrs;
	Word_t RxFIFO;
	Word_t RxFIFOPtrs;
	Word_t TxPrioCnt;
	Word_t TxPrioPtr;
	Word_t TxPrioBuf;

	Byte_t R[RREGDATASIZE];

	Byte_t BaudDiv[4];
	Byte_t TxControl[4];
	Byte_t RxControl[4];
	Byte_t TxEnables[4];
	Byte_t TxCompare[4];
	Byte_t TxReplace1[4];
	Byte_t TxReplace2[4];
} CHANNEL_T;

typedef CHANNEL_T CHANNEL_t;
typedef CHANNEL_T *CHANPTR_T;

#define InterfaceModeRS232  0x00
#define InterfaceModeRS422  0x08
#define InterfaceModeRS485  0x10
#define InterfaceModeRS232T 0x18

#define sClrBreak(ChP) \
do { \
   (ChP)->TxControl[3] &= ~SETBREAK; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sClrDTR(ChP) \
do { \
   (ChP)->TxControl[3] &= ~SET_DTR; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sClrRTS(ChP) \
do { \
   if ((ChP)->rtsToggle) break; \
   (ChP)->TxControl[3] &= ~SET_RTS; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sClrTxXOFF(ChP) \
do { \
   sOutB((ChP)->Cmd,TXOVERIDE | (Byte_t)(ChP)->ChanNum); \
   sOutB((ChP)->Cmd,(Byte_t)(ChP)->ChanNum); \
} while (0)

#define sCtlNumToCtlPtr(CTLNUM) &sController[CTLNUM]

#define sControllerEOI(CTLP) sOutB((CTLP)->MReg2IO,(CTLP)->MReg2 | INT_STROB)

#define sPCIControllerEOI(CTLP) \
do { \
    if ((CTLP)->isUPCI) { \
	Word_t w = sInW((CTLP)->PCIIO); \
	sOutW((CTLP)->PCIIO, (w ^ PCI_INT_CTRL_AIOP)); \
	sOutW((CTLP)->PCIIO, w); \
    } \
    else { \
	sOutW((CTLP)->PCIIO, PCI_STROB); \
    } \
} while (0)

#define sDisAiop(CTLP,AIOPNUM) \
do { \
   (CTLP)->MReg3 &= sBitMapClrTbl[AIOPNUM]; \
   sOutB((CTLP)->MReg3IO,(CTLP)->MReg3); \
} while (0)

#define sDisCTSFlowCtl(ChP) \
do { \
   (ChP)->TxControl[2] &= ~CTSFC_EN; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sDisIXANY(ChP) \
do { \
   (ChP)->R[0x0e] = 0x86; \
   out32((ChP)->IndexAddr,&(ChP)->R[0x0c]); \
} while (0)

#define sDisParity(ChP) \
do { \
   (ChP)->TxControl[2] &= ~PARITY_EN; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sDisRTSToggle(ChP) \
do { \
   (ChP)->TxControl[2] &= ~RTSTOG_EN; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
   (ChP)->rtsToggle = 0; \
} while (0)

#define sDisRxFIFO(ChP) \
do { \
   (ChP)->R[0x32] = 0x0a; \
   out32((ChP)->IndexAddr,&(ChP)->R[0x30]); \
} while (0)

#define sDisRxStatusMode(ChP) sOutW((ChP)->ChanStat,0)

#define sDisTransmit(ChP) \
do { \
   (ChP)->TxControl[3] &= ~TX_ENABLE; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sDisTxSoftFlowCtl(ChP) \
do { \
   (ChP)->R[0x06] = 0x8a; \
   out32((ChP)->IndexAddr,&(ChP)->R[0x04]); \
} while (0)

#define sEnAiop(CTLP,AIOPNUM) \
do { \
   (CTLP)->MReg3 |= sBitMapSetTbl[AIOPNUM]; \
   sOutB((CTLP)->MReg3IO,(CTLP)->MReg3); \
} while (0)

#define sEnCTSFlowCtl(ChP) \
do { \
   (ChP)->TxControl[2] |= CTSFC_EN; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sEnIXANY(ChP) \
do { \
   (ChP)->R[0x0e] = 0x21; \
   out32((ChP)->IndexAddr,&(ChP)->R[0x0c]); \
} while (0)

#define sEnParity(ChP) \
do { \
   (ChP)->TxControl[2] |= PARITY_EN; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sEnRTSToggle(ChP) \
do { \
   (ChP)->RxControl[2] &= ~RTSFC_EN; \
   out32((ChP)->IndexAddr,(ChP)->RxControl); \
   (ChP)->TxControl[2] |= RTSTOG_EN; \
   (ChP)->TxControl[3] &= ~SET_RTS; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
   (ChP)->rtsToggle = 1; \
} while (0)

#define sEnRxFIFO(ChP) \
do { \
   (ChP)->R[0x32] = 0x08; \
   out32((ChP)->IndexAddr,&(ChP)->R[0x30]); \
} while (0)

#define sEnRxProcessor(ChP) \
do { \
   (ChP)->RxControl[2] |= RXPROC_EN; \
   out32((ChP)->IndexAddr,(ChP)->RxControl); \
} while (0)

#define sEnRxStatusMode(ChP) sOutW((ChP)->ChanStat,STATMODE)

#define sEnTransmit(ChP) \
do { \
   (ChP)->TxControl[3] |= TX_ENABLE; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sEnTxSoftFlowCtl(ChP) \
do { \
   (ChP)->R[0x06] = 0xc5; \
   out32((ChP)->IndexAddr,&(ChP)->R[0x04]); \
} while (0)

#define sGetAiopIntStatus(CTLP,AIOPNUM) sInB((CTLP)->AiopIntChanIO[AIOPNUM])

#define sGetAiopNumChan(CTLP,AIOPNUM) (CTLP)->AiopNumChan[AIOPNUM]

#define sGetChanIntID(ChP) (sInB((ChP)->IntID) & (RXF_TRIG | TXFIFO_MT | SRC_INT | DELTA_CD | DELTA_CTS | DELTA_DSR))

#define sGetChanNum(ChP) (ChP)->ChanNum

#define sGetChanStatus(ChP) sInW((ChP)->ChanStat)

#define sGetChanStatusLo(ChP) sInB((ByteIO_t)(ChP)->ChanStat)

#if 0
#define sGetChanRI(ChP) ((ChP)->CtlP->AltChanRingIndicator ? \
                          (sInB((ByteIO_t)((ChP)->ChanStat+8)) & DSR_ACT) : \
                            (((ChP)->CtlP->boardType == ROCKET_TYPE_PC104) ? \
                               (!(sInB((ChP)->CtlP->AiopIO[3]) & sBitMapSetTbl[(ChP)->ChanNum])) : \
                             0))
#endif

#define sGetControllerIntStatus(CTLP) (sInB((CTLP)->MReg1IO) & 0x0f)

#define sPCIGetControllerIntStatus(CTLP) \
	((CTLP)->isUPCI ? \
	  (sInW((CTLP)->PCIIO2) & UPCI_AIOP_INTR_BITS) : \
	  ((sInW((CTLP)->PCIIO) >> 8) & AIOP_INTR_BITS))

#define sGetRxCnt(ChP) sInW((ChP)->TxRxCount)

#define sGetTxCnt(ChP) sInB((ByteIO_t)(ChP)->TxRxCount)

#define sGetTxRxDataIO(ChP) (ChP)->TxRxData

#define sInitChanDefaults(ChP) \
do { \
   (ChP)->CtlP = NULLCTLPTR; \
   (ChP)->AiopNum = NULLAIOP; \
   (ChP)->ChanID = AIOPID_NULL; \
   (ChP)->ChanNum = NULLCHAN; \
} while (0)

#define sResetAiopByNum(CTLP,AIOPNUM) \
do { \
   sOutB((CTLP)->AiopIO[(AIOPNUM)]+_CMD_REG,RESET_ALL); \
   sOutB((CTLP)->AiopIO[(AIOPNUM)]+_CMD_REG,0x0); \
} while (0)

#define sSendBreak(ChP) \
do { \
   (ChP)->TxControl[3] |= SETBREAK; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sSetBaud(ChP,DIVISOR) \
do { \
   (ChP)->BaudDiv[2] = (Byte_t)(DIVISOR); \
   (ChP)->BaudDiv[3] = (Byte_t)((DIVISOR) >> 8); \
   out32((ChP)->IndexAddr,(ChP)->BaudDiv); \
} while (0)

#define sSetData7(ChP) \
do { \
   (ChP)->TxControl[2] &= ~DATA8BIT; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sSetData8(ChP) \
do { \
   (ChP)->TxControl[2] |= DATA8BIT; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sSetDTR(ChP) \
do { \
   (ChP)->TxControl[3] |= SET_DTR; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sSetEvenParity(ChP) \
do { \
   (ChP)->TxControl[2] |= EVEN_PAR; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sSetOddParity(ChP) \
do { \
   (ChP)->TxControl[2] &= ~EVEN_PAR; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sSetRTS(ChP) \
do { \
   if ((ChP)->rtsToggle) break; \
   (ChP)->TxControl[3] |= SET_RTS; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sSetRxTrigger(ChP,LEVEL) \
do { \
   (ChP)->RxControl[2] &= ~TRIG_MASK; \
   (ChP)->RxControl[2] |= LEVEL; \
   out32((ChP)->IndexAddr,(ChP)->RxControl); \
} while (0)

#define sSetStop1(ChP) \
do { \
   (ChP)->TxControl[2] &= ~STOP2; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sSetStop2(ChP) \
do { \
   (ChP)->TxControl[2] |= STOP2; \
   out32((ChP)->IndexAddr,(ChP)->TxControl); \
} while (0)

#define sSetTxXOFFChar(ChP,CH) \
do { \
   (ChP)->R[0x07] = (CH); \
   out32((ChP)->IndexAddr,&(ChP)->R[0x04]); \
} while (0)

#define sSetTxXONChar(ChP,CH) \
do { \
   (ChP)->R[0x0b] = (CH); \
   out32((ChP)->IndexAddr,&(ChP)->R[0x08]); \
} while (0)

#define sStartRxProcessor(ChP) out32((ChP)->IndexAddr,&(ChP)->R[0])

#define sWriteTxByte(IO,DATA) sOutB(IO,DATA)


struct r_port {
	int magic;
	struct tty_port port;
	int line;
	int flags;		/* Don't yet match the ASY_ flags!! */
	unsigned int board:3;
	unsigned int aiop:2;
	unsigned int chan:3;
	CONTROLLER_t *ctlp;
	CHANNEL_t channel;
	int intmask;
	int xmit_fifo_room;	/* room in xmit fifo */
	unsigned char *xmit_buf;
	int xmit_head;
	int xmit_tail;
	int xmit_cnt;
	int cd_status;
	int ignore_status_mask;
	int read_status_mask;
	int cps;

	struct completion close_wait;	/* Not yet matching the core */
	spinlock_t slock;
	struct mutex write_mtx;
};

#define RPORT_MAGIC 0x525001

#define NUM_BOARDS 8
#define MAX_RP_PORTS (32*NUM_BOARDS)

#define XMIT_BUF_SIZE 4096

/* number of characters left in xmit buffer before we ask for more */
#define WAKEUP_CHARS 256

#define TTY_ROCKET_MAJOR	46
#define CUA_ROCKET_MAJOR	47

#ifdef PCI_VENDOR_ID_RP
#undef PCI_VENDOR_ID_RP
#undef PCI_DEVICE_ID_RP8OCTA
#undef PCI_DEVICE_ID_RP8INTF
#undef PCI_DEVICE_ID_RP16INTF
#undef PCI_DEVICE_ID_RP32INTF
#undef PCI_DEVICE_ID_URP8OCTA
#undef PCI_DEVICE_ID_URP8INTF
#undef PCI_DEVICE_ID_URP16INTF
#undef PCI_DEVICE_ID_CRP16INTF
#undef PCI_DEVICE_ID_URP32INTF
#endif

/*  Comtrol PCI Vendor ID */
#define PCI_VENDOR_ID_RP		0x11fe

/*  Comtrol Device ID's */
#define PCI_DEVICE_ID_RP32INTF		0x0001	/* Rocketport 32 port w/external I/F     */
#define PCI_DEVICE_ID_RP8INTF		0x0002	/* Rocketport 8 port w/external I/F      */
#define PCI_DEVICE_ID_RP16INTF		0x0003	/* Rocketport 16 port w/external I/F     */
#define PCI_DEVICE_ID_RP4QUAD		0x0004	/* Rocketport 4 port w/quad cable        */
#define PCI_DEVICE_ID_RP8OCTA		0x0005	/* Rocketport 8 port w/octa cable        */
#define PCI_DEVICE_ID_RP8J		0x0006	/* Rocketport 8 port w/RJ11 connectors   */
#define PCI_DEVICE_ID_RP4J		0x0007	/* Rocketport 4 port w/RJ11 connectors   */
#define PCI_DEVICE_ID_RP8SNI		0x0008	/* Rocketport 8 port w/ DB78 SNI (Siemens) connector */
#define PCI_DEVICE_ID_RP16SNI		0x0009	/* Rocketport 16 port w/ DB78 SNI (Siemens) connector   */
#define PCI_DEVICE_ID_RPP4		0x000A	/* Rocketport Plus 4 port                */
#define PCI_DEVICE_ID_RPP8		0x000B	/* Rocketport Plus 8 port                */
#define PCI_DEVICE_ID_RP6M		0x000C	/* RocketModem 6 port                    */
#define PCI_DEVICE_ID_RP4M		0x000D	/* RocketModem 4 port                    */
#define PCI_DEVICE_ID_RP2_232           0x000E	/* Rocketport Plus 2 port RS232          */
#define PCI_DEVICE_ID_RP2_422           0x000F	/* Rocketport Plus 2 port RS422          */ 

/* Universal PCI boards  */
#define PCI_DEVICE_ID_URP32INTF		0x0801	/* Rocketport UPCI 32 port w/external I/F */ 
#define PCI_DEVICE_ID_URP8INTF		0x0802	/* Rocketport UPCI 8 port w/external I/F  */
#define PCI_DEVICE_ID_URP16INTF		0x0803	/* Rocketport UPCI 16 port w/external I/F */
#define PCI_DEVICE_ID_URP8OCTA		0x0805	/* Rocketport UPCI 8 port w/octa cable    */
#define PCI_DEVICE_ID_UPCI_RM3_8PORT    0x080C	/* Rocketmodem III 8 port                 */
#define PCI_DEVICE_ID_UPCI_RM3_4PORT    0x080D	/* Rocketmodem III 4 port                 */

/* Compact PCI device */ 
#define PCI_DEVICE_ID_CRP16INTF		0x0903	/* Rocketport Compact PCI 16 port w/external I/F */

