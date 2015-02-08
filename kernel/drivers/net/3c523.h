
#ifndef _3c523_INCLUDE_
#define _3c523_INCLUDE_


#define SCP_DEFAULT_ADDRESS 0xfffff4



struct scp_struct
{
  unsigned short zero_dum0;	/* has to be zero */
  unsigned char  sysbus;	/* 0=16Bit,1=8Bit */
  unsigned char  zero_dum1;	/* has to be zero for 586 */
  unsigned short zero_dum2;
  unsigned short zero_dum3;
  char          *iscp;		/* pointer to the iscp-block */
};


struct iscp_struct
{
  unsigned char  busy;          /* 586 clears after successful init */
  unsigned char  zero_dummy;    /* hast to be zero */
  unsigned short scb_offset;    /* pointeroffset to the scb_base */
  char          *scb_base;      /* base-address of all 16-bit offsets */
};

struct scb_struct
{
  unsigned short status;        /* status word */
  unsigned short cmd;           /* command word */
  unsigned short cbl_offset;    /* pointeroffset, command block list */
  unsigned short rfa_offset;    /* pointeroffset, receive frame area */
  unsigned short crc_errs;      /* CRC-Error counter */
  unsigned short aln_errs;      /* alignmenterror counter */
  unsigned short rsc_errs;      /* Resourceerror counter */
  unsigned short ovrn_errs;     /* OVerrunerror counter */
};

#define RUC_MASK	0x0070	/* mask for RU commands */
#define RUC_NOP		0x0000	/* NOP-command */
#define RUC_START	0x0010	/* start RU */
#define RUC_RESUME	0x0020	/* resume RU after suspend */
#define RUC_SUSPEND	0x0030	/* suspend RU */
#define RUC_ABORT	0x0040	/* abort receiver operation immediately */

#define CUC_MASK	0x0700	/* mask for CU command */
#define CUC_NOP		0x0000	/* NOP-command */
#define CUC_START	0x0100	/* start execution of 1. cmd on the CBL */
#define CUC_RESUME	0x0200	/* resume after suspend */
#define CUC_SUSPEND	0x0300	/* Suspend CU */
#define CUC_ABORT	0x0400	/* abort command operation immediately */

#define ACK_MASK	0xf000	/* mask for ACK command */
#define ACK_CX		0x8000	/* acknowledges STAT_CX */
#define ACK_FR		0x4000	/* ack. STAT_FR */
#define ACK_CNA		0x2000	/* ack. STAT_CNA */
#define ACK_RNR		0x1000	/* ack. STAT_RNR */

#define STAT_MASK	0xf000	/* mask for cause of interrupt */
#define STAT_CX		0x8000	/* CU finished cmd with its I bit set */
#define STAT_FR		0x4000	/* RU finished receiving a frame */
#define STAT_CNA	0x2000	/* CU left active state */
#define STAT_RNR	0x1000	/* RU left ready state */

#define CU_STATUS	0x700	/* CU status, 0=idle */
#define CU_SUSPEND	0x100	/* CU is suspended */
#define CU_ACTIVE	0x200	/* CU is active */

#define RU_STATUS	0x70	/* RU status, 0=idle */
#define RU_SUSPEND	0x10	/* RU suspended */
#define RU_NOSPACE	0x20	/* RU no resources */
#define RU_READY	0x40	/* RU is ready */

struct rfd_struct
{
  unsigned short status;	/* status word */
  unsigned short last;		/* Bit15,Last Frame on List / Bit14,suspend */
  unsigned short next;		/* linkoffset to next RFD */
  unsigned short rbd_offset;	/* pointeroffset to RBD-buffer */
  unsigned char  dest[6];	/* ethernet-address, destination */
  unsigned char  source[6];	/* ethernet-address, source */
  unsigned short length;	/* 802.3 frame-length */
  unsigned short zero_dummy;	/* dummy */
};

#define RFD_LAST     0x8000	/* last: last rfd in the list */
#define RFD_SUSP     0x4000	/* last: suspend RU after  */
#define RFD_ERRMASK  0x0fe1     /* status: errormask */
#define RFD_MATCHADD 0x0002     /* status: Destinationaddress !matches IA */
#define RFD_RNR      0x0200	/* status: receiver out of resources */

struct rbd_struct
{
  unsigned short status;	/* status word,number of used bytes in buff */
  unsigned short next;		/* pointeroffset to next RBD */
  char          *buffer;	/* receive buffer address pointer */
  unsigned short size;		/* size of this buffer */
  unsigned short zero_dummy;    /* dummy */
};

#define RBD_LAST	0x8000	/* last buffer */
#define RBD_USED	0x4000	/* this buffer has data */
#define RBD_MASK	0x3fff	/* size-mask for length */

#define STAT_COMPL   0x8000	/* status: frame/command is complete */
#define STAT_BUSY    0x4000	/* status: frame/command is busy */
#define STAT_OK      0x2000	/* status: frame/command is ok */

#define CMD_NOP		0x0000	/* NOP */
#define CMD_IASETUP	0x0001	/* initial address setup command */
#define CMD_CONFIGURE	0x0002	/* configure command */
#define CMD_MCSETUP	0x0003	/* MC setup command */
#define CMD_XMIT	0x0004	/* transmit command */
#define CMD_TDR		0x0005	/* time domain reflectometer (TDR) command */
#define CMD_DUMP	0x0006	/* dump command */
#define CMD_DIAGNOSE	0x0007	/* diagnose command */

#define CMD_LAST	0x8000	/* indicates last command in the CBL */
#define CMD_SUSPEND	0x4000	/* suspend CU after this CB */
#define CMD_INT		0x2000	/* generate interrupt after execution */

struct nop_cmd_struct
{
  unsigned short cmd_status;	/* status of this command */
  unsigned short cmd_cmd;       /* the command itself (+bits) */
  unsigned short cmd_link;      /* offsetpointer to next command */
};

struct iasetup_cmd_struct
{
  unsigned short cmd_status;
  unsigned short cmd_cmd;
  unsigned short cmd_link;
  unsigned char  iaddr[6];
};

struct configure_cmd_struct
{
  unsigned short cmd_status;
  unsigned short cmd_cmd;
  unsigned short cmd_link;
  unsigned char  byte_cnt;   /* size of the config-cmd */
  unsigned char  fifo;       /* fifo/recv monitor */
  unsigned char  sav_bf;     /* save bad frames (bit7=1)*/
  unsigned char  adr_len;    /* adr_len(0-2),al_loc(3),pream(4-5),loopbak(6-7)*/
  unsigned char  priority;   /* lin_prio(0-2),exp_prio(4-6),bof_metd(7) */
  unsigned char  ifs;        /* inter frame spacing */
  unsigned char  time_low;   /* slot time low */
  unsigned char  time_high;  /* slot time high(0-2) and max. retries(4-7) */
  unsigned char  promisc;    /* promisc-mode(0) , et al (1-7) */
  unsigned char  carr_coll;  /* carrier(0-3)/collision(4-7) stuff */
  unsigned char  fram_len;   /* minimal frame len */
  unsigned char  dummy;	     /* dummy */
};

struct mcsetup_cmd_struct
{
  unsigned short cmd_status;
  unsigned short cmd_cmd;
  unsigned short cmd_link;
  unsigned short mc_cnt;		/* number of bytes in the MC-List */
  unsigned char  mc_list[0][6];  	/* pointer to 6 bytes entries */
};

struct transmit_cmd_struct
{
  unsigned short cmd_status;
  unsigned short cmd_cmd;
  unsigned short cmd_link;
  unsigned short tbd_offset;	/* pointeroffset to TBD */
  unsigned char  dest[6];       /* destination address of the frame */
  unsigned short length;	/* user defined: 802.3 length / Ether type */
};

#define TCMD_ERRMASK     0x0fa0
#define TCMD_MAXCOLLMASK 0x000f
#define TCMD_MAXCOLL     0x0020
#define TCMD_HEARTBEAT   0x0040
#define TCMD_DEFERRED    0x0080
#define TCMD_UNDERRUN    0x0100
#define TCMD_LOSTCTS     0x0200
#define TCMD_NOCARRIER   0x0400
#define TCMD_LATECOLL    0x0800

struct tdr_cmd_struct
{
  unsigned short cmd_status;
  unsigned short cmd_cmd;
  unsigned short cmd_link;
  unsigned short status;
};

#define TDR_LNK_OK	0x8000	/* No link problem identified */
#define TDR_XCVR_PRB	0x4000	/* indicates a transceiver problem */
#define TDR_ET_OPN	0x2000	/* open, no correct termination */
#define TDR_ET_SRT	0x1000	/* TDR detected a short circuit */
#define TDR_TIMEMASK	0x07ff	/* mask for the time field */

struct tbd_struct
{
  unsigned short size;		/* size + EOF-Flag(15) */
  unsigned short next;          /* pointeroffset to next TBD */
  char          *buffer;        /* pointer to buffer */
};

#define TBD_LAST 0x8000         /* EOF-Flag, indicates last buffer in list */

/*************************************************************************/

/* Offsets from the base I/O address. */
#define	ELMC_SA		0	/* first 6 bytes are IEEE network address */
#define ELMC_CTRL	6	/* control & status register */
#define ELMC_REVISION	7	/* revision register, first 4 bits only */
#define ELMC_IO_EXTENT  8

/* these are the bit selects for the port register 2 */
#define ELMC_STATUS_ENABLED	0x01
#define ELMC_STATUS_CSR_SELECT	0x06
#define ELMC_STATUS_MEMORY_SELECT	0x18
#define ELMC_STATUS_DISABLE_THIN	0x20
#define ELMC_STATUS_IRQ_SELECT	0xc0

#define ELMC_MCA_ID 0x6042

#define ELMC_CTRL_BS0	0x01	/* RW bank select */
#define ELMC_CTRL_BS1	0x02	/* RW bank select */
#define ELMC_CTRL_INTE	0x04	/* RW interrupt enable, assert high */
#define ELMC_CTRL_INT	0x08	/* R interrupt active, assert high */
/*#define ELMC_CTRL_*	0x10*/	/* reserved */
#define ELMC_CTRL_LBK	0x20	/* RW loopback enable, assert high */
#define ELMC_CTRL_CA	0x40	/* RW channel attention, assert high */
#define ELMC_CTRL_RST	0x80	/* RW 82586 reset, assert low */

/* some handy compound bits */

/* normal operation should have bank 3 and RST high, ints enabled */
#define ELMC_NORMAL (ELMC_CTRL_INTE|ELMC_CTRL_RST|0x3)

#endif /* _3c523_INCLUDE_ */
