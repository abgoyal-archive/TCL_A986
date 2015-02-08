



struct lance_regs {
	unsigned short rdp;		/* Register Data Port */
	unsigned short rap;		/* Register Address Port */
};



#define LE_CSR0		0x0000		/* LANCE Controller Status */
#define LE_CSR1		0x0001		/* IADR[15:0] */
#define LE_CSR2		0x0002		/* IADR[23:16] */
#define LE_CSR3		0x0003		/* Misc */



#define LE_C0_ERR	0x8000		/* Error */
#define LE_C0_BABL	0x4000		/* Babble: Transmitted too many bits */
#define LE_C0_CERR	0x2000		/* No Heartbeat (10BASE-T) */
#define LE_C0_MISS	0x1000		/* Missed Frame */
#define LE_C0_MERR	0x0800		/* Memory Error */
#define LE_C0_RINT	0x0400		/* Receive Interrupt */
#define LE_C0_TINT	0x0200		/* Transmit Interrupt */
#define LE_C0_IDON	0x0100		/* Initialization Done */
#define LE_C0_INTR	0x0080		/* Interrupt Flag */
#define LE_C0_INEA	0x0040		/* Interrupt Enable */
#define LE_C0_RXON	0x0020		/* Receive On */
#define LE_C0_TXON	0x0010		/* Transmit On */
#define LE_C0_TDMD	0x0008		/* Transmit Demand */
#define LE_C0_STOP	0x0004		/* Stop */
#define LE_C0_STRT	0x0002		/* Start */
#define LE_C0_INIT	0x0001		/* Initialize */



#define LE_C3_BSWP	0x0004		/* Byte Swap
					   (on for big endian byte order) */
#define LE_C3_ACON	0x0002		/* ALE Control
					   (on for active low ALE) */
#define LE_C3_BCON	0x0001		/* Byte Control */



#define LE_MO_PROM	0x8000		/* Promiscuous Mode */
#define LE_MO_INTL	0x0040		/* Internal Loopback */
#define LE_MO_DRTY	0x0020		/* Disable Retry */
#define LE_MO_FCOLL	0x0010		/* Force Collision */
#define LE_MO_DXMTFCS	0x0008		/* Disable Transmit CRC */
#define LE_MO_LOOP	0x0004		/* Loopback Enable */
#define LE_MO_DTX	0x0002		/* Disable Transmitter */
#define LE_MO_DRX	0x0001		/* Disable Receiver */


struct lance_rx_desc {
	unsigned short rmd0;        /* low address of packet */
	unsigned char  rmd1_bits;   /* descriptor bits */
	unsigned char  rmd1_hadr;   /* high address of packet */
	short    length;    	    /* This length is 2s complement (negative)!
				     * Buffer length
				     */
	unsigned short mblength;    /* Aactual number of bytes received */
};

struct lance_tx_desc {
	unsigned short tmd0;        /* low address of packet */
	unsigned char  tmd1_bits;   /* descriptor bits */
	unsigned char  tmd1_hadr;   /* high address of packet */
	short    length;       	    /* Length is 2s complement (negative)! */
	unsigned short misc;
};



#define LE_R1_OWN	0x80		/* LANCE owns the descriptor */
#define LE_R1_ERR	0x40		/* Error */
#define LE_R1_FRA	0x20		/* Framing Error */
#define LE_R1_OFL	0x10		/* Overflow Error */
#define LE_R1_CRC	0x08		/* CRC Error */
#define LE_R1_BUF	0x04		/* Buffer Error */
#define LE_R1_SOP	0x02		/* Start of Packet */
#define LE_R1_EOP	0x01		/* End of Packet */
#define LE_R1_POK       0x03		/* Packet is complete: SOP + EOP */



#define LE_T1_OWN	0x80		/* LANCE owns the descriptor */
#define LE_T1_ERR	0x40		/* Error */
#define LE_T1_RES	0x20		/* Reserved,
					   LANCE writes this with a zero */
#define LE_T1_EMORE	0x10		/* More than one retry needed */
#define LE_T1_EONE	0x08		/* One retry needed */
#define LE_T1_EDEF	0x04		/* Deferred */
#define LE_T1_SOP	0x02		/* Start of Packet */
#define LE_T1_EOP	0x01		/* End of Packet */
#define LE_T1_POK	0x03		/* Packet is complete: SOP + EOP */



#define LE_T3_BUF 	0x8000		/* Buffer Error */
#define LE_T3_UFL 	0x4000		/* Underflow Error */
#define LE_T3_LCOL 	0x1000		/* Late Collision */
#define LE_T3_CLOS 	0x0800		/* Loss of Carrier */
#define LE_T3_RTY 	0x0400		/* Retry Error */
#define LE_T3_TDR	0x03ff		/* Time Domain Reflectometry */



#define A2065_LANCE		0x4000

#define A2065_RAM		0x8000
#define A2065_RAM_SIZE		0x8000

