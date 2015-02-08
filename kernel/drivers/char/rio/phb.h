

#ifndef _phb_h
#define _phb_h 1

#define PHB_HANDSHAKE_SET      ((ushort) 0x001)	/* Set by LRT */

#define PHB_HANDSHAKE_RESET     ((ushort) 0x002)	/* Set by ISR / driver */

#define PHB_HANDSHAKE_FLAGS     (PHB_HANDSHAKE_RESET | PHB_HANDSHAKE_SET)
						/* Reset by ltt */


#define MAX_PHB               ((ushort) 128)	/* range 0-127 */

#define TXPKT_INCOMPLETE        0x0001	/* Previous tx packet not completed */
#define TXINTR_ENABLED          0x0002	/* Tx interrupt is enabled */
#define TX_TAB3                 0x0004	/* TAB3 mode */
#define TX_OCRNL                0x0008	/* OCRNL mode */
#define TX_ONLCR                0x0010	/* ONLCR mode */
#define TX_SENDSPACES           0x0020	/* Send n spaces command needs
					   completing */
#define TX_SENDNULL             0x0040	/* Escaping NULL needs completing */
#define TX_SENDLF               0x0080	/* LF -> CR LF needs completing */
#define TX_PARALLELBUG          0x0100	/* CD1400 LF -> CR LF bug on parallel
					   port */
#define TX_HANGOVER             (TX_SENDSPACES | TX_SENDLF | TX_SENDNULL)
#define TX_DTRFLOW		0x0200	/* DTR tx flow control */
#define	TX_DTRFLOWED		0x0400	/* DTR is low - don't allow more data
					   into the FIFO */
#define	TX_DATAINFIFO		0x0800	/* There is data in the FIFO */
#define	TX_BUSY			0x1000	/* Data in FIFO, shift or holding regs */

#define RX_SPARE	        0x0001	/* SPARE */
#define RXINTR_ENABLED          0x0002	/* Rx interrupt enabled */
#define RX_ICRNL                0x0008	/* ICRNL mode */
#define RX_INLCR                0x0010	/* INLCR mode */
#define RX_IGNCR                0x0020	/* IGNCR mode */
#define RX_CTSFLOW              0x0040	/* CTSFLOW enabled */
#define RX_IXOFF                0x0080	/* IXOFF enabled */
#define RX_CTSFLOWED            0x0100	/* CTSFLOW and CTS dropped */
#define RX_IXOFFED              0x0200	/* IXOFF and xoff sent */
#define RX_BUFFERED		0x0400	/* Try and pass on complete packets */

#define PORT_ISOPEN             0x0001	/* Port open? */
#define PORT_HUPCL              0x0002	/* Hangup on close? */
#define PORT_MOPENPEND          0x0004	/* Modem open pending */
#define PORT_ISPARALLEL         0x0008	/* Parallel port */
#define PORT_BREAK              0x0010	/* Port on break */
#define PORT_STATUSPEND		0x0020	/* Status packet pending */
#define PORT_BREAKPEND          0x0040	/* Break packet pending */
#define PORT_MODEMPEND          0x0080	/* Modem status packet pending */
#define PORT_PARALLELBUG        0x0100	/* CD1400 LF -> CR LF bug on parallel
					   port */
#define PORT_FULLMODEM          0x0200	/* Full modem signals */
#define PORT_RJ45               0x0400	/* RJ45 connector - no RI signal */
#define PORT_RESTRICTED         0x0600	/* Restricted connector - no RI / DTR */

#define PORT_MODEMBITS          0x0600	/* Mask for modem fields */

#define PORT_WCLOSE             0x0800	/* Waiting for close */
#define	PORT_HANDSHAKEFIX	0x1000	/* Port has H/W flow control fix */
#define	PORT_WASPCLOSED		0x2000	/* Port closed with PCLOSE */
#define	DUMPMODE		0x4000	/* Dump RTA mem */
#define	READ_REG		0x8000	/* Read CD1400 register */




struct PHB {
	u8 source;
	u8 handshake;
	u8 status;
	u16 timeout;		/* Maximum of 1.9 seconds */
	u8 link;		/* Send down this link */
	u8 destination;
	u16 tx_start;
	u16 tx_end;
	u16 tx_add;
	u16 tx_remove;

	u16 rx_start;
	u16 rx_end;
	u16 rx_add;
	u16 rx_remove;

};

#endif

/*********** end of file ***********/
