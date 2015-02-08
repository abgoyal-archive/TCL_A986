

#ifndef _rup_h
#define _rup_h 1

#define MAX_RUP          ((short) 16)
#define PKTS_PER_RUP     ((short) 2)	/* They are always used in pairs */

#define TX_RUP_INACTIVE          0	/* Nothing to transmit */
#define TX_PACKET_READY          1	/* Transmit packet ready */
#define TX_LOCK_RUP              2	/* Transmit side locked */

#define RX_RUP_INACTIVE          0	/* Nothing received */
#define RX_PACKET_READY          1	/* Packet received */

#define RUP_NO_OWNER             0xff	/* RUP not owned by any process */

struct RUP {
	u16 txpkt;		/* Outgoing packet */
	u16 rxpkt;		/* Incoming packet */
	u16 link;		/* Which link to send down? */
	u8 rup_dest_unit[2];	/* Destination unit */
	u16 handshake;		/* For handshaking */
	u16 timeout;		/* Timeout */
	u16 status;		/* Status */
	u16 txcontrol;		/* Transmit control */
	u16 rxcontrol;		/* Receive control */
};

#endif

/*********** end of file ***********/
