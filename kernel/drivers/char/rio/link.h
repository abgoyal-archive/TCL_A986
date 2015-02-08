

#ifndef _link_h
#define _link_h 1

/* Boot request stuff */
#define BOOT_REQUEST       ((ushort) 0)	/* Request for a boot */
#define BOOT_ABORT         ((ushort) 1)	/* Abort a boot */
#define BOOT_SEQUENCE      ((ushort) 2)	/* Packet with the number of packets
					   and load address */
#define BOOT_COMPLETED     ((ushort) 3)	/* Boot completed */


struct LPB {
	u16 link_number;	/* Link Number */
	u16 in_ch;	/* Link In Channel */
	u16 out_ch;	/* Link Out Channel */
	u8 attached_serial[4];  /* Attached serial number */
	u8 attached_host_serial[4];
	/* Serial number of Host who
	   booted the other end */
	u16 descheduled;	/* Currently Descheduled */
	u16 state;		/* Current state */
	u16 send_poll;		/* Send a Poll Packet */
	u16 ltt_p;	/* Process Descriptor */
	u16 lrt_p;	/* Process Descriptor */
	u16 lrt_status;		/* Current lrt status */
	u16 ltt_status;		/* Current ltt status */
	u16 timeout;		/* Timeout value */
	u16 topology;		/* Topology bits */
	u16 mon_ltt;
	u16 mon_lrt;
	u16 WaitNoBoot;	/* Secs to hold off booting */
	u16 add_packet_list;	/* Add packets to here */
	u16 remove_packet_list;	/* Send packets from here */

	u16 lrt_fail_chan;	/* Lrt's failure channel */
	u16 ltt_fail_chan;	/* Ltt's failure channel */

	/* RUP structure for HOST to driver communications */
	struct RUP rup;
	struct RUP link_rup;	/* RUP for the link (POLL,
				   topology etc.) */
	u16 attached_link;	/* Number of attached link */
	u16 csum_errors;	/* csum errors */
	u16 num_disconnects;	/* number of disconnects */
	u16 num_sync_rcvd;	/* # sync's received */
	u16 num_sync_rqst;	/* # sync requests */
	u16 num_tx;		/* Num pkts sent */
	u16 num_rx;		/* Num pkts received */
	u16 module_attached;	/* Module tpyes of attached */
	u16 led_timeout;	/* LED timeout */
	u16 first_port;		/* First port to service */
	u16 last_port;		/* Last port to service */
};

#endif

/*********** end of file ***********/
