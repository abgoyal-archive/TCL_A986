

#ifndef _IPWIRELESS_CS_SETUP_PROTOCOL_H_
#define _IPWIRELESS_CS_SETUP_PROTOCOL_H_

/* Version of the setup protocol and transport protocols */
#define TL_SETUP_VERSION		1

#define TL_SETUP_VERSION_QRY_TMO	1000
#define TL_SETUP_MAX_VERSION_QRY	30

/* Message numbers 0-9 are obsoleted and must not be reused! */
#define TL_SETUP_SIGNO_GET_VERSION_QRY	10
#define TL_SETUP_SIGNO_GET_VERSION_RSP	11
#define TL_SETUP_SIGNO_CONFIG_MSG	12
#define TL_SETUP_SIGNO_CONFIG_DONE_MSG	13
#define TL_SETUP_SIGNO_OPEN_MSG		14
#define TL_SETUP_SIGNO_CLOSE_MSG	15

#define TL_SETUP_SIGNO_INFO_MSG     20
#define TL_SETUP_SIGNO_INFO_MSG_ACK 21

#define TL_SETUP_SIGNO_REBOOT_MSG      22
#define TL_SETUP_SIGNO_REBOOT_MSG_ACK  23

/* Synchronous start-messages */
struct tl_setup_get_version_qry {
	unsigned char sig_no;		/* TL_SETUP_SIGNO_GET_VERSION_QRY */
} __attribute__ ((__packed__));

struct tl_setup_get_version_rsp {
	unsigned char sig_no;		/* TL_SETUP_SIGNO_GET_VERSION_RSP */
	unsigned char version;		/* TL_SETUP_VERSION */
} __attribute__ ((__packed__));

struct tl_setup_config_msg {
	unsigned char sig_no;		/* TL_SETUP_SIGNO_CONFIG_MSG */
	unsigned char port_no;
	unsigned char prio_data;
	unsigned char prio_ctrl;
} __attribute__ ((__packed__));

struct tl_setup_config_done_msg {
	unsigned char sig_no;		/* TL_SETUP_SIGNO_CONFIG_DONE_MSG */
} __attribute__ ((__packed__));

/* Asyncronous messages */
struct tl_setup_open_msg {
	unsigned char sig_no;		/* TL_SETUP_SIGNO_OPEN_MSG */
	unsigned char port_no;
} __attribute__ ((__packed__));

struct tl_setup_close_msg {
	unsigned char sig_no;		/* TL_SETUP_SIGNO_CLOSE_MSG */
	unsigned char port_no;
} __attribute__ ((__packed__));

/* Driver type  - for use in tl_setup_info_msg.driver_type */
#define COMM_DRIVER     0
#define NDISWAN_DRIVER  1
#define NDISWAN_DRIVER_MAJOR_VERSION  2
#define NDISWAN_DRIVER_MINOR_VERSION  0

struct tl_setup_info_msg {
	unsigned char sig_no;		/* TL_SETUP_SIGNO_INFO_MSG */
	unsigned char driver_type;
	unsigned char major_version;
	unsigned char minor_version;
} __attribute__ ((__packed__));

struct tl_setup_info_msgAck {
	unsigned char sig_no;		/* TL_SETUP_SIGNO_INFO_MSG_ACK */
} __attribute__ ((__packed__));

struct TlSetupRebootMsgAck {
	unsigned char sig_no;		/* TL_SETUP_SIGNO_REBOOT_MSG_ACK */
} __attribute__ ((__packed__));

/* Define a union of all the msgs that the driver can receive from the card.*/
union ipw_setup_rx_msg {
	unsigned char sig_no;
	struct tl_setup_get_version_rsp version_rsp_msg;
	struct tl_setup_open_msg open_msg;
	struct tl_setup_close_msg close_msg;
	struct tl_setup_info_msg InfoMsg;
	struct tl_setup_info_msgAck info_msg_ack;
} __attribute__ ((__packed__));

#endif				/* _IPWIRELESS_CS_SETUP_PROTOCOL_H_ */
