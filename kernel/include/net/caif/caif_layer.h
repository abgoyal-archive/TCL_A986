

#ifndef CAIF_LAYER_H_
#define CAIF_LAYER_H_

#include <linux/list.h>

struct cflayer;
struct cfpkt;
struct cfpktq;
struct caif_payload_info;
struct caif_packet_funcs;

#define CAIF_MAX_FRAMESIZE 4096
#define CAIF_MAX_PAYLOAD_SIZE (4096 - 64)
#define CAIF_NEEDED_HEADROOM (10)
#define CAIF_NEEDED_TAILROOM (2)

#define CAIF_LAYER_NAME_SZ 16
#define CAIF_SUCCESS	1
#define CAIF_FAILURE	0

#define caif_assert(assert)					\
do {								\
	if (!(assert)) {					\
		pr_err("caif:Assert detected:'%s'\n", #assert); \
		WARN_ON(!(assert));				\
	}							\
} while (0)


enum caif_ctrlcmd {
	CAIF_CTRLCMD_FLOW_OFF_IND,
	CAIF_CTRLCMD_FLOW_ON_IND,
	CAIF_CTRLCMD_REMOTE_SHUTDOWN_IND,
	CAIF_CTRLCMD_INIT_RSP,
	CAIF_CTRLCMD_DEINIT_RSP,
	CAIF_CTRLCMD_INIT_FAIL_RSP,
	_CAIF_CTRLCMD_PHYIF_FLOW_OFF_IND,
	_CAIF_CTRLCMD_PHYIF_FLOW_ON_IND,
	_CAIF_CTRLCMD_PHYIF_DOWN_IND,
};

enum caif_modemcmd {
	CAIF_MODEMCMD_FLOW_ON_REQ = 0,
	CAIF_MODEMCMD_FLOW_OFF_REQ = 1,
	_CAIF_MODEMCMD_PHYIF_USEFULL = 3,
	_CAIF_MODEMCMD_PHYIF_USELESS = 4
};

enum caif_direction {
	CAIF_DIR_IN = 0,
	CAIF_DIR_OUT = 1
};

struct cflayer {
	struct cflayer *up;
	struct cflayer *dn;
	struct list_head node;

	/*
	 *  receive() - Receive Function.
	 *  Contract: Each layer must implement a receive function passing the
	 *  CAIF packets upwards in the stack.
	 *	Packet handling rules:
	 *	      - The CAIF packet (cfpkt) cannot be accessed after
	 *		     passing it to the next layer using up->receive().
	 *	      - If parsing of the packet fails, the packet must be
	 *		     destroyed and -1 returned from the function.
	 *	      - If parsing succeeds (and above layers return OK) then
	 *		      the function must return a value > 0.
	 *
	 *  Returns result < 0 indicates an error, 0 or positive value
	 *	     indicates success.
	 *
	 *  @layr: Pointer to the current layer the receive function is
	 *		implemented for (this pointer).
	 *  @cfpkt: Pointer to CaifPacket to be handled.
	 */
	int (*receive)(struct cflayer *layr, struct cfpkt *cfpkt);

	/*
	 *  transmit() - Transmit Function.
	 *  Contract: Each layer must implement a transmit function passing the
	 *	CAIF packet downwards in the stack.
	 *	Packet handling rules:
	 *	      - The CAIF packet (cfpkt) ownership is passed to the
	 *		transmit function. This means that the the packet
	 *		cannot be accessed after passing it to the below
	 *		layer using dn->transmit().
	 *
	 *	      - If transmit fails, however, the ownership is returned
	 *		to thecaller. The caller of "dn->transmit()" must
	 *		destroy or resend packet.
	 *
	 *	      - Return value less than zero means error, zero or
	 *		greater than zero means OK.
	 *
	 *	 result < 0 indicates an error, 0 or positive value
	 *	 indicate success.
	 *
	 *  @layr:	Pointer to the current layer the receive function
	 *		isimplemented for (this pointer).
	 *  @cfpkt:	 Pointer to CaifPacket to be handled.
	 */
	int (*transmit) (struct cflayer *layr, struct cfpkt *cfpkt);

	/*
	 *  cttrlcmd() - Control Function upwards in CAIF Stack.
	 *  Used for signaling responses (CAIF_CTRLCMD_*_RSP)
	 *  and asynchronous events from the modem  (CAIF_CTRLCMD_*_IND)
	 *
	 *  @layr:	Pointer to the current layer the receive function
	 *		is implemented for (this pointer).
	 *  @ctrl:	Control Command.
	 */
	void (*ctrlcmd) (struct cflayer *layr, enum caif_ctrlcmd ctrl,
			 int phyid);

	/*
	 *  modemctrl() - Control Function used for controlling the modem.
	 *  Used to signal down-wards in the CAIF stack.
	 *  Returns 0 on success, < 0 upon failure.
	 *
	 *  @layr:	Pointer to the current layer the receive function
	 *		is implemented for (this pointer).
	 *  @ctrl:  Control Command.
	 */
	int (*modemcmd) (struct cflayer *layr, enum caif_modemcmd ctrl);

	unsigned short prio;
	unsigned int id;
	unsigned int type;
	char name[CAIF_LAYER_NAME_SZ];
};

#define layer_set_up(layr, above) ((layr)->up = (struct cflayer *)(above))

#define layer_set_dn(layr, below) ((layr)->dn = (struct cflayer *)(below))

struct dev_info {
	void *dev;
	unsigned int id;
};

struct caif_payload_info {
	struct dev_info *dev_info;
	unsigned short hdr_len;
	unsigned short channel_id;
};

#endif	/* CAIF_LAYER_H_ */
