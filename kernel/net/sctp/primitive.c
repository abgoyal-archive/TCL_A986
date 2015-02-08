

#include <linux/types.h>
#include <linux/list.h> /* For struct list_head */
#include <linux/socket.h>
#include <linux/ip.h>
#include <linux/time.h> /* For struct timeval */
#include <linux/gfp.h>
#include <net/sock.h>
#include <net/sctp/sctp.h>
#include <net/sctp/sm.h>

#define DECLARE_PRIMITIVE(name) \
/* This is called in the code as sctp_primitive_ ## name.  */ \
int sctp_primitive_ ## name(struct sctp_association *asoc, \
			    void *arg) { \
	int error = 0; \
	sctp_event_t event_type; sctp_subtype_t subtype; \
	sctp_state_t state; \
	struct sctp_endpoint *ep; \
	\
	event_type = SCTP_EVENT_T_PRIMITIVE; \
	subtype = SCTP_ST_PRIMITIVE(SCTP_PRIMITIVE_ ## name); \
	state = asoc ? asoc->state : SCTP_STATE_CLOSED; \
	ep = asoc ? asoc->ep : NULL; \
	\
	error = sctp_do_sm(event_type, subtype, state, ep, asoc, \
			   arg, GFP_KERNEL); \
	return error; \
}


/* This is called in the code as sctp_primitive_ASSOCIATE.  */

DECLARE_PRIMITIVE(ASSOCIATE)


DECLARE_PRIMITIVE(SHUTDOWN);


DECLARE_PRIMITIVE(ABORT);


DECLARE_PRIMITIVE(SEND);


DECLARE_PRIMITIVE(REQUESTHEARTBEAT);


DECLARE_PRIMITIVE(ASCONF);
