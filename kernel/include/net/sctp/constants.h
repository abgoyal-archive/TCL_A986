

#ifndef __sctp_constants_h__
#define __sctp_constants_h__

#include <linux/sctp.h>
#include <linux/ipv6.h> /* For ipv6hdr. */
#include <net/sctp/user.h>
#include <net/tcp_states.h>  /* For TCP states used in sctp_sock_state_t */

/* Value used for stream negotiation. */
enum { SCTP_MAX_STREAM = 0xffff };
enum { SCTP_DEFAULT_OUTSTREAMS = 10 };
enum { SCTP_DEFAULT_INSTREAMS = SCTP_MAX_STREAM };

#define SCTP_CID_BASE_MAX		SCTP_CID_SHUTDOWN_COMPLETE
#define SCTP_CID_MAX			SCTP_CID_ASCONF_ACK

#define SCTP_NUM_BASE_CHUNK_TYPES	(SCTP_CID_BASE_MAX + 1)

#define SCTP_NUM_ADDIP_CHUNK_TYPES	2

#define SCTP_NUM_PRSCTP_CHUNK_TYPES	1

#define SCTP_NUM_AUTH_CHUNK_TYPES	1

#define SCTP_NUM_CHUNK_TYPES		(SCTP_NUM_BASE_CHUNK_TYPES + \
					 SCTP_NUM_ADDIP_CHUNK_TYPES +\
					 SCTP_NUM_PRSCTP_CHUNK_TYPES +\
					 SCTP_NUM_AUTH_CHUNK_TYPES)

/* These are the different flavours of event.  */
typedef enum {

	SCTP_EVENT_T_CHUNK = 1,
	SCTP_EVENT_T_TIMEOUT,
	SCTP_EVENT_T_OTHER,
	SCTP_EVENT_T_PRIMITIVE

} sctp_event_t;

#define SCTP_EVENT_T_MAX SCTP_EVENT_T_PRIMITIVE
#define SCTP_EVENT_T_NUM (SCTP_EVENT_T_MAX + 1)


typedef enum {
	SCTP_EVENT_TIMEOUT_NONE = 0,
	SCTP_EVENT_TIMEOUT_T1_COOKIE,
	SCTP_EVENT_TIMEOUT_T1_INIT,
	SCTP_EVENT_TIMEOUT_T2_SHUTDOWN,
	SCTP_EVENT_TIMEOUT_T3_RTX,
	SCTP_EVENT_TIMEOUT_T4_RTO,
	SCTP_EVENT_TIMEOUT_T5_SHUTDOWN_GUARD,
	SCTP_EVENT_TIMEOUT_HEARTBEAT,
	SCTP_EVENT_TIMEOUT_SACK,
	SCTP_EVENT_TIMEOUT_AUTOCLOSE,
} sctp_event_timeout_t;

#define SCTP_EVENT_TIMEOUT_MAX		SCTP_EVENT_TIMEOUT_AUTOCLOSE
#define SCTP_NUM_TIMEOUT_TYPES		(SCTP_EVENT_TIMEOUT_MAX + 1)

typedef enum {
	SCTP_EVENT_NO_PENDING_TSN = 0,
	SCTP_EVENT_ICMP_PROTO_UNREACH,
} sctp_event_other_t;

#define SCTP_EVENT_OTHER_MAX		SCTP_EVENT_ICMP_PROTO_UNREACH
#define SCTP_NUM_OTHER_TYPES		(SCTP_EVENT_OTHER_MAX + 1)

/* These are primitive requests from the ULP.  */
typedef enum {
	SCTP_PRIMITIVE_ASSOCIATE = 0,
	SCTP_PRIMITIVE_SHUTDOWN,
	SCTP_PRIMITIVE_ABORT,
	SCTP_PRIMITIVE_SEND,
	SCTP_PRIMITIVE_REQUESTHEARTBEAT,
	SCTP_PRIMITIVE_ASCONF,
} sctp_event_primitive_t;

#define SCTP_EVENT_PRIMITIVE_MAX	SCTP_PRIMITIVE_ASCONF
#define SCTP_NUM_PRIMITIVE_TYPES	(SCTP_EVENT_PRIMITIVE_MAX + 1)


typedef union {
	sctp_cid_t chunk;
	sctp_event_timeout_t timeout;
	sctp_event_other_t other;
	sctp_event_primitive_t primitive;
} sctp_subtype_t;

#define SCTP_SUBTYPE_CONSTRUCTOR(_name, _type, _elt) \
static inline sctp_subtype_t	\
SCTP_ST_## _name (_type _arg)		\
{ sctp_subtype_t _retval; _retval._elt = _arg; return _retval; }

SCTP_SUBTYPE_CONSTRUCTOR(CHUNK,		sctp_cid_t,		chunk)
SCTP_SUBTYPE_CONSTRUCTOR(TIMEOUT,	sctp_event_timeout_t,	timeout)
SCTP_SUBTYPE_CONSTRUCTOR(OTHER,		sctp_event_other_t,	other)
SCTP_SUBTYPE_CONSTRUCTOR(PRIMITIVE,	sctp_event_primitive_t,	primitive)


#define sctp_chunk_is_control(a) (a->chunk_hdr->type != SCTP_CID_DATA)
#define sctp_chunk_is_data(a) (a->chunk_hdr->type == SCTP_CID_DATA)

/* Calculate the actual data size in a data chunk */
#define SCTP_DATA_SNDSIZE(c) ((int)((unsigned long)(c->chunk_end)\
		       		- (unsigned long)(c->chunk_hdr)\
				- sizeof(sctp_data_chunk_t)))

#define SCTP_MAX_ERROR_CAUSE  SCTP_ERROR_NONEXIST_IP
#define SCTP_NUM_ERROR_CAUSE  10

/* Internal error codes */
typedef enum {

	SCTP_IERROR_NO_ERROR	        = 0,
	SCTP_IERROR_BASE		= 1000,
	SCTP_IERROR_NO_COOKIE,
	SCTP_IERROR_BAD_SIG,
	SCTP_IERROR_STALE_COOKIE,
	SCTP_IERROR_NOMEM,
	SCTP_IERROR_MALFORMED,
	SCTP_IERROR_BAD_TAG,
	SCTP_IERROR_BIG_GAP,
	SCTP_IERROR_DUP_TSN,
	SCTP_IERROR_HIGH_TSN,
	SCTP_IERROR_IGNORE_TSN,
	SCTP_IERROR_NO_DATA,
	SCTP_IERROR_BAD_STREAM,
	SCTP_IERROR_BAD_PORTS,
	SCTP_IERROR_AUTH_BAD_HMAC,
	SCTP_IERROR_AUTH_BAD_KEYID,
	SCTP_IERROR_PROTO_VIOLATION,
	SCTP_IERROR_ERROR,
	SCTP_IERROR_ABORT,
} sctp_ierror_t;



/* SCTP state defines for internal state machine */
typedef enum {

	SCTP_STATE_EMPTY		= 0,
	SCTP_STATE_CLOSED		= 1,
	SCTP_STATE_COOKIE_WAIT		= 2,
	SCTP_STATE_COOKIE_ECHOED	= 3,
	SCTP_STATE_ESTABLISHED		= 4,
	SCTP_STATE_SHUTDOWN_PENDING	= 5,
	SCTP_STATE_SHUTDOWN_SENT	= 6,
	SCTP_STATE_SHUTDOWN_RECEIVED	= 7,
	SCTP_STATE_SHUTDOWN_ACK_SENT	= 8,

} sctp_state_t;

#define SCTP_STATE_MAX			SCTP_STATE_SHUTDOWN_ACK_SENT
#define SCTP_STATE_NUM_STATES		(SCTP_STATE_MAX + 1)

typedef enum {
	SCTP_SS_CLOSED         = TCP_CLOSE,
	SCTP_SS_LISTENING      = TCP_LISTEN,
	SCTP_SS_ESTABLISHING   = TCP_SYN_SENT,
	SCTP_SS_ESTABLISHED    = TCP_ESTABLISHED,
	SCTP_SS_CLOSING        = TCP_CLOSING,
} sctp_sock_state_t;

/* These functions map various type to printable names.  */
const char *sctp_cname(const sctp_subtype_t);	/* chunk types */
const char *sctp_oname(const sctp_subtype_t);	/* other events */
const char *sctp_tname(const sctp_subtype_t);	/* timeouts */
const char *sctp_pname(const sctp_subtype_t);	/* primitives */

/* This is a table of printable names of sctp_state_t's.  */
extern const char *const sctp_state_tbl[];
extern const char *const sctp_evttype_tbl[];
extern const char *const sctp_status_tbl[];

/* Maximum chunk length considering padding requirements. */
enum { SCTP_MAX_CHUNK_LEN = ((1<<16) - sizeof(__u32)) };

enum { SCTP_ARBITRARY_COOKIE_ECHO_LEN = 200 };

#define SCTP_TSN_MAP_INITIAL BITS_PER_LONG
#define SCTP_TSN_MAP_INCREMENT SCTP_TSN_MAP_INITIAL
#define SCTP_TSN_MAP_SIZE 4096
#define SCTP_TSN_MAX_GAP  65535

enum { SCTP_MIN_PMTU = 576 };
enum { SCTP_MAX_DUP_TSNS = 16 };
enum { SCTP_MAX_GABS = 16 };

/* Heartbeat interval - 30 secs */
#define SCTP_DEFAULT_TIMEOUT_HEARTBEAT	(30*1000)

/* Delayed sack timer - 200ms */
#define SCTP_DEFAULT_TIMEOUT_SACK	(200)

#define SCTP_RTO_INITIAL	(3 * 1000)
#define SCTP_RTO_MIN		(1 * 1000)
#define SCTP_RTO_MAX		(60 * 1000)

#define SCTP_RTO_ALPHA          3   /* 1/8 when converted to right shifts. */
#define SCTP_RTO_BETA           2   /* 1/4 when converted to right shifts. */

/* Maximum number of new data packets that can be sent in a burst.  */
#define SCTP_DEFAULT_MAX_BURST		4

#define SCTP_CLOCK_GRANULARITY	1	/* 1 jiffy */

#define SCTP_DEF_MAX_INIT 6
#define SCTP_DEF_MAX_SEND 10

#define SCTP_DEFAULT_COOKIE_LIFE	(60 * 1000) /* 60 seconds */

#define SCTP_DEFAULT_MINWINDOW	1500	/* default minimum rwnd size */
#define SCTP_DEFAULT_MAXWINDOW	65535	/* default rwnd size */
#define SCTP_DEFAULT_RWND_SHIFT  4	/* by default, update on 1/16 of
					 * rcvbuf, which is 1/8 of initial
					 * window
					 */
#define SCTP_DEFAULT_MAXSEGMENT 1500	/* MTU size, this is the limit
                                         * to which we will raise the P-MTU.
					 */
#define SCTP_DEFAULT_MINSEGMENT 512	/* MTU size ... if no mtu disc */
#define SCTP_HOW_MANY_SECRETS 2		/* How many secrets I keep */
#define SCTP_HOW_LONG_COOKIE_LIVE 3600	/* How many seconds the current
					 * secret will live?
					 */
#define SCTP_SECRET_SIZE 32		/* Number of octets in a 256 bits. */

#define SCTP_SIGNATURE_SIZE 20	        /* size of a SLA-1 signature */

#define SCTP_COOKIE_MULTIPLE 32 /* Pad out our cookie to make our hash
				 * functions simpler to write.
				 */

#if defined (CONFIG_SCTP_HMAC_MD5)
#define SCTP_COOKIE_HMAC_ALG "hmac(md5)"
#elif defined (CONFIG_SCTP_HMAC_SHA1)
#define SCTP_COOKIE_HMAC_ALG "hmac(sha1)"
#else
#define SCTP_COOKIE_HMAC_ALG NULL
#endif

typedef enum {
	SCTP_XMIT_OK,
	SCTP_XMIT_PMTU_FULL,
	SCTP_XMIT_RWND_FULL,
	SCTP_XMIT_NAGLE_DELAY,
} sctp_xmit_t;

/* These are the commands for manipulating transports.  */
typedef enum {
	SCTP_TRANSPORT_UP,
	SCTP_TRANSPORT_DOWN,
} sctp_transport_cmd_t;

typedef enum {
	SCTP_SCOPE_GLOBAL,		/* IPv4 global addresses */
	SCTP_SCOPE_PRIVATE,		/* IPv4 private addresses */
	SCTP_SCOPE_LINK,		/* IPv4 link local address */
	SCTP_SCOPE_LOOPBACK,		/* IPv4 loopback address */
	SCTP_SCOPE_UNUSABLE,		/* IPv4 unusable addresses */
} sctp_scope_t;

typedef enum {
	SCTP_SCOPE_POLICY_DISABLE,	/* Disable IPv4 address scoping */
	SCTP_SCOPE_POLICY_ENABLE,	/* Enable IPv4 address scoping */
	SCTP_SCOPE_POLICY_PRIVATE,	/* Follow draft but allow IPv4 private addresses */
	SCTP_SCOPE_POLICY_LINK,		/* Follow draft but allow IPv4 link local addresses */
} sctp_scope_policy_t;

#define IS_IPV4_UNUSABLE_ADDRESS(a)	    \
	((htonl(INADDR_BROADCAST) == a) ||  \
	 ipv4_is_multicast(a) ||	    \
	 ipv4_is_zeronet(a) ||		    \
	 ipv4_is_test_198(a) ||		    \
	 ipv4_is_anycast_6to4(a))

/* Flags used for the bind address copy functions.  */
#define SCTP_ADDR6_ALLOWED	0x00000001	/* IPv6 address is allowed by
						   local sock family */
#define SCTP_ADDR4_PEERSUPP	0x00000002	/* IPv4 address is supported by
						   peer */
#define SCTP_ADDR6_PEERSUPP	0x00000004	/* IPv6 address is supported by
						   peer */

/* Reasons to retransmit. */
typedef enum {
	SCTP_RTXR_T3_RTX,
	SCTP_RTXR_FAST_RTX,
	SCTP_RTXR_PMTUD,
	SCTP_RTXR_T1_RTX,
} sctp_retransmit_reason_t;

/* Reasons to lower cwnd. */
typedef enum {
	SCTP_LOWER_CWND_T3_RTX,
	SCTP_LOWER_CWND_FAST_RTX,
	SCTP_LOWER_CWND_ECNE,
	SCTP_LOWER_CWND_INACTIVE,
} sctp_lower_cwnd_t;


/* SCTP-AUTH Necessary constants */

enum {
	SCTP_AUTH_HMAC_ID_RESERVED_0,
	SCTP_AUTH_HMAC_ID_SHA1,
	SCTP_AUTH_HMAC_ID_RESERVED_2,
#if defined (CONFIG_CRYPTO_SHA256) || defined (CONFIG_CRYPTO_SHA256_MODULE)
	SCTP_AUTH_HMAC_ID_SHA256,
#endif
	__SCTP_AUTH_HMAC_MAX
};

#define SCTP_AUTH_HMAC_ID_MAX	__SCTP_AUTH_HMAC_MAX - 1
#define SCTP_AUTH_NUM_HMACS 	__SCTP_AUTH_HMAC_MAX
#define SCTP_SHA1_SIG_SIZE 20
#define SCTP_SHA256_SIG_SIZE 32

#define SCTP_NUM_NOAUTH_CHUNKS	4
#define SCTP_AUTH_MAX_CHUNKS	(SCTP_NUM_CHUNK_TYPES - SCTP_NUM_NOAUTH_CHUNKS)

#define SCTP_AUTH_RANDOM_LENGTH 32

#endif /* __sctp_constants_h__ */
