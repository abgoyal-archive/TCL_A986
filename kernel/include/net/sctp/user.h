

#ifndef __net_sctp_user_h__
#define __net_sctp_user_h__

#include <linux/types.h>
#include <linux/socket.h>

typedef __s32 sctp_assoc_t;

#define SCTP_RTOINFO	0
#define SCTP_ASSOCINFO  1
#define SCTP_INITMSG	2
#define SCTP_NODELAY	3		/* Get/set nodelay option. */
#define SCTP_AUTOCLOSE	4
#define SCTP_SET_PEER_PRIMARY_ADDR 5
#define SCTP_PRIMARY_ADDR	6
#define SCTP_ADAPTATION_LAYER	7
#define SCTP_DISABLE_FRAGMENTS	8
#define SCTP_PEER_ADDR_PARAMS	9
#define SCTP_DEFAULT_SEND_PARAM	10
#define SCTP_EVENTS	11
#define SCTP_I_WANT_MAPPED_V4_ADDR 12	/* Turn on/off mapped v4 addresses  */
#define SCTP_MAXSEG	13		/* Get/set maximum fragment. */
#define SCTP_STATUS	14
#define SCTP_GET_PEER_ADDR_INFO	15
#define SCTP_DELAYED_ACK_TIME	16
#define SCTP_DELAYED_ACK SCTP_DELAYED_ACK_TIME
#define SCTP_CONTEXT	17
#define SCTP_FRAGMENT_INTERLEAVE	18
#define SCTP_PARTIAL_DELIVERY_POINT	19 /* Set/Get partial delivery point */
#define SCTP_MAX_BURST	20		/* Set/Get max burst */
#define SCTP_AUTH_CHUNK	21	/* Set only: add a chunk type to authenticate */
#define SCTP_HMAC_IDENT	22
#define SCTP_AUTH_KEY	23
#define SCTP_AUTH_ACTIVE_KEY	24
#define SCTP_AUTH_DELETE_KEY	25
#define SCTP_PEER_AUTH_CHUNKS	26	/* Read only */
#define SCTP_LOCAL_AUTH_CHUNKS	27	/* Read only */
#define SCTP_GET_ASSOC_NUMBER	28	/* Read only */

#define SCTP_SOCKOPT_BINDX_ADD	100	/* BINDX requests for adding addrs */
#define SCTP_SOCKOPT_BINDX_REM	101	/* BINDX requests for removing addrs. */
#define SCTP_SOCKOPT_PEELOFF	102	/* peel off association. */
/* Options 104-106 are deprecated and removed. Do not use this space */
#define SCTP_SOCKOPT_CONNECTX_OLD	107	/* CONNECTX old requests. */
#define SCTP_GET_PEER_ADDRS	108		/* Get all peer addresss. */
#define SCTP_GET_LOCAL_ADDRS	109		/* Get all local addresss. */
#define SCTP_SOCKOPT_CONNECTX	110		/* CONNECTX requests. */
#define SCTP_SOCKOPT_CONNECTX3	111	/* CONNECTX requests (updated) */

struct sctp_initmsg {
	__u16 sinit_num_ostreams;
	__u16 sinit_max_instreams;
	__u16 sinit_max_attempts;
	__u16 sinit_max_init_timeo;
};

struct sctp_sndrcvinfo {
	__u16 sinfo_stream;
	__u16 sinfo_ssn;
	__u16 sinfo_flags;
	__u32 sinfo_ppid;
	__u32 sinfo_context;
	__u32 sinfo_timetolive;
	__u32 sinfo_tsn;
	__u32 sinfo_cumtsn;
	sctp_assoc_t sinfo_assoc_id;
};


enum sctp_sinfo_flags {
	SCTP_UNORDERED = 1,  /* Send/receive message unordered. */
	SCTP_ADDR_OVER = 2,  /* Override the primary destination. */
	SCTP_ABORT=4,        /* Send an ABORT message to the peer. */
	SCTP_SACK_IMMEDIATELY = 8,	/* SACK should be sent without delay */
	SCTP_EOF=MSG_FIN,    /* Initiate graceful shutdown process. */	
};


/* These are cmsg_types.  */
typedef enum sctp_cmsg_type {
	SCTP_INIT,              /* 5.2.1 SCTP Initiation Structure */
	SCTP_SNDRCV,            /* 5.2.2 SCTP Header Information Structure */
} sctp_cmsg_t;


struct sctp_assoc_change {
	__u16 sac_type;
	__u16 sac_flags;
	__u32 sac_length;
	__u16 sac_state;
	__u16 sac_error;
	__u16 sac_outbound_streams;
	__u16 sac_inbound_streams;
	sctp_assoc_t sac_assoc_id;
	__u8 sac_info[0];
};

enum sctp_sac_state {
	SCTP_COMM_UP,
	SCTP_COMM_LOST,
	SCTP_RESTART,
	SCTP_SHUTDOWN_COMP,
	SCTP_CANT_STR_ASSOC,
};

struct sctp_paddr_change {
	__u16 spc_type;
	__u16 spc_flags;
	__u32 spc_length;
	struct sockaddr_storage spc_aaddr;
	int spc_state;
	int spc_error;
	sctp_assoc_t spc_assoc_id;
} __attribute__((packed, aligned(4)));

enum sctp_spc_state {
	SCTP_ADDR_AVAILABLE,
	SCTP_ADDR_UNREACHABLE,
	SCTP_ADDR_REMOVED,
	SCTP_ADDR_ADDED,
	SCTP_ADDR_MADE_PRIM,
	SCTP_ADDR_CONFIRMED,
};


struct sctp_remote_error {
	__u16 sre_type;
	__u16 sre_flags;
	__u32 sre_length;
	__u16 sre_error;
	sctp_assoc_t sre_assoc_id;
	__u8 sre_data[0];
};


struct sctp_send_failed {
	__u16 ssf_type;
	__u16 ssf_flags;
	__u32 ssf_length;
	__u32 ssf_error;
	struct sctp_sndrcvinfo ssf_info;
	sctp_assoc_t ssf_assoc_id;
	__u8 ssf_data[0];
};

enum sctp_ssf_flags {
	SCTP_DATA_UNSENT,
	SCTP_DATA_SENT,
};

struct sctp_shutdown_event {
	__u16 sse_type;
	__u16 sse_flags;
	__u32 sse_length;
	sctp_assoc_t sse_assoc_id;
};

struct sctp_adaptation_event {
	__u16 sai_type;
	__u16 sai_flags;
	__u32 sai_length;
	__u32 sai_adaptation_ind;
	sctp_assoc_t sai_assoc_id;
};

struct sctp_pdapi_event {
	__u16 pdapi_type;
	__u16 pdapi_flags;
	__u32 pdapi_length;
	__u32 pdapi_indication;
	sctp_assoc_t pdapi_assoc_id;
};

enum { SCTP_PARTIAL_DELIVERY_ABORTED=0, };

struct sctp_authkey_event {
	__u16 auth_type;
	__u16 auth_flags;
	__u32 auth_length;
	__u16 auth_keynumber;
	__u16 auth_altkeynumber;
	__u32 auth_indication;
	sctp_assoc_t auth_assoc_id;
};

enum { SCTP_AUTH_NEWKEY = 0, };


struct sctp_event_subscribe {
	__u8 sctp_data_io_event;
	__u8 sctp_association_event;
	__u8 sctp_address_event;
	__u8 sctp_send_failure_event;
	__u8 sctp_peer_error_event;
	__u8 sctp_shutdown_event;
	__u8 sctp_partial_delivery_event;
	__u8 sctp_adaptation_layer_event;
	__u8 sctp_authentication_event;
};

union sctp_notification {
	struct {
		__u16 sn_type;             /* Notification type. */
		__u16 sn_flags;
		__u32 sn_length;
	} sn_header;
	struct sctp_assoc_change sn_assoc_change;
	struct sctp_paddr_change sn_paddr_change;
	struct sctp_remote_error sn_remote_error;
	struct sctp_send_failed sn_send_failed;
	struct sctp_shutdown_event sn_shutdown_event;
	struct sctp_adaptation_event sn_adaptation_event;
	struct sctp_pdapi_event sn_pdapi_event;
	struct sctp_authkey_event sn_authkey_event;
};


enum sctp_sn_type {
	SCTP_SN_TYPE_BASE     = (1<<15),
	SCTP_ASSOC_CHANGE,
	SCTP_PEER_ADDR_CHANGE,
	SCTP_SEND_FAILED,
	SCTP_REMOTE_ERROR,
	SCTP_SHUTDOWN_EVENT,
	SCTP_PARTIAL_DELIVERY_EVENT,
	SCTP_ADAPTATION_INDICATION,
	SCTP_AUTHENTICATION_INDICATION,
};

typedef enum sctp_sn_error {
	SCTP_FAILED_THRESHOLD,
	SCTP_RECEIVED_SACK,
	SCTP_HEARTBEAT_SUCCESS,
	SCTP_RESPONSE_TO_USER_REQ,
	SCTP_INTERNAL_ERROR,
	SCTP_SHUTDOWN_GUARD_EXPIRES,
	SCTP_PEER_FAULTY,
} sctp_sn_error_t;

struct sctp_rtoinfo {
	sctp_assoc_t	srto_assoc_id;
	__u32		srto_initial;
	__u32		srto_max;
	__u32		srto_min;
};

struct sctp_assocparams {
	sctp_assoc_t	sasoc_assoc_id;
	__u16		sasoc_asocmaxrxt;
	__u16		sasoc_number_peer_destinations;
	__u32		sasoc_peer_rwnd;
	__u32		sasoc_local_rwnd;
	__u32		sasoc_cookie_life;
};

struct sctp_setpeerprim {
	sctp_assoc_t            sspp_assoc_id;
	struct sockaddr_storage sspp_addr;
} __attribute__((packed, aligned(4)));

struct sctp_prim {
	sctp_assoc_t            ssp_assoc_id;
	struct sockaddr_storage ssp_addr;
} __attribute__((packed, aligned(4)));

struct sctp_setadaptation {
	__u32	ssb_adaptation_ind;
};

enum  sctp_spp_flags {
	SPP_HB_ENABLE = 1<<0,		/*Enable heartbeats*/
	SPP_HB_DISABLE = 1<<1,		/*Disable heartbeats*/
	SPP_HB = SPP_HB_ENABLE | SPP_HB_DISABLE,
	SPP_HB_DEMAND = 1<<2,		/*Send heartbeat immediately*/
	SPP_PMTUD_ENABLE = 1<<3,	/*Enable PMTU discovery*/
	SPP_PMTUD_DISABLE = 1<<4,	/*Disable PMTU discovery*/
	SPP_PMTUD = SPP_PMTUD_ENABLE | SPP_PMTUD_DISABLE,
	SPP_SACKDELAY_ENABLE = 1<<5,	/*Enable SACK*/
	SPP_SACKDELAY_DISABLE = 1<<6,	/*Disable SACK*/
	SPP_SACKDELAY = SPP_SACKDELAY_ENABLE | SPP_SACKDELAY_DISABLE,
	SPP_HB_TIME_IS_ZERO = 1<<7,	/* Set HB delay to 0 */
};

struct sctp_paddrparams {
	sctp_assoc_t		spp_assoc_id;
	struct sockaddr_storage	spp_address;
	__u32			spp_hbinterval;
	__u16			spp_pathmaxrxt;
	__u32			spp_pathmtu;
	__u32			spp_sackdelay;
	__u32			spp_flags;
} __attribute__((packed, aligned(4)));

struct sctp_authchunk {
	__u8		sauth_chunk;
};

struct sctp_hmacalgo {
	__u32		shmac_num_idents;
	__u16		shmac_idents[];
};

struct sctp_authkey {
	sctp_assoc_t	sca_assoc_id;
	__u16		sca_keynumber;
	__u16		sca_keylength;
	__u8		sca_key[];
};


struct sctp_authkeyid {
	sctp_assoc_t	scact_assoc_id;
	__u16		scact_keynumber;
};


struct sctp_sack_info {
	sctp_assoc_t	sack_assoc_id;
	uint32_t	sack_delay;
	uint32_t	sack_freq;
};

struct sctp_assoc_value {
    sctp_assoc_t            assoc_id;
    uint32_t                assoc_value;
};

struct sctp_paddrinfo {
	sctp_assoc_t		spinfo_assoc_id;
	struct sockaddr_storage	spinfo_address;
	__s32			spinfo_state;
	__u32			spinfo_cwnd;
	__u32			spinfo_srtt;
	__u32			spinfo_rto;
	__u32			spinfo_mtu;
} __attribute__((packed, aligned(4)));

/* Peer addresses's state. */
enum sctp_spinfo_state {
	SCTP_INACTIVE,
	SCTP_ACTIVE,
	SCTP_UNCONFIRMED,
	SCTP_UNKNOWN = 0xffff  /* Value used for transport state unknown */
};

struct sctp_status {
	sctp_assoc_t		sstat_assoc_id;
	__s32			sstat_state;
	__u32			sstat_rwnd;
	__u16			sstat_unackdata;
	__u16			sstat_penddata;
	__u16			sstat_instrms;
	__u16			sstat_outstrms;
	__u32			sstat_fragmentation_point;
	struct sctp_paddrinfo	sstat_primary;
};

struct sctp_authchunks {
	sctp_assoc_t	gauth_assoc_id;
	__u32		gauth_number_of_chunks;
	uint8_t		gauth_chunks[];
};

struct sctp_getaddrs_old {
	sctp_assoc_t            assoc_id;
	int			addr_num;
	struct sockaddr		__user *addrs;
};
struct sctp_getaddrs {
	sctp_assoc_t		assoc_id; /*input*/
	__u32			addr_num; /*output*/
	__u8			addrs[0]; /*output, variable size*/
};

/* These are bit fields for msghdr->msg_flags.  See section 5.1.  */
/* On user space Linux, these live in <bits/socket.h> as an enum.  */
enum sctp_msg_flags {
	MSG_NOTIFICATION = 0x8000,
#define MSG_NOTIFICATION MSG_NOTIFICATION
};

#define SCTP_BINDX_ADD_ADDR 0x01
#define SCTP_BINDX_REM_ADDR 0x02

typedef struct {
	sctp_assoc_t associd;
	int sd;
} sctp_peeloff_arg_t;

#endif /* __net_sctp_user_h__ */
