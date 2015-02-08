

#ifndef _IP_H_
#define _IP_H_

#ifdef HOST_WORDS_BIGENDIAN
# ifndef NTOHL
#  define NTOHL(d)
# endif
# ifndef NTOHS
#  define NTOHS(d)
# endif
# ifndef HTONL
#  define HTONL(d)
# endif
# ifndef HTONS
#  define HTONS(d)
# endif
#else
# ifndef NTOHL
#  define NTOHL(d) ((d) = ntohl((d)))
# endif
# ifndef NTOHS
#  define NTOHS(d) ((d) = ntohs((u_int16_t)(d)))
# endif
# ifndef HTONL
#  define HTONL(d) ((d) = htonl((d)))
# endif
# ifndef HTONS
#  define HTONS(d) ((d) = htons((u_int16_t)(d)))
# endif
#endif

typedef u_int32_t n_long;                 /* long as received from the net */

#define	IPVERSION	4

struct ip {
#ifdef HOST_WORDS_BIGENDIAN
	u_int ip_v:4,			/* version */
		ip_hl:4;		/* header length */
#else
	u_int ip_hl:4,		/* header length */
		ip_v:4;			/* version */
#endif
	u_int8_t ip_tos;			/* type of service */
	u_int16_t	ip_len;			/* total length */
	u_int16_t	ip_id;			/* identification */
	u_int16_t	ip_off;			/* fragment offset field */
#define	IP_DF 0x4000			/* don't fragment flag */
#define	IP_MF 0x2000			/* more fragments flag */
#define	IP_OFFMASK 0x1fff		/* mask for fragmenting bits */
	u_int8_t ip_ttl;			/* time to live */
	u_int8_t ip_p;			/* protocol */
	u_int16_t	ip_sum;			/* checksum */
	struct	in_addr ip_src,ip_dst;	/* source and dest address */
};

#define	IP_MAXPACKET	65535		/* maximum packet size */

#define	IPTOS_LOWDELAY		0x10
#define	IPTOS_THROUGHPUT	0x08
#define	IPTOS_RELIABILITY	0x04

#define	IPOPT_COPIED(o)		((o)&0x80)
#define	IPOPT_CLASS(o)		((o)&0x60)
#define	IPOPT_NUMBER(o)		((o)&0x1f)

#define	IPOPT_CONTROL		0x00
#define	IPOPT_RESERVED1		0x20
#define	IPOPT_DEBMEAS		0x40
#define	IPOPT_RESERVED2		0x60

#define	IPOPT_EOL		0		/* end of option list */
#define	IPOPT_NOP		1		/* no operation */

#define	IPOPT_RR		7		/* record packet route */
#define	IPOPT_TS		68		/* timestamp */
#define	IPOPT_SECURITY		130		/* provide s,c,h,tcc */
#define	IPOPT_LSRR		131		/* loose source route */
#define	IPOPT_SATID		136		/* satnet id */
#define	IPOPT_SSRR		137		/* strict source route */

#define	IPOPT_OPTVAL		0		/* option ID */
#define	IPOPT_OLEN		1		/* option length */
#define IPOPT_OFFSET		2		/* offset within option */
#define	IPOPT_MINOFF		4		/* min value of above */

struct	ip_timestamp {
	u_int8_t	ipt_code;		/* IPOPT_TS */
	u_int8_t	ipt_len;		/* size of structure (variable) */
	u_int8_t	ipt_ptr;		/* index of current entry */
#ifdef HOST_WORDS_BIGENDIAN
	u_int	ipt_oflw:4,		/* overflow counter */
		ipt_flg:4;		/* flags, see below */
#else
	u_int	ipt_flg:4,		/* flags, see below */
		ipt_oflw:4;		/* overflow counter */
#endif
	union ipt_timestamp {
		n_long	ipt_time[1];
		struct	ipt_ta {
			struct in_addr ipt_addr;
			n_long ipt_time;
		} ipt_ta[1];
	} ipt_timestamp;
};

/* flag bits for ipt_flg */
#define	IPOPT_TS_TSONLY		0		/* timestamps only */
#define	IPOPT_TS_TSANDADDR	1		/* timestamps and addresses */
#define	IPOPT_TS_PRESPEC	3		/* specified modules only */

/* bits for security (not byte swapped) */
#define	IPOPT_SECUR_UNCLASS	0x0000
#define	IPOPT_SECUR_CONFID	0xf135
#define	IPOPT_SECUR_EFTO	0x789a
#define	IPOPT_SECUR_MMMM	0xbc4d
#define	IPOPT_SECUR_RESTR	0xaf13
#define	IPOPT_SECUR_SECRET	0xd788
#define	IPOPT_SECUR_TOPSECRET	0x6bc5

#define	MAXTTL		255		/* maximum time to live (seconds) */
#define	IPDEFTTL	64		/* default ttl, from RFC 1340 */
#define	IPFRAGTTL	60		/* time to live for frags, slowhz */
#define	IPTTLDEC	1		/* subtracted when forwarding */

#define	IP_MSS		576		/* default maximum segment size */

#if SIZEOF_CHAR_P == 4
struct mbuf_ptr {
	struct mbuf *mptr;
	uint32_t dummy;
};
#else
struct mbuf_ptr {
	struct mbuf *mptr;
};
#endif
struct qlink {
	void *next, *prev;
};

struct ipovly {
	struct mbuf_ptr ih_mbuf;	/* backpointer to mbuf */
	u_int8_t	ih_x1;			/* (unused) */
	u_int8_t	ih_pr;			/* protocol */
	u_int16_t	ih_len;			/* protocol length */
	struct	in_addr ih_src;		/* source internet address */
	struct	in_addr ih_dst;		/* destination internet address */
} __attribute__((packed));

struct ipq {
        struct qlink frag_link;			/* to ip headers of fragments */
	struct qlink ip_link;				/* to other reass headers */
	u_int8_t	ipq_ttl;		/* time for reass q to live */
	u_int8_t	ipq_p;			/* protocol of this fragment */
	u_int16_t	ipq_id;			/* sequence id for reassembly */
	struct	in_addr ipq_src,ipq_dst;
};

struct	ipasfrag {
	struct qlink ipf_link;
	struct ip ipf_ip;
};

#define ipf_off      ipf_ip.ip_off
#define ipf_tos      ipf_ip.ip_tos
#define ipf_len      ipf_ip.ip_len
#define ipf_next     ipf_link.next
#define ipf_prev     ipf_link.prev 

#define MAX_IPOPTLEN	40

struct ipoption {
	struct	in_addr ipopt_dst;	/* first-hop dst if source routed */
	int8_t	ipopt_list[MAX_IPOPTLEN];	/* options proper */
};

#ifdef LOG_ENABLED

struct	ipstat {
	u_long	ips_total;		/* total packets received */
	u_long	ips_badsum;		/* checksum bad */
	u_long	ips_tooshort;		/* packet too short */
	u_long	ips_toosmall;		/* not enough data */
	u_long	ips_badhlen;		/* ip header length < data size */
	u_long	ips_badlen;		/* ip length < ip header length */
	u_long	ips_fragments;		/* fragments received */
	u_long	ips_fragdropped;	/* frags dropped (dups, out of space) */
	u_long	ips_fragtimeout;	/* fragments timed out */
	u_long	ips_forward;		/* packets forwarded */
	u_long	ips_cantforward;	/* packets rcvd for unreachable dest */
	u_long	ips_redirectsent;	/* packets forwarded on same net */
	u_long	ips_noproto;		/* unknown or unsupported protocol */
	u_long	ips_delivered;		/* datagrams delivered to upper level*/
	u_long	ips_localout;		/* total ip packets generated here */
	u_long	ips_odropped;		/* lost packets due to nobufs, etc. */
	u_long	ips_reassembled;	/* total packets reassembled ok */
	u_long	ips_fragmented;		/* datagrams successfully fragmented */
	u_long	ips_ofragments;		/* output fragments created */
	u_long	ips_cantfrag;		/* don't fragment flag was set, etc. */
	u_long	ips_badoptions;		/* error in option processing */
	u_long	ips_noroute;		/* packets discarded due to no route */
	u_long	ips_badvers;		/* ip version != 4 */
	u_long	ips_rawout;		/* total raw ip packets generated */
	u_long	ips_unaligned;		/* times the ip packet was not aligned */
};

extern struct	ipstat	ipstat;
#endif

extern struct	ipq	ipq;			/* ip reass. queue */
extern u_int16_t	ip_id;				/* ip packet ctr, for ids */

#endif
