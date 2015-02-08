
/* @(#) $Header: /tcpdump/master/tcpdump/tcp.h,v 1.11.2.1 2005/11/29 09:09:26 hannes Exp $ (LBL) */

typedef	u_int32_t	tcp_seq;
struct tcphdr {
	u_int16_t	th_sport;		/* source port */
	u_int16_t	th_dport;		/* destination port */
	tcp_seq		th_seq;			/* sequence number */
	tcp_seq		th_ack;			/* acknowledgement number */
	u_int8_t	th_offx2;		/* data offset, rsvd */
#define TH_OFF(th)	(((th)->th_offx2 & 0xf0) >> 4)
	u_int8_t	th_flags;
#define	TH_FIN	0x01
#define	TH_SYN	0x02
#define	TH_RST	0x04
#define	TH_PUSH	0x08
#define	TH_ACK	0x10
#define	TH_URG	0x20
#define TH_ECNECHO	0x40	/* ECN Echo */
#define TH_CWR		0x80	/* ECN Cwnd Reduced */
	u_int16_t	th_win;			/* window */
	u_int16_t	th_sum;			/* checksum */
	u_int16_t	th_urp;			/* urgent pointer */
};

#define	TCPOPT_EOL		0
#define	TCPOPT_NOP		1
#define	TCPOPT_MAXSEG		2
#define    TCPOLEN_MAXSEG		4
#define	TCPOPT_WSCALE		3	/* window scale factor (rfc1323) */
#define	TCPOPT_SACKOK		4	/* selective ack ok (rfc2018) */
#define	TCPOPT_SACK		5	/* selective ack (rfc2018) */
#define	TCPOPT_ECHO		6	/* echo (rfc1072) */
#define	TCPOPT_ECHOREPLY	7	/* echo (rfc1072) */
#define TCPOPT_TIMESTAMP	8	/* timestamp (rfc1323) */
#define    TCPOLEN_TIMESTAMP		10
#define    TCPOLEN_TSTAMP_APPA		(TCPOLEN_TIMESTAMP+2) /* appendix A */
#define TCPOPT_CC		11	/* T/TCP CC options (rfc1644) */
#define TCPOPT_CCNEW		12	/* T/TCP CC options (rfc1644) */
#define TCPOPT_CCECHO		13	/* T/TCP CC options (rfc1644) */
#define TCPOPT_SIGNATURE	19	/* Keyed MD5 (rfc2385) */
#define    TCPOLEN_SIGNATURE		18
#define TCP_SIGLEN 16			/* length of an option 19 digest */
#define TCPOPT_AUTH             20      /* Enhanced AUTH option */

#define TCPOPT_TSTAMP_HDR	\
    (TCPOPT_NOP<<24|TCPOPT_NOP<<16|TCPOPT_TIMESTAMP<<8|TCPOLEN_TIMESTAMP)
