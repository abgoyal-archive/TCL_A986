

#ifndef _SLCOMPRESS_H_
#define _SLCOMPRESS_H_

#define MAX_STATES 16		/* must be > 2 and < 256 */
#define MAX_HDR MLEN		/* XXX 4bsd-ism: should really be 128 */



/* packet types */
#define TYPE_IP 0x40
#define TYPE_UNCOMPRESSED_TCP 0x70
#define TYPE_COMPRESSED_TCP 0x80
#define TYPE_ERROR 0x00

/* Bits in first octet of compressed packet */
#define NEW_C	0x40	/* flag bits for what changed in a packet */
#define NEW_I	0x20
#define NEW_S	0x08
#define NEW_A	0x04
#define NEW_W	0x02
#define NEW_U	0x01

/* reserved, special-case values of above */
#define SPECIAL_I (NEW_S|NEW_W|NEW_U)		/* echoed interactive traffic */
#define SPECIAL_D (NEW_S|NEW_A|NEW_W|NEW_U)	/* unidirectional data */
#define SPECIALS_MASK (NEW_S|NEW_A|NEW_W|NEW_U)

#define TCP_PUSH_BIT 0x10


struct cstate {
	struct cstate *cs_next;	/* next most recently used cstate (xmit only) */
	u_short cs_hlen;	/* size of hdr (receive only) */
	u_char cs_id;		/* connection # associated with this state */
	u_char cs_filler;
	union {
		char csu_hdr[MAX_HDR];
		struct ip csu_ip;	/* ip/tcp hdr from most recent packet */
	} slcs_u;
};
#define cs_ip slcs_u.csu_ip
#define cs_hdr slcs_u.csu_hdr

struct slcompress {
	struct cstate *last_cs;	/* most recently used tstate */
	u_char last_recv;	/* last rcvd conn. id */
	u_char last_xmit;	/* last sent conn. id */
	u_short flags;
#ifndef SL_NO_STATS
	int sls_packets;	/* outbound packets */
	int sls_compressed;	/* outbound compressed packets */
	int sls_searches;	/* searches for connection state */
	int sls_misses;		/* times couldn't find conn. state */
	int sls_uncompressedin;	/* inbound uncompressed packets */
	int sls_compressedin;	/* inbound compressed packets */
	int sls_errorin;	/* inbound unknown type packets */
	int sls_tossed;		/* inbound packets tossed because of error */
#endif
	struct cstate tstate[MAX_STATES];	/* xmit connection states */
	struct cstate rstate[MAX_STATES];	/* receive connection states */
};
/* flag values */
#define SLF_TOSS 1		/* tossing rcvd frames because of input err */

void	sl_compress_init __P((struct slcompress *));
void	sl_compress_setup __P((struct slcompress *, int));
u_int	sl_compress_tcp __P((struct mbuf *,
	    struct ip *, struct slcompress *, int));
int	sl_uncompress_tcp __P((u_char **, int, u_int, struct slcompress *));
int	sl_uncompress_tcp_core __P((u_char *, int, int, u_int,
	    struct slcompress *, u_char **, u_int *));

#endif /* _SLCOMPRESS_H_ */
