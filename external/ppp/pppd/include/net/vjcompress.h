

#ifndef _VJCOMPRESS_H_
#define _VJCOMPRESS_H_

#define MAX_STATES 16		/* must be > 2 and < 256 */
#define MAX_HDR	   128



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
    struct cstate *cs_next;	/* next most recently used state (xmit only) */
    u_short cs_hlen;		/* size of hdr (receive only) */
    u_char cs_id;		/* connection # associated with this state */
    u_char cs_filler;
    union {
	char csu_hdr[MAX_HDR];
	struct ip csu_ip;	/* ip/tcp hdr from most recent packet */
    } vjcs_u;
};
#define cs_ip vjcs_u.csu_ip
#define cs_hdr vjcs_u.csu_hdr

struct vjcompress {
    struct cstate *last_cs;	/* most recently used tstate */
    u_char last_recv;		/* last rcvd conn. id */
    u_char last_xmit;		/* last sent conn. id */
    u_short flags;
#ifndef VJ_NO_STATS
    struct vjstat stats;
#endif
    struct cstate tstate[MAX_STATES];	/* xmit connection states */
    struct cstate rstate[MAX_STATES];	/* receive connection states */
};

/* flag values */
#define VJF_TOSS 1		/* tossing rcvd frames because of input err */

extern void  vj_compress_init __P((struct vjcompress *comp, int max_state));
extern u_int vj_compress_tcp __P((struct ip *ip, u_int mlen,
				struct vjcompress *comp, int compress_cid_flag,
				u_char **vjhdrp));
extern void  vj_uncompress_err __P((struct vjcompress *comp));
extern int   vj_uncompress_uncomp __P((u_char *buf, int buflen,
				struct vjcompress *comp));
extern int   vj_uncompress_tcp __P((u_char *buf, int buflen, int total_len,
				struct vjcompress *comp, u_char **hdrp,
				u_int *hlenp));

#endif /* _VJCOMPRESS_H_ */
