
#ifndef _SLHC_H
#define _SLHC_H




#include <linux/ip.h>
#include <linux/tcp.h>

/* SLIP compression masks for len/vers byte */
#define SL_TYPE_IP 0x40
#define SL_TYPE_UNCOMPRESSED_TCP 0x70
#define SL_TYPE_COMPRESSED_TCP 0x80
#define SL_TYPE_ERROR 0x00

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


typedef __u8 byte_t;
typedef __u32 int32;

struct cstate {
	byte_t	cs_this;	/* connection id number (xmit) */
	struct cstate *next;	/* next in ring (xmit) */
	struct iphdr cs_ip;	/* ip/tcp hdr from most recent packet */
	struct tcphdr cs_tcp;
	unsigned char cs_ipopt[64];
	unsigned char cs_tcpopt[64];
	int cs_hsize;
};
#define NULLSLSTATE	(struct cstate *)0

struct slcompress {
	struct cstate *tstate;	/* transmit connection states (array)*/
	struct cstate *rstate;	/* receive connection states (array)*/

	byte_t tslot_limit;	/* highest transmit slot id (0-l)*/
	byte_t rslot_limit;	/* highest receive slot id (0-l)*/

	byte_t xmit_oldest;	/* oldest xmit in ring */
	byte_t xmit_current;	/* most recent xmit id */
	byte_t recv_current;	/* most recent rcvd id */

	byte_t flags;
#define SLF_TOSS	0x01	/* tossing rcvd frames until id received */

	int32 sls_o_nontcp;	/* outbound non-TCP packets */
	int32 sls_o_tcp;	/* outbound TCP packets */
	int32 sls_o_uncompressed;	/* outbound uncompressed packets */
	int32 sls_o_compressed;	/* outbound compressed packets */
	int32 sls_o_searches;	/* searches for connection state */
	int32 sls_o_misses;	/* times couldn't find conn. state */

	int32 sls_i_uncompressed;	/* inbound uncompressed packets */
	int32 sls_i_compressed;	/* inbound compressed packets */
	int32 sls_i_error;	/* inbound error packets */
	int32 sls_i_tossed;	/* inbound packets tossed because of error */

	int32 sls_i_runt;
	int32 sls_i_badcheck;
};
#define NULLSLCOMPR	(struct slcompress *)0

/* In slhc.c: */
struct slcompress *slhc_init(int rslots, int tslots);
void slhc_free(struct slcompress *comp);

int slhc_compress(struct slcompress *comp, unsigned char *icp, int isize,
		  unsigned char *ocp, unsigned char **cpp, int compress_cid);
int slhc_uncompress(struct slcompress *comp, unsigned char *icp, int isize);
int slhc_remember(struct slcompress *comp, unsigned char *icp, int isize);
int slhc_toss(struct slcompress *comp);

#endif	/* _SLHC_H */
