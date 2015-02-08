


#ifndef HAVE___ATTRIBUTE__
#define __attribute__(x)
#endif /* HAVE___ATTRIBUTE__ */

/* Address qualifiers. */

#define Q_HOST		1
#define Q_NET		2
#define Q_PORT		3
#define Q_GATEWAY	4
#define Q_PROTO		5
#define Q_PROTOCHAIN	6
#define Q_PORTRANGE	7

/* Protocol qualifiers. */

#define Q_LINK		1
#define Q_IP		2
#define Q_ARP		3
#define Q_RARP		4
#define Q_SCTP		5
#define Q_TCP		6
#define Q_UDP		7
#define Q_ICMP		8
#define Q_IGMP		9
#define Q_IGRP		10


#define	Q_ATALK		11
#define	Q_DECNET	12
#define	Q_LAT		13
#define Q_SCA		14
#define	Q_MOPRC		15
#define	Q_MOPDL		16


#define Q_IPV6		17
#define Q_ICMPV6	18
#define Q_AH		19
#define Q_ESP		20

#define Q_PIM		21
#define Q_VRRP		22

#define Q_AARP		23

#define Q_ISO		24
#define Q_ESIS		25
#define Q_ISIS		26
#define Q_CLNP		27

#define Q_STP		28

#define Q_IPX		29

#define Q_NETBEUI	30

/* IS-IS Levels */
#define Q_ISIS_L1       31
#define Q_ISIS_L2       32
/* PDU types */
#define Q_ISIS_IIH      33
#define Q_ISIS_LAN_IIH  34
#define Q_ISIS_PTP_IIH  35
#define Q_ISIS_SNP      36
#define Q_ISIS_CSNP     37
#define Q_ISIS_PSNP     38
#define Q_ISIS_LSP      39

#define Q_RADIO		40

/* Directional qualifiers. */

#define Q_SRC		1
#define Q_DST		2
#define Q_OR		3
#define Q_AND		4

#define Q_DEFAULT	0
#define Q_UNDEF		255

/* ATM types */
#define A_METAC		22	/* Meta signalling Circuit */
#define A_BCC		23	/* Broadcast Circuit */
#define A_OAMF4SC	24	/* Segment OAM F4 Circuit */
#define A_OAMF4EC	25	/* End-to-End OAM F4 Circuit */
#define A_SC		26	/* Signalling Circuit*/
#define A_ILMIC		27	/* ILMI Circuit */
#define A_OAM		28	/* OAM cells : F4 only */
#define A_OAMF4		29	/* OAM F4 cells: Segment + End-to-end */
#define A_LANE		30	/* LANE traffic */
#define A_LLC		31	/* LLC-encapsulated traffic */

/* Based on Q.2931 signalling protocol */
#define A_SETUP		41	/* Setup message */
#define A_CALLPROCEED	42	/* Call proceeding message */
#define A_CONNECT	43	/* Connect message */
#define A_CONNECTACK	44	/* Connect Ack message */
#define A_RELEASE	45	/* Release message */
#define A_RELEASE_DONE	46	/* Release message */
 
/* ATM field types */
#define A_VPI		51
#define A_VCI		52
#define A_PROTOTYPE	53
#define A_MSGTYPE	54
#define A_CALLREFTYPE	55

#define A_CONNECTMSG	70	/* returns Q.2931 signalling messages for
				   establishing and destroying switched
				   virtual connection */
#define A_METACONNECT	71	/* returns Q.2931 signalling messages for
				   establishing and destroying predefined
				   virtual circuits, such as broadcast
				   circuit, oamf4 segment circuit, oamf4
				   end-to-end circuits, ILMI circuits or
				   connection signalling circuit. */

/* MTP2 types */
#define M_FISU		22	/* FISU */
#define M_LSSU		23	/* LSSU */
#define M_MSU		24	/* MSU */

/* MTP3 field types */
#define M_SIO		1
#define M_OPC		2
#define M_DPC		3
#define M_SLS		4


struct slist;

struct stmt {
	int code;
	struct slist *jt;	/*only for relative jump in block*/
	struct slist *jf;	/*only for relative jump in block*/
	bpf_int32 k;
};

struct slist {
	struct stmt s;
	struct slist *next;
};

typedef bpf_u_int32 atomset;
#define ATOMMASK(n) (1 << (n))
#define ATOMELEM(d, n) (d & ATOMMASK(n))

typedef bpf_u_int32 *uset;

#define N_ATOMS (BPF_MEMWORDS+2)

struct edge {
	int id;
	int code;
	uset edom;
	struct block *succ;
	struct block *pred;
	struct edge *next;	/* link list of incoming edges for a node */
};

struct block {
	int id;
	struct slist *stmts;	/* side effect stmts */
	struct stmt s;		/* branch stmt */
	int mark;
	int longjt;		/* jt branch requires long jump */
	int longjf;		/* jf branch requires long jump */
	int level;
	int offset;
	int sense;
	struct edge et;
	struct edge ef;
	struct block *head;
	struct block *link;	/* link field used by optimizer */
	uset dom;
	uset closure;
	struct edge *in_edges;
	atomset def, kill;
	atomset in_use;
	atomset out_use;
	int oval;
	int val[N_ATOMS];
};

struct arth {
	struct block *b;	/* protocol checks */
	struct slist *s;	/* stmt list */
	int regno;		/* virtual register number of result */
};

struct qual {
	unsigned char addr;
	unsigned char proto;
	unsigned char dir;
	unsigned char pad;
};

struct arth *gen_loadi(int);
struct arth *gen_load(int, struct arth *, int);
struct arth *gen_loadlen(void);
struct arth *gen_neg(struct arth *);
struct arth *gen_arth(int, struct arth *, struct arth *);

void gen_and(struct block *, struct block *);
void gen_or(struct block *, struct block *);
void gen_not(struct block *);

struct block *gen_scode(const char *, struct qual);
struct block *gen_ecode(const u_char *, struct qual);
struct block *gen_acode(const u_char *, struct qual);
struct block *gen_mcode(const char *, const char *, int, struct qual);
#ifdef INET6
struct block *gen_mcode6(const char *, const char *, int, struct qual);
#endif
struct block *gen_ncode(const char *, bpf_u_int32, struct qual);
struct block *gen_proto_abbrev(int);
struct block *gen_relation(int, struct arth *, struct arth *, int);
struct block *gen_less(int);
struct block *gen_greater(int);
struct block *gen_byteop(int, int, int);
struct block *gen_broadcast(int);
struct block *gen_multicast(int);
struct block *gen_inbound(int);

struct block *gen_vlan(int);
struct block *gen_mpls(int);

struct block *gen_pppoed(void);
struct block *gen_pppoes(void);

struct block *gen_atmfield_code(int atmfield, bpf_int32 jvalue, bpf_u_int32 jtype, int reverse);
struct block *gen_atmtype_abbrev(int type);
struct block *gen_atmmulti_abbrev(int type);

struct block *gen_mtp2type_abbrev(int type);
struct block *gen_mtp3field_code(int mtp3field, bpf_u_int32 jvalue, bpf_u_int32 jtype, int reverse);

struct block *gen_pf_ifname(const char *);
struct block *gen_pf_rnr(int);
struct block *gen_pf_srnr(int);
struct block *gen_pf_ruleset(char *);
struct block *gen_pf_reason(int);
struct block *gen_pf_action(int);
struct block *gen_pf_dir(int);

void bpf_optimize(struct block **);
void bpf_error(const char *, ...)
    __attribute__((noreturn, format (printf, 1, 2)));

void finish_parse(struct block *);
char *sdup(const char *);

struct bpf_insn *icode_to_fcode(struct block *, int *);
int pcap_parse(void);
void lex_init(const char *);
void lex_cleanup(void);
void sappend(struct slist *, struct slist *);

/* XXX */
#define JT(b)  ((b)->et.succ)
#define JF(b)  ((b)->ef.succ)

extern int no_optimize;
