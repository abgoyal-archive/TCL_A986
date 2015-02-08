/*	$NetBSD: proposal.h,v 1.6 2006/12/09 05:52:57 manu Exp $	*/

/* Id: proposal.h,v 1.5 2004/06/11 16:00:17 ludvigm Exp */


#ifndef _PROPOSAL_H
#define _PROPOSAL_H

#include <sys/queue.h>

/* SA proposal specification */
struct saprop {
	int prop_no;
	time_t lifetime;
	int lifebyte;
	int pfs_group;			/* pfs group */
	int claim;			/* flag to send RESPONDER-LIFETIME. */
					/* XXX assumed DOI values are 1 or 2. */
#ifdef HAVE_SECCTX
	struct security_ctx sctx;       /* security context structure */
#endif
	struct saproto *head;
	struct saprop *next;
};

/* SA protocol specification */
struct saproto {
	int proto_id;
	size_t spisize;			/* spi size */
	int encmode;			/* encryption mode */

	int udp_encap;			/* UDP encapsulation */

	/* XXX should be vchar_t * */
	/* these are network byte order */
	u_int32_t spi;			/* inbound. i.e. --SA-> me */
	u_int32_t spi_p;		/* outbound. i.e. me -SA-> */

	vchar_t *keymat;		/* KEYMAT */
	vchar_t *keymat_p;		/* peer's KEYMAT */

	int reqid_out;			/* request id (outbound) */
	int reqid_in;			/* request id (inbound) */

	int ok;				/* if 1, success to set SA in kenrel */

	struct satrns *head;		/* header of transform */
	struct saproto *next;		/* next protocol */
};

/* SA algorithm specification */
struct satrns {
	int trns_no;
	int trns_id;			/* transform id */
	int encklen;			/* key length of encryption algorithm */
	int authtype;			/* authentication algorithm if ESP */

	struct satrns *next;		/* next transform */
};

struct prop_pair {
	struct isakmp_pl_p *prop;
	struct isakmp_pl_t *trns;
	struct prop_pair *next;	/* next prop_pair with same proposal # */
				/* (bundle case) */
	struct prop_pair *tnext; /* next prop_pair in same proposal payload */
				/* (multiple tranform case) */
};
#define MAXPROPPAIRLEN	256	/* It's enough because field size is 1 octet. */

#define PROP_CHECK_OBEY		1
#define PROP_CHECK_STRICT	2
#define PROP_CHECK_CLAIM	3
#define PROP_CHECK_EXACT	4

struct sainfo;
struct ph1handle;
struct secpolicy;
extern struct saprop *newsaprop __P((void));
extern struct saproto *newsaproto __P((void));
extern void inssaprop __P((struct saprop **, struct saprop *));
extern void inssaproto __P((struct saprop *, struct saproto *));
extern void inssaprotorev __P((struct saprop *, struct saproto *));
extern struct satrns *newsatrns __P((void));
extern void inssatrns __P((struct saproto *, struct satrns *));
extern struct saprop *cmpsaprop_alloc __P((struct ph1handle *,
	const struct saprop *, const struct saprop *, int));
extern int cmpsaprop __P((const struct saprop *, const struct saprop *));
extern int cmpsatrns __P((int, const struct satrns *, const struct satrns *, int));
extern int set_satrnsbysainfo __P((struct saproto *, struct sainfo *));
extern struct saprop *aproppair2saprop __P((struct prop_pair *));
extern void free_proppair __P((struct prop_pair **));
extern void flushsaprop __P((struct saprop *));
extern void flushsaproto __P((struct saproto *));
extern void flushsatrns __P((struct satrns *));
extern void printsaprop __P((const int, const struct saprop *));
extern void printsaprop0 __P((const int, const struct saprop *));
extern void printsaproto __P((const int, const struct saproto *));
extern void printsatrns __P((const int, const int, const struct satrns *));
extern void print_proppair0 __P((int, struct prop_pair *, int));
extern void print_proppair __P((int, struct prop_pair *));
extern int set_proposal_from_policy __P((struct ph2handle *,
	struct secpolicy *, struct secpolicy *));
extern int set_proposal_from_proposal __P((struct ph2handle *));

#endif /* _PROPOSAL_H */
