/*	$NetBSD: sainfo.h,v 1.5 2006/10/03 08:01:56 vanhu Exp $	*/

/* Id: sainfo.h,v 1.5 2006/07/09 17:19:38 manubsd Exp */


#ifndef _SAINFO_H
#define _SAINFO_H

#include <sys/queue.h>

/* SA info */
struct sainfo {
	vchar_t *idsrc;
	vchar_t *iddst;
		/*
		 * idsrc and iddst are constructed body of ID payload.
		 * that is (struct ipsecdoi_id_b) + ID value.
		 * If idsrc == NULL, that is anonymous entry.
		 */

#ifdef ENABLE_HYBRID
	vchar_t *group;
#endif

	time_t lifetime;
	int lifebyte;
	int pfs_group;		/* only use when pfs is required. */
	vchar_t *id_i;		/* identifier of the authorized initiator */
	struct sainfoalg *algs[MAXALGCLASS];

	int remoteid;

	LIST_ENTRY(sainfo) chain;
};

/* algorithm type */
struct sainfoalg {
	int alg;
	int encklen;			/* key length if encryption algorithm */
	struct sainfoalg *next;
};

extern struct sainfo *getsainfo __P((const vchar_t *,
	const vchar_t *, const vchar_t *, int));
extern struct sainfo *newsainfo __P((void));
extern void delsainfo __P((struct sainfo *));
extern void inssainfo __P((struct sainfo *));
extern void remsainfo __P((struct sainfo *));
extern void flushsainfo __P((void));
extern void initsainfo __P((void));
extern struct sainfoalg *newsainfoalg __P((void));
extern void delsainfoalg __P((struct sainfoalg *));
extern void inssainfoalg __P((struct sainfoalg **, struct sainfoalg *));
extern const char * sainfo2str __P((const struct sainfo *));

extern void save_sainfotree __P((void));
extern void save_sainfotree_flush __P((void));
extern void save_sainfotree_restore __P((void));

#endif /* _SAINFO_H */
