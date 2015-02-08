/*	$NetBSD: isakmp_agg.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/* Id: isakmp_agg.h,v 1.3 2004/06/11 16:00:16 ludvigm Exp */


#ifndef _ISAKMP_AGG_H
#define _ISAKMP_AGG_H

extern int agg_i1send __P((struct ph1handle *, vchar_t *));
extern int agg_i2recv __P((struct ph1handle *, vchar_t *));
extern int agg_i2send __P((struct ph1handle *, vchar_t *));

extern int agg_r1recv __P((struct ph1handle *, vchar_t *));
extern int agg_r1send __P((struct ph1handle *, vchar_t *));
extern int agg_r2recv __P((struct ph1handle *, vchar_t *));
extern int agg_r2send __P((struct ph1handle *, vchar_t *));

#endif /* _ISAKMP_AGG_H */
