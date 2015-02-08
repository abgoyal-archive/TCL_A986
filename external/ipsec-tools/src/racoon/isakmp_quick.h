/*	$NetBSD: isakmp_quick.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/* Id: isakmp_quick.h,v 1.3 2004/06/11 16:00:16 ludvigm Exp */


#ifndef _ISAKMP_QUICK_H
#define _ISAKMP_QUICK_H

extern int quick_i1prep __P((struct ph2handle *, vchar_t *));
extern int quick_i1send __P((struct ph2handle *, vchar_t *));
extern int quick_i2recv __P((struct ph2handle *, vchar_t *));
extern int quick_i2send __P((struct ph2handle *, vchar_t *));
extern int quick_i3recv __P((struct ph2handle *, vchar_t *));

extern int quick_r1recv __P((struct ph2handle *, vchar_t *));
extern int quick_r1prep __P((struct ph2handle *, vchar_t *));
extern int quick_r2send __P((struct ph2handle *, vchar_t *));
extern int quick_r3recv __P((struct ph2handle *, vchar_t *));
extern int quick_r3send __P((struct ph2handle *, vchar_t *));
extern int quick_r3prep __P((struct ph2handle *, vchar_t *));

#endif /* _ISAKMP_QUICK_H */
