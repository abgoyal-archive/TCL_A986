/*	$NetBSD: isakmp_ident.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/* Id: isakmp_ident.h,v 1.3 2004/06/11 16:00:16 ludvigm Exp */


#ifndef _ISAKMP_IDENT_H
#define _ISAKMP_IDENT_H

extern int ident_i1send __P((struct ph1handle *, vchar_t *));
extern int ident_i2recv __P((struct ph1handle *, vchar_t *));
extern int ident_i2send __P((struct ph1handle *, vchar_t *));
extern int ident_i3recv __P((struct ph1handle *, vchar_t *));
extern int ident_i3send __P((struct ph1handle *, vchar_t *));
extern int ident_i4recv __P((struct ph1handle *, vchar_t *));
extern int ident_i4send __P((struct ph1handle *, vchar_t *));

extern int ident_r1recv __P((struct ph1handle *, vchar_t *));
extern int ident_r1send __P((struct ph1handle *, vchar_t *));
extern int ident_r2recv __P((struct ph1handle *, vchar_t *));
extern int ident_r2send __P((struct ph1handle *, vchar_t *));
extern int ident_r3recv __P((struct ph1handle *, vchar_t *));
extern int ident_r3send __P((struct ph1handle *, vchar_t *));

#endif /* _ISAKMP_IDENT_H */
