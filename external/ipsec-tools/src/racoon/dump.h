/*	$NetBSD: dump.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/* Id: dump.h,v 1.3 2004/06/11 16:00:16 ludvigm Exp */


#ifndef _DUMP_H
#define _DUMP_H

extern int isakmp_dump_open __P((char *));
extern int isakmp_dump_close __P((void));
extern int isakmp_dump __P((vchar_t *, struct sockaddr *, struct sockaddr *));

#endif /* _DUMP_H */
