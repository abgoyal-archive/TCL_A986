/*	$NetBSD: str2val.h,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/* Id: str2val.h,v 1.3 2004/06/11 16:00:17 ludvigm Exp */


#ifndef _STR2VAL_H
#define _STR2VAL_H

extern caddr_t val2str __P((const char *, size_t));
extern char *str2val __P((const char *, int, size_t *));

#endif /* _STR2VAL_H */
