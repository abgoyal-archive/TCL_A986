/*	$NetBSD: cfparse_proto.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/* Id: cfparse_proto.h,v 1.3 2004/06/11 16:00:15 ludvigm Exp */


#ifndef _CFPARSE_PROTO_H
#define _CFPARSE_PROTO_H

/* cfparse.y */
extern int yyparse __P((void));
extern int cfparse __P((void));
extern int cfreparse __P((void));

#endif /* _CFPARSE_PROTO_H */
