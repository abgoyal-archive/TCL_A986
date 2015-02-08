/*	$NetBSD: cftoken_proto.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/* Id: cftoken_proto.h,v 1.3 2004/06/11 16:00:15 ludvigm Exp */


#ifndef _CFTOKEN_PROTO_H
#define _CFTOKEN_PROTO_H

extern int yyerrorcount;

extern int yylex __P((void));
extern void yyerror __P((char *, ...));
extern void yywarn __P((char *, ...));

extern int yycf_switch_buffer __P((char *));
extern int yycf_set_buffer __P((char *));
extern void yycf_init_buffer __P((void));
extern void yycf_clean_buffer __P((void));

#endif /* _CFTOKEN_PROTO_H */
