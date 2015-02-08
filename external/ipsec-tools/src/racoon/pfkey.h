/*	$NetBSD: pfkey.h,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/* Id: pfkey.h,v 1.3 2004/06/11 16:00:17 ludvigm Exp */


#ifndef _PFKEY_H
#define _PFKEY_H

struct pfkey_satype {
	u_int8_t	ps_satype;
	const char	*ps_name;
};

extern const struct pfkey_satype pfkey_satypes[];
extern const int pfkey_nsatypes;

extern int pfkey_handler __P((void));
extern vchar_t *pfkey_dump_sadb __P((int));
extern void pfkey_flush_sadb __P((u_int));
extern int pfkey_init __P((void));

extern struct pfkey_st *pfkey_getpst __P((caddr_t *, int, int));

extern int pk_checkalg __P((int, int, int));

struct ph2handle;
extern int pk_sendgetspi __P((struct ph2handle *));
extern int pk_sendupdate __P((struct ph2handle *));
extern int pk_sendadd __P((struct ph2handle *));
extern int pk_sendeacquire __P((struct ph2handle *));
extern int pk_sendspdupdate2 __P((struct ph2handle *));
extern int pk_sendspdadd2 __P((struct ph2handle *));
extern int pk_sendspddelete __P((struct ph2handle *));

extern void pfkey_timeover_stub __P((void *));
extern void pfkey_timeover __P((struct ph2handle *));

extern u_int pfkey2ipsecdoi_proto __P((u_int));
extern u_int ipsecdoi2pfkey_proto __P((u_int));
extern u_int pfkey2ipsecdoi_mode __P((u_int));
extern u_int ipsecdoi2pfkey_mode __P((u_int));

extern int pfkey_convertfromipsecdoi __P(( u_int, u_int, u_int,
	u_int *, u_int *, u_int *, u_int *, u_int *));
extern u_int32_t pk_getseq __P((void));
extern const char *sadbsecas2str
	__P((struct sockaddr *, struct sockaddr *, int, u_int32_t, int));

#endif /* _PFKEY_H */
