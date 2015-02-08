/*	$NetBSD: grabmyaddr.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/* Id: grabmyaddr.h,v 1.5 2004/06/11 16:00:16 ludvigm Exp */


#ifndef _GRABMYADDR_H
#define _GRABMYADDR_H

struct myaddrs {
	struct myaddrs *next;
	struct sockaddr *addr;
	int sock;
	int udp_encap;
};

extern void clear_myaddr __P((struct myaddrs **));
extern void grab_myaddrs __P((void));
extern int update_myaddrs __P((void));
extern int autoconf_myaddrsport __P((void));
extern u_short getmyaddrsport __P((struct sockaddr *));
extern struct myaddrs *newmyaddr __P((void));
extern struct myaddrs *dupmyaddr __P((struct myaddrs *));
extern void insmyaddr __P((struct myaddrs *, struct myaddrs **));
extern void delmyaddr __P((struct myaddrs *));
extern int initmyaddr __P((void));
extern int getsockmyaddr __P((struct sockaddr *));

#endif /* _GRABMYADDR_H */
