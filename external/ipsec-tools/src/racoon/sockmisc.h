/*	$NetBSD: sockmisc.h,v 1.7 2006/09/09 16:22:10 manu Exp $	*/

/* Id: sockmisc.h,v 1.9 2005/10/05 16:55:41 manubsd Exp */


#ifndef _SOCKMISC_H
#define _SOCKMISC_H

struct netaddr {
	union {
		struct sockaddr sa;
		struct sockaddr_in sin;
		struct sockaddr_in6 sin6;
	} sa;
	unsigned long prefix;
};

extern const int niflags;

extern int cmpsaddrwop __P((const struct sockaddr *, const struct sockaddr *));
extern int cmpsaddrwild __P((const struct sockaddr *, const struct sockaddr *));
extern int cmpsaddrstrict __P((const struct sockaddr *, const struct sockaddr *));

#ifdef ENABLE_NATT 
#define CMPSADDR(saddr1, saddr2) cmpsaddrstrict((saddr1), (saddr2))
#else 
#define CMPSADDR(saddr1, saddr2) cmpsaddrwop((saddr1), (saddr2))
#endif

extern struct sockaddr *getlocaladdr __P((struct sockaddr *));

extern int recvfromto __P((int, void *, size_t, int,
	struct sockaddr *, socklen_t *, struct sockaddr *, unsigned int *));
extern int sendfromto __P((int, const void *, size_t,
	struct sockaddr *, struct sockaddr *, int));

extern int setsockopt_bypass __P((int, int));

extern struct sockaddr *newsaddr __P((int));
extern struct sockaddr *dupsaddr __P((struct sockaddr *));
extern char *saddr2str __P((const struct sockaddr *));
extern char *saddrwop2str __P((const struct sockaddr *));
extern char *saddr2str_fromto __P((const char *format, 
				   const struct sockaddr *saddr, 
				   const struct sockaddr *daddr));
extern struct sockaddr *str2saddr __P((char *, char *));
extern void mask_sockaddr __P((struct sockaddr *, const struct sockaddr *,
	size_t));

/* struct netaddr functions */
extern char *naddrwop2str __P((const struct netaddr *naddr));
extern char *naddrwop2str_fromto __P((const char *format, const struct netaddr *saddr,
				      const struct netaddr *daddr));
extern int naddr_score(const struct netaddr *naddr, const struct sockaddr *saddr);

/* Some usefull functions for sockaddr port manipulations. */
extern u_int16_t extract_port __P((const struct sockaddr *addr));
extern u_int16_t *set_port __P((struct sockaddr *addr, u_int16_t new_port));
extern u_int16_t *get_port_ptr __P((struct sockaddr *addr));

#endif /* _SOCKMISC_H */
