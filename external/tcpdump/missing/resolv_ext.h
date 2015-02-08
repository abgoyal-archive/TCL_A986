

#ifndef MAXRESOLVSORT		/* XXX */
#define	MAXRESOLVSORT		10	/* number of net to sort on */
#endif

#ifndef HAVE_RES_STATE_EXT
struct __res_state_ext {
	struct sockaddr_storage nsaddr_list[MAXNS];
	struct {
		int af;		/* address family for addr, mask */
		union {
			struct in_addr ina;
#ifdef INET6
			struct in6_addr in6a;
#endif
		} addr, mask;
	} sort_list[MAXRESOLVSORT];
};
#endif

extern struct __res_state_ext _res_ext;
