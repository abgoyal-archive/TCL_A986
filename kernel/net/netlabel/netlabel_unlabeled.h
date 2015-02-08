


#ifndef _NETLABEL_UNLABELED_H
#define _NETLABEL_UNLABELED_H

#include <net/netlabel.h>


/* NetLabel Unlabeled commands */
enum {
	NLBL_UNLABEL_C_UNSPEC,
	NLBL_UNLABEL_C_ACCEPT,
	NLBL_UNLABEL_C_LIST,
	NLBL_UNLABEL_C_STATICADD,
	NLBL_UNLABEL_C_STATICREMOVE,
	NLBL_UNLABEL_C_STATICLIST,
	NLBL_UNLABEL_C_STATICADDDEF,
	NLBL_UNLABEL_C_STATICREMOVEDEF,
	NLBL_UNLABEL_C_STATICLISTDEF,
	__NLBL_UNLABEL_C_MAX,
};
#define NLBL_UNLABEL_C_MAX (__NLBL_UNLABEL_C_MAX - 1)

/* NetLabel Unlabeled attributes */
enum {
	NLBL_UNLABEL_A_UNSPEC,
	NLBL_UNLABEL_A_ACPTFLG,
	/* (NLA_U8)
	 * if true then unlabeled packets are allowed to pass, else unlabeled
	 * packets are rejected */
	NLBL_UNLABEL_A_IPV6ADDR,
	/* (NLA_BINARY, struct in6_addr)
	 * an IPv6 address */
	NLBL_UNLABEL_A_IPV6MASK,
	/* (NLA_BINARY, struct in6_addr)
	 * an IPv6 address mask */
	NLBL_UNLABEL_A_IPV4ADDR,
	/* (NLA_BINARY, struct in_addr)
	 * an IPv4 address */
	NLBL_UNLABEL_A_IPV4MASK,
	/* (NLA_BINARY, struct in_addr)
	 * and IPv4 address mask */
	NLBL_UNLABEL_A_IFACE,
	/* (NLA_NULL_STRING)
	 * network interface */
	NLBL_UNLABEL_A_SECCTX,
	/* (NLA_BINARY)
	 * a LSM specific security context */
	__NLBL_UNLABEL_A_MAX,
};
#define NLBL_UNLABEL_A_MAX (__NLBL_UNLABEL_A_MAX - 1)

/* NetLabel protocol functions */
int netlbl_unlabel_genl_init(void);

/* Unlabeled connection hash table size */
/* XXX - currently this number is an uneducated guess */
#define NETLBL_UNLHSH_BITSIZE       7

/* General Unlabeled init function */
int netlbl_unlabel_init(u32 size);

/* Static/Fallback label management functions */
int netlbl_unlhsh_add(struct net *net,
		      const char *dev_name,
		      const void *addr,
		      const void *mask,
		      u32 addr_len,
		      u32 secid,
		      struct netlbl_audit *audit_info);
int netlbl_unlhsh_remove(struct net *net,
			 const char *dev_name,
			 const void *addr,
			 const void *mask,
			 u32 addr_len,
			 struct netlbl_audit *audit_info);

/* Process Unlabeled incoming network packets */
int netlbl_unlabel_getattr(const struct sk_buff *skb,
			   u16 family,
			   struct netlbl_lsm_secattr *secattr);

/* Set the default configuration to allow Unlabeled packets */
int netlbl_unlabel_defconf(void);

#endif
