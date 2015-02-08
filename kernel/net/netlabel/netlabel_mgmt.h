


#ifndef _NETLABEL_MGMT_H
#define _NETLABEL_MGMT_H

#include <net/netlabel.h>
#include <asm/atomic.h>


/* NetLabel Management commands */
enum {
	NLBL_MGMT_C_UNSPEC,
	NLBL_MGMT_C_ADD,
	NLBL_MGMT_C_REMOVE,
	NLBL_MGMT_C_LISTALL,
	NLBL_MGMT_C_ADDDEF,
	NLBL_MGMT_C_REMOVEDEF,
	NLBL_MGMT_C_LISTDEF,
	NLBL_MGMT_C_PROTOCOLS,
	NLBL_MGMT_C_VERSION,
	__NLBL_MGMT_C_MAX,
};
#define NLBL_MGMT_C_MAX (__NLBL_MGMT_C_MAX - 1)

/* NetLabel Management attributes */
enum {
	NLBL_MGMT_A_UNSPEC,
	NLBL_MGMT_A_DOMAIN,
	/* (NLA_NUL_STRING)
	 * the NULL terminated LSM domain string */
	NLBL_MGMT_A_PROTOCOL,
	/* (NLA_U32)
	 * the NetLabel protocol type (defined by NETLBL_NLTYPE_*) */
	NLBL_MGMT_A_VERSION,
	/* (NLA_U32)
	 * the NetLabel protocol version number (defined by
	 * NETLBL_PROTO_VERSION) */
	NLBL_MGMT_A_CV4DOI,
	/* (NLA_U32)
	 * the CIPSOv4 DOI value */
	NLBL_MGMT_A_IPV6ADDR,
	/* (NLA_BINARY, struct in6_addr)
	 * an IPv6 address */
	NLBL_MGMT_A_IPV6MASK,
	/* (NLA_BINARY, struct in6_addr)
	 * an IPv6 address mask */
	NLBL_MGMT_A_IPV4ADDR,
	/* (NLA_BINARY, struct in_addr)
	 * an IPv4 address */
	NLBL_MGMT_A_IPV4MASK,
	/* (NLA_BINARY, struct in_addr)
	 * and IPv4 address mask */
	NLBL_MGMT_A_ADDRSELECTOR,
	/* (NLA_NESTED)
	 * an IP address selector, must contain an address, mask, and protocol
	 * attribute plus any protocol specific attributes */
	NLBL_MGMT_A_SELECTORLIST,
	/* (NLA_NESTED)
	 * the selector list, there must be at least one
	 * NLBL_MGMT_A_ADDRSELECTOR attribute */
	__NLBL_MGMT_A_MAX,
};
#define NLBL_MGMT_A_MAX (__NLBL_MGMT_A_MAX - 1)

/* NetLabel protocol functions */
int netlbl_mgmt_genl_init(void);

/* NetLabel configured protocol reference counter */
extern atomic_t netlabel_mgmt_protocount;

#endif
