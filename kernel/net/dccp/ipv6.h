
#ifndef _DCCP_IPV6_H
#define _DCCP_IPV6_H

#include <linux/dccp.h>
#include <linux/ipv6.h>

struct dccp6_sock {
	struct dccp_sock  dccp;
	/*
	 * ipv6_pinfo has to be the last member of dccp6_sock,
	 * see inet6_sk_generic.
	 */
	struct ipv6_pinfo inet6;
};

struct dccp6_request_sock {
	struct dccp_request_sock  dccp;
	struct inet6_request_sock inet6;
};

struct dccp6_timewait_sock {
	struct inet_timewait_sock   inet;
	struct inet6_timewait_sock  tw6;
};

#endif /* _DCCP_IPV6_H */
