

#ifndef IP_ADDR_H
#define IP_ADDR_H

struct hostapd_ip_addr {
	union {
		struct in_addr v4;
#ifdef CONFIG_IPV6
		struct in6_addr v6;
#endif /* CONFIG_IPV6 */
	} u;
	int af; /* AF_INET / AF_INET6 */
};

const char * hostapd_ip_txt(const struct hostapd_ip_addr *addr, char *buf,
			    size_t buflen);
int hostapd_ip_diff(struct hostapd_ip_addr *a, struct hostapd_ip_addr *b);
int hostapd_parse_ip_addr(const char *txt, struct hostapd_ip_addr *addr);

#endif /* IP_ADDR_H */
