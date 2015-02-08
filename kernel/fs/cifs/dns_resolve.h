

#ifndef _DNS_RESOLVE_H
#define _DNS_RESOLVE_H

#ifdef __KERNEL__
extern int __init cifs_init_dns_resolver(void);
extern void cifs_exit_dns_resolver(void);
extern int dns_resolve_server_name_to_ip(const char *unc, char **ip_addr);
#endif /* KERNEL */

#endif /* _DNS_RESOLVE_H */
