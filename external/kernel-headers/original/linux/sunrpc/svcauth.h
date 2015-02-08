
#ifndef _LINUX_SUNRPC_SVCAUTH_H_
#define _LINUX_SUNRPC_SVCAUTH_H_

#ifdef __KERNEL__

#include <linux/string.h>
#include <linux/sunrpc/msg_prot.h>
#include <linux/sunrpc/cache.h>
#include <linux/hash.h>

#define SVC_CRED_NGROUPS	32
struct svc_cred {
	uid_t			cr_uid;
	gid_t			cr_gid;
	struct group_info	*cr_group_info;
};

struct svc_rqst;		/* forward decl */

struct auth_domain {
	struct kref		ref;
	struct hlist_node	hash;
	char			*name;
	struct auth_ops		*flavour;
};

struct auth_ops {
	char *	name;
	struct module *owner;
	int	flavour;
	int	(*accept)(struct svc_rqst *rq, u32 *authp);
	int	(*release)(struct svc_rqst *rq);
	void	(*domain_release)(struct auth_domain *);
	int	(*set_client)(struct svc_rqst *rq);
};

#define	SVC_GARBAGE	1
#define	SVC_SYSERR	2
#define	SVC_VALID	3
#define	SVC_NEGATIVE	4
#define	SVC_OK		5
#define	SVC_DROP	6
#define	SVC_DENIED	7
#define	SVC_PENDING	8
#define	SVC_COMPLETE	9


extern int	svc_authenticate(struct svc_rqst *rqstp, u32 *authp);
extern int	svc_authorise(struct svc_rqst *rqstp);
extern int	svc_set_client(struct svc_rqst *rqstp);
extern int	svc_auth_register(rpc_authflavor_t flavor, struct auth_ops *aops);
extern void	svc_auth_unregister(rpc_authflavor_t flavor);

extern struct auth_domain *unix_domain_find(char *name);
extern void auth_domain_put(struct auth_domain *item);
extern int auth_unix_add_addr(struct in_addr addr, struct auth_domain *dom);
extern struct auth_domain *auth_domain_lookup(char *name, struct auth_domain *new);
extern struct auth_domain *auth_domain_find(char *name);
extern struct auth_domain *auth_unix_lookup(struct in_addr addr);
extern int auth_unix_forget_old(struct auth_domain *dom);
extern void svcauth_unix_purge(void);

static inline unsigned long hash_str(char *name, int bits)
{
	unsigned long hash = 0;
	unsigned long l = 0;
	int len = 0;
	unsigned char c;
	do {
		if (unlikely(!(c = *name++))) {
			c = (char)len; len = -1;
		}
		l = (l << 8) | c;
		len++;
		if ((len & (BITS_PER_LONG/8-1))==0)
			hash = hash_long(hash^l, BITS_PER_LONG);
	} while (len);
	return hash >> (BITS_PER_LONG - bits);
}

static inline unsigned long hash_mem(char *buf, int length, int bits)
{
	unsigned long hash = 0;
	unsigned long l = 0;
	int len = 0;
	unsigned char c;
	do {
		if (len == length) {
			c = (char)len; len = -1;
		} else
			c = *buf++;
		l = (l << 8) | c;
		len++;
		if ((len & (BITS_PER_LONG/8-1))==0)
			hash = hash_long(hash^l, BITS_PER_LONG);
	} while (len);
	return hash >> (BITS_PER_LONG - bits);
}

#endif /* __KERNEL__ */

#endif /* _LINUX_SUNRPC_SVCAUTH_H_ */
