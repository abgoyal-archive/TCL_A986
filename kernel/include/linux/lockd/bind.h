

#ifndef LINUX_LOCKD_BIND_H
#define LINUX_LOCKD_BIND_H

#include <linux/lockd/nlm.h>
/* need xdr-encoded error codes too, so... */
#include <linux/lockd/xdr.h>
#ifdef CONFIG_LOCKD_V4
#include <linux/lockd/xdr4.h>
#endif

/* Dummy declarations */
struct svc_rqst;

struct nlmsvc_binding {
	__be32			(*fopen)(struct svc_rqst *,
						struct nfs_fh *,
						struct file **);
	void			(*fclose)(struct file *);
};

extern struct nlmsvc_binding *	nlmsvc_ops;

struct nlmclnt_initdata {
	const char		*hostname;
	const struct sockaddr	*address;
	size_t			addrlen;
	unsigned short		protocol;
	u32			nfs_version;
	int			noresvport;
};


extern struct nlm_host *nlmclnt_init(const struct nlmclnt_initdata *nlm_init);
extern void	nlmclnt_done(struct nlm_host *host);

extern int	nlmclnt_proc(struct nlm_host *host, int cmd,
					struct file_lock *fl);
extern int	lockd_up(void);
extern void	lockd_down(void);

#endif /* LINUX_LOCKD_BIND_H */
