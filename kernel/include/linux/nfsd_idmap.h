

#ifndef LINUX_NFSD_IDMAP_H
#define LINUX_NFSD_IDMAP_H

#include <linux/in.h>
#include <linux/sunrpc/svc.h>

/* XXX from linux/nfs_idmap.h */
#define IDMAP_NAMESZ 128

#ifdef CONFIG_NFSD_V4
int nfsd_idmap_init(void);
void nfsd_idmap_shutdown(void);
#else
static inline int nfsd_idmap_init(void)
{
	return 0;
}
static inline void nfsd_idmap_shutdown(void)
{
}
#endif

int nfsd_map_name_to_uid(struct svc_rqst *, const char *, size_t, __u32 *);
int nfsd_map_name_to_gid(struct svc_rqst *, const char *, size_t, __u32 *);
int nfsd_map_uid_to_name(struct svc_rqst *, __u32, char *);
int nfsd_map_gid_to_name(struct svc_rqst *, __u32, char *);

#endif /* LINUX_NFSD_IDMAP_H */
