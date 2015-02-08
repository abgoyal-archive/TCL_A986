
#ifndef LINUX_NFSD_AUTH_H
#define LINUX_NFSD_AUTH_H

#ifdef __KERNEL__

#define nfsd_luid(rq, uid)	((u32)(uid))
#define nfsd_lgid(rq, gid)	((u32)(gid))
#define nfsd_ruid(rq, uid)	((u32)(uid))
#define nfsd_rgid(rq, gid)	((u32)(gid))

int nfsd_setuser(struct svc_rqst *, struct svc_export *);

#endif /* __KERNEL__ */
#endif /* LINUX_NFSD_AUTH_H */
