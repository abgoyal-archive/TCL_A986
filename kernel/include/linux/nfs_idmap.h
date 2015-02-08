

#ifndef NFS_IDMAP_H
#define NFS_IDMAP_H

#include <linux/types.h>

/* XXX from bits/utmp.h  */
#define IDMAP_NAMESZ  128

#define IDMAP_TYPE_USER  0
#define IDMAP_TYPE_GROUP 1

#define IDMAP_CONV_IDTONAME 0
#define IDMAP_CONV_NAMETOID 1

#define IDMAP_STATUS_INVALIDMSG 0x01
#define IDMAP_STATUS_AGAIN      0x02
#define IDMAP_STATUS_LOOKUPFAIL 0x04
#define IDMAP_STATUS_SUCCESS    0x08

struct idmap_msg {
	__u8  im_type;
	__u8  im_conv;
	char  im_name[IDMAP_NAMESZ];
	__u32 im_id;
	__u8  im_status;
};

#ifdef __KERNEL__

/* Forward declaration to make this header independent of others */
struct nfs_client;

int nfs_idmap_new(struct nfs_client *);
void nfs_idmap_delete(struct nfs_client *);

int nfs_map_name_to_uid(struct nfs_client *, const char *, size_t, __u32 *);
int nfs_map_group_to_gid(struct nfs_client *, const char *, size_t, __u32 *);
int nfs_map_uid_to_name(struct nfs_client *, __u32, char *);
int nfs_map_gid_to_group(struct nfs_client *, __u32, char *);

extern unsigned int nfs_idmap_cache_timeout;
#endif /* __KERNEL__ */

#endif /* NFS_IDMAP_H */
