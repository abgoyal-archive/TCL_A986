#ifndef __TUNNEL_H__
#define __TUNNEL_H__ 1

#include <linux/types.h>

const char *tnl_strproto(__u8 proto);
int tnl_ioctl_get_ifindex(const char *dev);
int tnl_ioctl_get_iftype(const char *dev);
char * tnl_ioctl_get_ifname(int idx);
int tnl_get_ioctl(const char *basedev, void *p);
int tnl_add_ioctl(int cmd, const char *basedev, const char *name, void *p);
int tnl_del_ioctl(const char *basedev, const char *name, void *p);
int tnl_prl_ioctl(int cmd, const char *name, void *p);

#endif
