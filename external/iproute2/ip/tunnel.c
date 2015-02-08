
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <linux/if.h>
#include <linux/ip.h>
#include <linux/if_tunnel.h>

#include "utils.h"
#include "tunnel.h"

const char *tnl_strproto(__u8 proto)
{
	static char buf[16];

	switch (proto) {
	case IPPROTO_IPIP:
		strcpy(buf, "ip");
		break;
	case IPPROTO_GRE:
		strcpy(buf, "gre");
		break;
	case IPPROTO_IPV6:
		strcpy(buf, "ipv6");
		break;
	case 0:
		strcpy(buf, "any");
		break;
	default:
		strcpy(buf, "unknown");
		break;
	}

	return buf;
}

int tnl_ioctl_get_ifindex(const char *dev)
{
	struct ifreq ifr;
	int fd;
	int err;

	strncpy(ifr.ifr_name, dev, IFNAMSIZ);
	fd = socket(preferred_family, SOCK_DGRAM, 0);
	err = ioctl(fd, SIOCGIFINDEX, &ifr);
	if (err) {
		perror("ioctl");
		return 0;
	}
	close(fd);
	return ifr.ifr_ifindex;
}

int tnl_ioctl_get_iftype(const char *dev)
{
	struct ifreq ifr;
	int fd;
	int err;

	strncpy(ifr.ifr_name, dev, IFNAMSIZ);
	fd = socket(preferred_family, SOCK_DGRAM, 0);
	err = ioctl(fd, SIOCGIFHWADDR, &ifr);
	if (err) {
		perror("ioctl");
		return -1;
	}
	close(fd);
	return ifr.ifr_addr.sa_family;
}


char * tnl_ioctl_get_ifname(int idx)
{
	static struct ifreq ifr;
	int fd;
	int err;

	ifr.ifr_ifindex = idx;
	fd = socket(preferred_family, SOCK_DGRAM, 0);
	err = ioctl(fd, SIOCGIFNAME, &ifr);
	if (err) {
		perror("ioctl");
		return NULL;
	}
	close(fd);
	return ifr.ifr_name;
}

int tnl_get_ioctl(const char *basedev, void *p)
{
	struct ifreq ifr;
	int fd;
	int err;

	strncpy(ifr.ifr_name, basedev, IFNAMSIZ);
	ifr.ifr_ifru.ifru_data = (void*)p;
	fd = socket(preferred_family, SOCK_DGRAM, 0);
	err = ioctl(fd, SIOCGETTUNNEL, &ifr);
	if (err)
		perror("ioctl");
	close(fd);
	return err;
}

int tnl_add_ioctl(int cmd, const char *basedev, const char *name, void *p)
{
	struct ifreq ifr;
	int fd;
	int err;

	if (cmd == SIOCCHGTUNNEL && name[0])
		strncpy(ifr.ifr_name, name, IFNAMSIZ);
	else
		strncpy(ifr.ifr_name, basedev, IFNAMSIZ);
	ifr.ifr_ifru.ifru_data = p;
	fd = socket(preferred_family, SOCK_DGRAM, 0);
	err = ioctl(fd, cmd, &ifr);
	if (err)
		perror("ioctl");
	close(fd);
	return err;
}

int tnl_del_ioctl(const char *basedev, const char *name, void *p)
{
	struct ifreq ifr;
	int fd;
	int err;

	if (name[0])
		strncpy(ifr.ifr_name, name, IFNAMSIZ);
	else
		strncpy(ifr.ifr_name, basedev, IFNAMSIZ);
	ifr.ifr_ifru.ifru_data = p;
	fd = socket(preferred_family, SOCK_DGRAM, 0);
	err = ioctl(fd, SIOCDELTUNNEL, &ifr);
	if (err)
		perror("ioctl");
	close(fd);
	return err;
}

int tnl_prl_ioctl(int cmd, const char *name, void *p)
{
	struct ifreq ifr;
	int fd;
	int err;

	strncpy(ifr.ifr_name, name, IFNAMSIZ);
	ifr.ifr_ifru.ifru_data = p;
	fd = socket(preferred_family, SOCK_DGRAM, 0);
	err = ioctl(fd, cmd, &ifr);
	if (err)
		perror("ioctl");
	close(fd);
	return err;
}
