

#if !defined(IB_ADDR_H)
#define IB_ADDR_H

#include <linux/in.h>
#include <linux/in6.h>
#include <linux/if_arp.h>
#include <linux/netdevice.h>
#include <linux/socket.h>
#include <rdma/ib_verbs.h>

struct rdma_addr_client {
	atomic_t refcount;
	struct completion comp;
};

void rdma_addr_register_client(struct rdma_addr_client *client);

void rdma_addr_unregister_client(struct rdma_addr_client *client);

struct rdma_dev_addr {
	unsigned char src_dev_addr[MAX_ADDR_LEN];
	unsigned char dst_dev_addr[MAX_ADDR_LEN];
	unsigned char broadcast[MAX_ADDR_LEN];
	unsigned short dev_type;
	int bound_dev_if;
};

int rdma_translate_ip(struct sockaddr *addr, struct rdma_dev_addr *dev_addr);

int rdma_resolve_ip(struct rdma_addr_client *client,
		    struct sockaddr *src_addr, struct sockaddr *dst_addr,
		    struct rdma_dev_addr *addr, int timeout_ms,
		    void (*callback)(int status, struct sockaddr *src_addr,
				     struct rdma_dev_addr *addr, void *context),
		    void *context);

void rdma_addr_cancel(struct rdma_dev_addr *addr);

int rdma_copy_addr(struct rdma_dev_addr *dev_addr, struct net_device *dev,
	      const unsigned char *dst_dev_addr);

static inline int ip_addr_size(struct sockaddr *addr)
{
	return addr->sa_family == AF_INET6 ?
	       sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
}

static inline u16 ib_addr_get_pkey(struct rdma_dev_addr *dev_addr)
{
	return ((u16)dev_addr->broadcast[8] << 8) | (u16)dev_addr->broadcast[9];
}

static inline void ib_addr_set_pkey(struct rdma_dev_addr *dev_addr, u16 pkey)
{
	dev_addr->broadcast[8] = pkey >> 8;
	dev_addr->broadcast[9] = (unsigned char) pkey;
}

static inline void ib_addr_get_mgid(struct rdma_dev_addr *dev_addr,
				    union ib_gid *gid)
{
	memcpy(gid, dev_addr->broadcast + 4, sizeof *gid);
}

static inline int rdma_addr_gid_offset(struct rdma_dev_addr *dev_addr)
{
	return dev_addr->dev_type == ARPHRD_INFINIBAND ? 4 : 0;
}

static inline void rdma_addr_get_sgid(struct rdma_dev_addr *dev_addr, union ib_gid *gid)
{
	memcpy(gid, dev_addr->src_dev_addr + rdma_addr_gid_offset(dev_addr), sizeof *gid);
}

static inline void rdma_addr_set_sgid(struct rdma_dev_addr *dev_addr, union ib_gid *gid)
{
	memcpy(dev_addr->src_dev_addr + rdma_addr_gid_offset(dev_addr), gid, sizeof *gid);
}

static inline void rdma_addr_get_dgid(struct rdma_dev_addr *dev_addr, union ib_gid *gid)
{
	memcpy(gid, dev_addr->dst_dev_addr + rdma_addr_gid_offset(dev_addr), sizeof *gid);
}

static inline void rdma_addr_set_dgid(struct rdma_dev_addr *dev_addr, union ib_gid *gid)
{
	memcpy(dev_addr->dst_dev_addr + rdma_addr_gid_offset(dev_addr), gid, sizeof *gid);
}

#endif /* IB_ADDR_H */
