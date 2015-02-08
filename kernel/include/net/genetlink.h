
#ifndef __NET_GENERIC_NETLINK_H
#define __NET_GENERIC_NETLINK_H

#include <linux/genetlink.h>
#include <net/netlink.h>
#include <net/net_namespace.h>

struct genl_multicast_group {
	struct genl_family	*family;	/* private */
	struct list_head	list;		/* private */
	char			name[GENL_NAMSIZ];
	u32			id;
};

struct genl_family {
	unsigned int		id;
	unsigned int		hdrsize;
	char			name[GENL_NAMSIZ];
	unsigned int		version;
	unsigned int		maxattr;
	bool			netnsok;
	struct nlattr **	attrbuf;	/* private */
	struct list_head	ops_list;	/* private */
	struct list_head	family_list;	/* private */
	struct list_head	mcast_groups;	/* private */
};

struct genl_info {
	u32			snd_seq;
	u32			snd_pid;
	struct nlmsghdr *	nlhdr;
	struct genlmsghdr *	genlhdr;
	void *			userhdr;
	struct nlattr **	attrs;
#ifdef CONFIG_NET_NS
	struct net *		_net;
#endif
};

#ifdef CONFIG_NET_NS
static inline struct net *genl_info_net(struct genl_info *info)
{
	return info->_net;
}

static inline void genl_info_net_set(struct genl_info *info, struct net *net)
{
	info->_net = net;
}
#else
static inline struct net *genl_info_net(struct genl_info *info)
{
	return &init_net;
}

static inline void genl_info_net_set(struct genl_info *info, struct net *net)
{
}
#endif

struct genl_ops {
	u8			cmd;
	unsigned int		flags;
	const struct nla_policy	*policy;
	int		       (*doit)(struct sk_buff *skb,
				       struct genl_info *info);
	int		       (*dumpit)(struct sk_buff *skb,
					 struct netlink_callback *cb);
	int		       (*done)(struct netlink_callback *cb);
	struct list_head	ops_list;
};

extern int genl_register_family(struct genl_family *family);
extern int genl_register_family_with_ops(struct genl_family *family,
	struct genl_ops *ops, size_t n_ops);
extern int genl_unregister_family(struct genl_family *family);
extern int genl_register_ops(struct genl_family *, struct genl_ops *ops);
extern int genl_unregister_ops(struct genl_family *, struct genl_ops *ops);
extern int genl_register_mc_group(struct genl_family *family,
				  struct genl_multicast_group *grp);
extern void genl_unregister_mc_group(struct genl_family *family,
				     struct genl_multicast_group *grp);

static inline void *genlmsg_put(struct sk_buff *skb, u32 pid, u32 seq,
				struct genl_family *family, int flags, u8 cmd)
{
	struct nlmsghdr *nlh;
	struct genlmsghdr *hdr;

	nlh = nlmsg_put(skb, pid, seq, family->id, GENL_HDRLEN +
			family->hdrsize, flags);
	if (nlh == NULL)
		return NULL;

	hdr = nlmsg_data(nlh);
	hdr->cmd = cmd;
	hdr->version = family->version;
	hdr->reserved = 0;

	return (char *) hdr + GENL_HDRLEN;
}

static inline void *genlmsg_put_reply(struct sk_buff *skb,
				      struct genl_info *info,
				      struct genl_family *family,
				      int flags, u8 cmd)
{
	return genlmsg_put(skb, info->snd_pid, info->snd_seq, family,
			   flags, cmd);
}

static inline int genlmsg_end(struct sk_buff *skb, void *hdr)
{
	return nlmsg_end(skb, hdr - GENL_HDRLEN - NLMSG_HDRLEN);
}

static inline void genlmsg_cancel(struct sk_buff *skb, void *hdr)
{
	nlmsg_cancel(skb, hdr - GENL_HDRLEN - NLMSG_HDRLEN);
}

static inline int genlmsg_multicast_netns(struct net *net, struct sk_buff *skb,
					  u32 pid, unsigned int group, gfp_t flags)
{
	return nlmsg_multicast(net->genl_sock, skb, pid, group, flags);
}

static inline int genlmsg_multicast(struct sk_buff *skb, u32 pid,
				    unsigned int group, gfp_t flags)
{
	return genlmsg_multicast_netns(&init_net, skb, pid, group, flags);
}

int genlmsg_multicast_allns(struct sk_buff *skb, u32 pid,
			    unsigned int group, gfp_t flags);

static inline int genlmsg_unicast(struct net *net, struct sk_buff *skb, u32 pid)
{
	return nlmsg_unicast(net->genl_sock, skb, pid);
}

static inline int genlmsg_reply(struct sk_buff *skb, struct genl_info *info)
{
	return genlmsg_unicast(genl_info_net(info), skb, info->snd_pid);
}

static inline void *genlmsg_data(const struct genlmsghdr *gnlh)
{
	return ((unsigned char *) gnlh + GENL_HDRLEN);
}

static inline int genlmsg_len(const struct genlmsghdr *gnlh)
{
	struct nlmsghdr *nlh = (struct nlmsghdr *)((unsigned char *)gnlh -
							NLMSG_HDRLEN);
	return (nlh->nlmsg_len - GENL_HDRLEN - NLMSG_HDRLEN);
}

static inline int genlmsg_msg_size(int payload)
{
	return GENL_HDRLEN + payload;
}

static inline int genlmsg_total_size(int payload)
{
	return NLMSG_ALIGN(genlmsg_msg_size(payload));
}

static inline struct sk_buff *genlmsg_new(size_t payload, gfp_t flags)
{
	return nlmsg_new(genlmsg_total_size(payload), flags);
}


#endif	/* __NET_GENERIC_NETLINK_H */
