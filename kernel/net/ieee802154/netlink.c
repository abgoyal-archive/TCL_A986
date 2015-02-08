

#include <linux/kernel.h>
#include <linux/gfp.h>
#include <net/genetlink.h>
#include <linux/nl802154.h>

#include "ieee802154.h"

static unsigned int ieee802154_seq_num;
static DEFINE_SPINLOCK(ieee802154_seq_lock);

struct genl_family nl802154_family = {
	.id		= GENL_ID_GENERATE,
	.hdrsize	= 0,
	.name		= IEEE802154_NL_NAME,
	.version	= 1,
	.maxattr	= IEEE802154_ATTR_MAX,
};

/* Requests to userspace */
struct sk_buff *ieee802154_nl_create(int flags, u8 req)
{
	void *hdr;
	struct sk_buff *msg = nlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);
	unsigned long f;

	if (!msg)
		return NULL;

	spin_lock_irqsave(&ieee802154_seq_lock, f);
	hdr = genlmsg_put(msg, 0, ieee802154_seq_num++,
			&nl802154_family, flags, req);
	spin_unlock_irqrestore(&ieee802154_seq_lock, f);
	if (!hdr) {
		nlmsg_free(msg);
		return NULL;
	}

	return msg;
}

int ieee802154_nl_mcast(struct sk_buff *msg, unsigned int group)
{
	/* XXX: nlh is right at the start of msg */
	void *hdr = genlmsg_data(NLMSG_DATA(msg->data));

	if (genlmsg_end(msg, hdr) < 0)
		goto out;

	return genlmsg_multicast(msg, 0, group, GFP_ATOMIC);
out:
	nlmsg_free(msg);
	return -ENOBUFS;
}

struct sk_buff *ieee802154_nl_new_reply(struct genl_info *info,
		int flags, u8 req)
{
	void *hdr;
	struct sk_buff *msg = nlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);

	if (!msg)
		return NULL;

	hdr = genlmsg_put_reply(msg, info,
			&nl802154_family, flags, req);
	if (!hdr) {
		nlmsg_free(msg);
		return NULL;
	}

	return msg;
}

int ieee802154_nl_reply(struct sk_buff *msg, struct genl_info *info)
{
	/* XXX: nlh is right at the start of msg */
	void *hdr = genlmsg_data(NLMSG_DATA(msg->data));

	if (genlmsg_end(msg, hdr) < 0)
		goto out;

	return genlmsg_reply(msg, info);
out:
	nlmsg_free(msg);
	return -ENOBUFS;
}

int __init ieee802154_nl_init(void)
{
	int rc;

	rc = genl_register_family(&nl802154_family);
	if (rc)
		goto fail;

	rc = nl802154_mac_register();
	if (rc)
		goto fail;

	rc = nl802154_phy_register();
	if (rc)
		goto fail;

	return 0;

fail:
	genl_unregister_family(&nl802154_family);
	return rc;
}

void __exit ieee802154_nl_exit(void)
{
	genl_unregister_family(&nl802154_family);
}

