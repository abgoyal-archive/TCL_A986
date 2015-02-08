

#include <linux/kernel.h>
#include <net/netlink.h>
#include <linux/nl802154.h>

#define NLA_HW_ADDR NLA_U64

const struct nla_policy ieee802154_policy[IEEE802154_ATTR_MAX + 1] = {
	[IEEE802154_ATTR_DEV_NAME] = { .type = NLA_STRING, },
	[IEEE802154_ATTR_DEV_INDEX] = { .type = NLA_U32, },
	[IEEE802154_ATTR_PHY_NAME] = { .type = NLA_STRING, },

	[IEEE802154_ATTR_STATUS] = { .type = NLA_U8, },
	[IEEE802154_ATTR_SHORT_ADDR] = { .type = NLA_U16, },
	[IEEE802154_ATTR_HW_ADDR] = { .type = NLA_HW_ADDR, },
	[IEEE802154_ATTR_PAN_ID] = { .type = NLA_U16, },
	[IEEE802154_ATTR_CHANNEL] = { .type = NLA_U8, },
	[IEEE802154_ATTR_PAGE] = { .type = NLA_U8, },
	[IEEE802154_ATTR_COORD_SHORT_ADDR] = { .type = NLA_U16, },
	[IEEE802154_ATTR_COORD_HW_ADDR] = { .type = NLA_HW_ADDR, },
	[IEEE802154_ATTR_COORD_PAN_ID] = { .type = NLA_U16, },
	[IEEE802154_ATTR_SRC_SHORT_ADDR] = { .type = NLA_U16, },
	[IEEE802154_ATTR_SRC_HW_ADDR] = { .type = NLA_HW_ADDR, },
	[IEEE802154_ATTR_SRC_PAN_ID] = { .type = NLA_U16, },
	[IEEE802154_ATTR_DEST_SHORT_ADDR] = { .type = NLA_U16, },
	[IEEE802154_ATTR_DEST_HW_ADDR] = { .type = NLA_HW_ADDR, },
	[IEEE802154_ATTR_DEST_PAN_ID] = { .type = NLA_U16, },

	[IEEE802154_ATTR_CAPABILITY] = { .type = NLA_U8, },
	[IEEE802154_ATTR_REASON] = { .type = NLA_U8, },
	[IEEE802154_ATTR_SCAN_TYPE] = { .type = NLA_U8, },
	[IEEE802154_ATTR_CHANNELS] = { .type = NLA_U32, },
	[IEEE802154_ATTR_DURATION] = { .type = NLA_U8, },
	[IEEE802154_ATTR_ED_LIST] = { .len = 27 },
	[IEEE802154_ATTR_CHANNEL_PAGE_LIST] = { .len = 32 * 4, },
};

