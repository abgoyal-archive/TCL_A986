

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/ipv6.h>
#include <linux/skbuff.h>
#include <net/act_api.h>
#include <net/pkt_cls.h>
#include <net/netlink.h>

#define RSVP_DST_LEN	4
#define RSVP_ID		"rsvp6"
#define RSVP_OPS	cls_rsvp6_ops

#include "cls_rsvp.h"
MODULE_LICENSE("GPL");
