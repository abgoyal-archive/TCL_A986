
#ifndef _TRACE_NET_H
#define _TRACE_NET_H

#include <linux/tracepoint.h>

struct sk_buff;
DECLARE_TRACE(net_dev_xmit,
	TP_PROTO(struct sk_buff *skb),
	TP_ARGS(skb));
DECLARE_TRACE(net_dev_receive,
	TP_PROTO(struct sk_buff *skb),
	TP_ARGS(skb));
DECLARE_TRACE(net_tcpv4_rcv,
	TP_PROTO(struct sk_buff *skb),
	TP_ARGS(skb));
DECLARE_TRACE(net_udpv4_rcv,
	TP_PROTO(struct sk_buff *skb),
	TP_ARGS(skb));

struct napi_struct;
DECLARE_TRACE(net_napi_schedule,
	TP_PROTO(struct napi_struct *n),
	TP_ARGS(n));
DECLARE_TRACE(net_napi_poll,
	TP_PROTO(struct napi_struct *n),
	TP_ARGS(n));
DECLARE_TRACE(net_napi_complete,
	TP_PROTO(struct napi_struct *n),
	TP_ARGS(n));

#endif
