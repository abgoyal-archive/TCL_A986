
#ifndef _CCID_H
#define _CCID_H

#include <net/sock.h>
#include <linux/compiler.h>
#include <linux/dccp.h>
#include <linux/list.h>
#include <linux/module.h>

/* maximum value for a CCID (RFC 4340, 19.5) */
#define CCID_MAX		255
#define CCID_SLAB_NAME_LENGTH	32

struct tcp_info;

struct ccid_operations {
	unsigned char		ccid_id;
	__u32			ccid_ccmps;
	const char		*ccid_name;
	struct kmem_cache	*ccid_hc_rx_slab,
				*ccid_hc_tx_slab;
	char			ccid_hc_rx_slab_name[CCID_SLAB_NAME_LENGTH];
	char			ccid_hc_tx_slab_name[CCID_SLAB_NAME_LENGTH];
	__u32			ccid_hc_rx_obj_size,
				ccid_hc_tx_obj_size;
	/* Interface Routines */
	int		(*ccid_hc_rx_init)(struct ccid *ccid, struct sock *sk);
	int		(*ccid_hc_tx_init)(struct ccid *ccid, struct sock *sk);
	void		(*ccid_hc_rx_exit)(struct sock *sk);
	void		(*ccid_hc_tx_exit)(struct sock *sk);
	void		(*ccid_hc_rx_packet_recv)(struct sock *sk,
						  struct sk_buff *skb);
	int		(*ccid_hc_rx_parse_options)(struct sock *sk,
						    unsigned char option,
						    unsigned char len, u16 idx,
						    unsigned char* value);
	int		(*ccid_hc_rx_insert_options)(struct sock *sk,
						     struct sk_buff *skb);
	void		(*ccid_hc_tx_packet_recv)(struct sock *sk,
						  struct sk_buff *skb);
	int		(*ccid_hc_tx_parse_options)(struct sock *sk,
						    unsigned char option,
						    unsigned char len, u16 idx,
						    unsigned char* value);
	int		(*ccid_hc_tx_send_packet)(struct sock *sk,
						  struct sk_buff *skb);
	void		(*ccid_hc_tx_packet_sent)(struct sock *sk,
						  int more, unsigned int len);
	void		(*ccid_hc_rx_get_info)(struct sock *sk,
					       struct tcp_info *info);
	void		(*ccid_hc_tx_get_info)(struct sock *sk,
					       struct tcp_info *info);
	int		(*ccid_hc_rx_getsockopt)(struct sock *sk,
						 const int optname, int len,
						 u32 __user *optval,
						 int __user *optlen);
	int		(*ccid_hc_tx_getsockopt)(struct sock *sk,
						 const int optname, int len,
						 u32 __user *optval,
						 int __user *optlen);
};

extern struct ccid_operations ccid2_ops;
#ifdef CONFIG_IP_DCCP_CCID3
extern struct ccid_operations ccid3_ops;
#endif

extern int  ccid_initialize_builtins(void);
extern void ccid_cleanup_builtins(void);

struct ccid {
	struct ccid_operations *ccid_ops;
	char		       ccid_priv[0];
};

static inline void *ccid_priv(const struct ccid *ccid)
{
	return (void *)ccid->ccid_priv;
}

extern bool ccid_support_check(u8 const *ccid_array, u8 array_len);
extern int  ccid_get_builtin_ccids(u8 **ccid_array, u8 *array_len);
extern int  ccid_getsockopt_builtin_ccids(struct sock *sk, int len,
					  char __user *, int __user *);

extern struct ccid *ccid_new(const u8 id, struct sock *sk, bool rx);

static inline int ccid_get_current_rx_ccid(struct dccp_sock *dp)
{
	struct ccid *ccid = dp->dccps_hc_rx_ccid;

	if (ccid == NULL || ccid->ccid_ops == NULL)
		return -1;
	return ccid->ccid_ops->ccid_id;
}

static inline int ccid_get_current_tx_ccid(struct dccp_sock *dp)
{
	struct ccid *ccid = dp->dccps_hc_tx_ccid;

	if (ccid == NULL || ccid->ccid_ops == NULL)
		return -1;
	return ccid->ccid_ops->ccid_id;
}

extern void ccid_hc_rx_delete(struct ccid *ccid, struct sock *sk);
extern void ccid_hc_tx_delete(struct ccid *ccid, struct sock *sk);

static inline int ccid_hc_tx_send_packet(struct ccid *ccid, struct sock *sk,
					 struct sk_buff *skb)
{
	int rc = 0;
	if (ccid->ccid_ops->ccid_hc_tx_send_packet != NULL)
		rc = ccid->ccid_ops->ccid_hc_tx_send_packet(sk, skb);
	return rc;
}

static inline void ccid_hc_tx_packet_sent(struct ccid *ccid, struct sock *sk,
					  int more, unsigned int len)
{
	if (ccid->ccid_ops->ccid_hc_tx_packet_sent != NULL)
		ccid->ccid_ops->ccid_hc_tx_packet_sent(sk, more, len);
}

static inline void ccid_hc_rx_packet_recv(struct ccid *ccid, struct sock *sk,
					  struct sk_buff *skb)
{
	if (ccid->ccid_ops->ccid_hc_rx_packet_recv != NULL)
		ccid->ccid_ops->ccid_hc_rx_packet_recv(sk, skb);
}

static inline void ccid_hc_tx_packet_recv(struct ccid *ccid, struct sock *sk,
					  struct sk_buff *skb)
{
	if (ccid->ccid_ops->ccid_hc_tx_packet_recv != NULL)
		ccid->ccid_ops->ccid_hc_tx_packet_recv(sk, skb);
}

static inline int ccid_hc_tx_parse_options(struct ccid *ccid, struct sock *sk,
					   unsigned char option,
					   unsigned char len, u16 idx,
					   unsigned char* value)
{
	int rc = 0;
	if (ccid->ccid_ops->ccid_hc_tx_parse_options != NULL)
		rc = ccid->ccid_ops->ccid_hc_tx_parse_options(sk, option, len, idx,
						    value);
	return rc;
}

static inline int ccid_hc_rx_parse_options(struct ccid *ccid, struct sock *sk,
					   unsigned char option,
					   unsigned char len, u16 idx,
					   unsigned char* value)
{
	int rc = 0;
	if (ccid->ccid_ops->ccid_hc_rx_parse_options != NULL)
		rc = ccid->ccid_ops->ccid_hc_rx_parse_options(sk, option, len, idx, value);
	return rc;
}

static inline int ccid_hc_rx_insert_options(struct ccid *ccid, struct sock *sk,
					    struct sk_buff *skb)
{
	if (ccid->ccid_ops->ccid_hc_rx_insert_options != NULL)
		return ccid->ccid_ops->ccid_hc_rx_insert_options(sk, skb);
	return 0;
}

static inline void ccid_hc_rx_get_info(struct ccid *ccid, struct sock *sk,
				       struct tcp_info *info)
{
	if (ccid->ccid_ops->ccid_hc_rx_get_info != NULL)
		ccid->ccid_ops->ccid_hc_rx_get_info(sk, info);
}

static inline void ccid_hc_tx_get_info(struct ccid *ccid, struct sock *sk,
				       struct tcp_info *info)
{
	if (ccid->ccid_ops->ccid_hc_tx_get_info != NULL)
		ccid->ccid_ops->ccid_hc_tx_get_info(sk, info);
}

static inline int ccid_hc_rx_getsockopt(struct ccid *ccid, struct sock *sk,
					const int optname, int len,
					u32 __user *optval, int __user *optlen)
{
	int rc = -ENOPROTOOPT;
	if (ccid->ccid_ops->ccid_hc_rx_getsockopt != NULL)
		rc = ccid->ccid_ops->ccid_hc_rx_getsockopt(sk, optname, len,
						 optval, optlen);
	return rc;
}

static inline int ccid_hc_tx_getsockopt(struct ccid *ccid, struct sock *sk,
					const int optname, int len,
					u32 __user *optval, int __user *optlen)
{
	int rc = -ENOPROTOOPT;
	if (ccid->ccid_ops->ccid_hc_tx_getsockopt != NULL)
		rc = ccid->ccid_ops->ccid_hc_tx_getsockopt(sk, optname, len,
						 optval, optlen);
	return rc;
}
#endif /* _CCID_H */
