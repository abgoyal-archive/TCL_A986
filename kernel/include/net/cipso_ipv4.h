


#ifndef _CIPSO_IPV4_H
#define _CIPSO_IPV4_H

#include <linux/types.h>
#include <linux/rcupdate.h>
#include <linux/list.h>
#include <linux/net.h>
#include <linux/skbuff.h>
#include <net/netlabel.h>
#include <net/request_sock.h>
#include <asm/atomic.h>

/* known doi values */
#define CIPSO_V4_DOI_UNKNOWN          0x00000000

/* standard tag types */
#define CIPSO_V4_TAG_INVALID          0
#define CIPSO_V4_TAG_RBITMAP          1
#define CIPSO_V4_TAG_ENUM             2
#define CIPSO_V4_TAG_RANGE            5
#define CIPSO_V4_TAG_PBITMAP          6
#define CIPSO_V4_TAG_FREEFORM         7

/* non-standard tag types (tags > 127) */
#define CIPSO_V4_TAG_LOCAL            128

/* doi mapping types */
#define CIPSO_V4_MAP_UNKNOWN          0
#define CIPSO_V4_MAP_TRANS            1
#define CIPSO_V4_MAP_PASS             2
#define CIPSO_V4_MAP_LOCAL            3

/* limits */
#define CIPSO_V4_MAX_REM_LVLS         255
#define CIPSO_V4_INV_LVL              0x80000000
#define CIPSO_V4_MAX_LOC_LVLS         (CIPSO_V4_INV_LVL - 1)
#define CIPSO_V4_MAX_REM_CATS         65534
#define CIPSO_V4_INV_CAT              0x80000000
#define CIPSO_V4_MAX_LOC_CATS         (CIPSO_V4_INV_CAT - 1)


/* DOI definition struct */
#define CIPSO_V4_TAG_MAXCNT           5
struct cipso_v4_doi {
	u32 doi;
	u32 type;
	union {
		struct cipso_v4_std_map_tbl *std;
	} map;
	u8 tags[CIPSO_V4_TAG_MAXCNT];

	atomic_t refcount;
	struct list_head list;
	struct rcu_head rcu;
};

/* Standard CIPSO mapping table */
struct cipso_v4_std_map_tbl {
	struct {
		u32 *cipso;
		u32 *local;
		u32 cipso_size;
		u32 local_size;
	} lvl;
	struct {
		u32 *cipso;
		u32 *local;
		u32 cipso_size;
		u32 local_size;
	} cat;
};


#ifdef CONFIG_NETLABEL
extern int cipso_v4_cache_enabled;
extern int cipso_v4_cache_bucketsize;
extern int cipso_v4_rbm_optfmt;
extern int cipso_v4_rbm_strictvalid;
#endif


#define CIPSO_V4_OPTEXIST(x) (IPCB(x)->opt.cipso != 0)
#define CIPSO_V4_OPTPTR(x) (skb_network_header(x) + IPCB(x)->opt.cipso)


#ifdef CONFIG_NETLABEL
int cipso_v4_doi_add(struct cipso_v4_doi *doi_def,
		     struct netlbl_audit *audit_info);
void cipso_v4_doi_free(struct cipso_v4_doi *doi_def);
int cipso_v4_doi_remove(u32 doi, struct netlbl_audit *audit_info);
struct cipso_v4_doi *cipso_v4_doi_getdef(u32 doi);
void cipso_v4_doi_putdef(struct cipso_v4_doi *doi_def);
int cipso_v4_doi_walk(u32 *skip_cnt,
		     int (*callback) (struct cipso_v4_doi *doi_def, void *arg),
	             void *cb_arg);
#else
static inline int cipso_v4_doi_add(struct cipso_v4_doi *doi_def,
				   struct netlbl_audit *audit_info)
{
	return -ENOSYS;
}

static inline void cipso_v4_doi_free(struct cipso_v4_doi *doi_def)
{
	return;
}

static inline int cipso_v4_doi_remove(u32 doi,
				      struct netlbl_audit *audit_info)
{
	return 0;
}

static inline struct cipso_v4_doi *cipso_v4_doi_getdef(u32 doi)
{
	return NULL;
}

static inline int cipso_v4_doi_walk(u32 *skip_cnt,
		     int (*callback) (struct cipso_v4_doi *doi_def, void *arg),
		     void *cb_arg)
{
	return 0;
}

static inline int cipso_v4_doi_domhsh_add(struct cipso_v4_doi *doi_def,
					  const char *domain)
{
	return -ENOSYS;
}

static inline int cipso_v4_doi_domhsh_remove(struct cipso_v4_doi *doi_def,
					     const char *domain)
{
	return 0;
}
#endif /* CONFIG_NETLABEL */


#ifdef CONFIG_NETLABEL
void cipso_v4_cache_invalidate(void);
int cipso_v4_cache_add(const struct sk_buff *skb,
		       const struct netlbl_lsm_secattr *secattr);
#else
static inline void cipso_v4_cache_invalidate(void)
{
	return;
}

static inline int cipso_v4_cache_add(const struct sk_buff *skb,
				     const struct netlbl_lsm_secattr *secattr)
{
	return 0;
}
#endif /* CONFIG_NETLABEL */


#ifdef CONFIG_NETLABEL
void cipso_v4_error(struct sk_buff *skb, int error, u32 gateway);
int cipso_v4_sock_setattr(struct sock *sk,
			  const struct cipso_v4_doi *doi_def,
			  const struct netlbl_lsm_secattr *secattr);
void cipso_v4_sock_delattr(struct sock *sk);
int cipso_v4_sock_getattr(struct sock *sk, struct netlbl_lsm_secattr *secattr);
int cipso_v4_req_setattr(struct request_sock *req,
			 const struct cipso_v4_doi *doi_def,
			 const struct netlbl_lsm_secattr *secattr);
void cipso_v4_req_delattr(struct request_sock *req);
int cipso_v4_skbuff_setattr(struct sk_buff *skb,
			    const struct cipso_v4_doi *doi_def,
			    const struct netlbl_lsm_secattr *secattr);
int cipso_v4_skbuff_delattr(struct sk_buff *skb);
int cipso_v4_skbuff_getattr(const struct sk_buff *skb,
			    struct netlbl_lsm_secattr *secattr);
int cipso_v4_validate(const struct sk_buff *skb, unsigned char **option);
#else
static inline void cipso_v4_error(struct sk_buff *skb,
				  int error,
				  u32 gateway)
{
	return;
}

static inline int cipso_v4_sock_setattr(struct sock *sk,
				      const struct cipso_v4_doi *doi_def,
				      const struct netlbl_lsm_secattr *secattr)
{
	return -ENOSYS;
}

static inline void cipso_v4_sock_delattr(struct sock *sk)
{
}

static inline int cipso_v4_sock_getattr(struct sock *sk,
					struct netlbl_lsm_secattr *secattr)
{
	return -ENOSYS;
}

static inline int cipso_v4_req_setattr(struct request_sock *req,
				       const struct cipso_v4_doi *doi_def,
				       const struct netlbl_lsm_secattr *secattr)
{
	return -ENOSYS;
}

static inline void cipso_v4_req_delattr(struct request_sock *req)
{
	return;
}

static inline int cipso_v4_skbuff_setattr(struct sk_buff *skb,
				      const struct cipso_v4_doi *doi_def,
				      const struct netlbl_lsm_secattr *secattr)
{
	return -ENOSYS;
}

static inline int cipso_v4_skbuff_delattr(struct sk_buff *skb)
{
	return -ENOSYS;
}

static inline int cipso_v4_skbuff_getattr(const struct sk_buff *skb,
					  struct netlbl_lsm_secattr *secattr)
{
	return -ENOSYS;
}

static inline int cipso_v4_validate(const struct sk_buff *skb,
				    unsigned char **option)
{
	return -ENOSYS;
}
#endif /* CONFIG_NETLABEL */

#endif /* _CIPSO_IPV4_H */
