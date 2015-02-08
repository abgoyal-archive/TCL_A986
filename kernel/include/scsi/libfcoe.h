

#ifndef _LIBFCOE_H
#define _LIBFCOE_H

#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>
#include <scsi/fc/fc_fcoe.h>
#include <scsi/libfc.h>

#define FCOE_MAX_CMD_LEN	16	/* Supported CDB length */

#define FCOE_CTLR_START_DELAY	2000	/* mS after first adv. to choose FCF */
#define FCOE_CTRL_SOL_TOV	2000	/* min. solicitation interval (mS) */
#define FCOE_CTLR_FCF_LIMIT	20	/* max. number of FCF entries */

enum fip_state {
	FIP_ST_DISABLED,
	FIP_ST_LINK_WAIT,
	FIP_ST_AUTO,
	FIP_ST_NON_FIP,
	FIP_ST_ENABLED,
};

struct fcoe_ctlr {
	enum fip_state state;
	enum fip_state mode;
	struct fc_lport *lp;
	struct fcoe_fcf *sel_fcf;
	struct list_head fcfs;
	u16 fcf_count;
	unsigned long sol_time;
	unsigned long sel_time;
	unsigned long port_ka_time;
	unsigned long ctlr_ka_time;
	struct timer_list timer;
	struct work_struct timer_work;
	struct work_struct recv_work;
	struct sk_buff_head fip_recv_list;
	u16 user_mfs;
	u16 flogi_oxid;
	u8 flogi_count;
	u8 reset_req;
	u8 map_dest;
	u8 spma;
	u8 send_ctlr_ka;
	u8 send_port_ka;
	u8 dest_addr[ETH_ALEN];
	u8 ctl_src_addr[ETH_ALEN];

	void (*send)(struct fcoe_ctlr *, struct sk_buff *);
	void (*update_mac)(struct fc_lport *, u8 *addr);
	u8 * (*get_src_addr)(struct fc_lport *);
	spinlock_t lock;
};

struct fcoe_fcf {
	struct list_head list;
	unsigned long time;

	u64 switch_name;
	u64 fabric_name;
	u32 fc_map;
	u16 vfid;
	u8 fcf_mac[ETH_ALEN];

	u8 pri;
	u16 flags;
	u32 fka_period;
	u8 fd_flags:1;
};

/* FIP API functions */
void fcoe_ctlr_init(struct fcoe_ctlr *);
void fcoe_ctlr_destroy(struct fcoe_ctlr *);
void fcoe_ctlr_link_up(struct fcoe_ctlr *);
int fcoe_ctlr_link_down(struct fcoe_ctlr *);
int fcoe_ctlr_els_send(struct fcoe_ctlr *, struct fc_lport *, struct sk_buff *);
void fcoe_ctlr_recv(struct fcoe_ctlr *, struct sk_buff *);
int fcoe_ctlr_recv_flogi(struct fcoe_ctlr *, struct fc_lport *,
			 struct fc_frame *);

/* libfcoe funcs */
u64 fcoe_wwn_from_mac(unsigned char mac[], unsigned int, unsigned int);
int fcoe_libfc_config(struct fc_lport *, struct libfc_function_template *);

#endif /* _LIBFCOE_H */
