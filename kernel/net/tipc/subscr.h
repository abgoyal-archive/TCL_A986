

#ifndef _TIPC_SUBSCR_H
#define _TIPC_SUBSCR_H

struct subscription;

typedef void (*tipc_subscr_event) (struct subscription *sub,
				   u32 found_lower, u32 found_upper,
				   u32 event, u32 port_ref, u32 node);


struct subscription {
	struct tipc_name_seq seq;
	u32 timeout;
	u32 filter;
	tipc_subscr_event event_cb;
	struct timer_list timer;
	struct list_head nameseq_list;
	struct list_head subscription_list;
	u32 server_ref;
	struct tipc_event evt;
};

int tipc_subscr_overlap(struct subscription *sub,
			u32 found_lower,
			u32 found_upper);

void tipc_subscr_report_overlap(struct subscription *sub,
				u32 found_lower,
				u32 found_upper,
				u32 event,
				u32 port_ref,
				u32 node,
				int must_report);

int tipc_subscr_start(void);

void tipc_subscr_stop(void);


#endif
