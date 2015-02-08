

#ifndef _TIPC_CONFIG_H
#define _TIPC_CONFIG_H

/* ---------------------------------------------------------------------- */

#include "core.h"
#include "link.h"

struct sk_buff *tipc_cfg_reply_alloc(int payload_size);
int tipc_cfg_append_tlv(struct sk_buff *buf, int tlv_type,
			void *tlv_data, int tlv_data_size);
struct sk_buff *tipc_cfg_reply_unsigned_type(u16 tlv_type, u32 value);
struct sk_buff *tipc_cfg_reply_string_type(u16 tlv_type, char *string);

static inline struct sk_buff *tipc_cfg_reply_none(void)
{
	return tipc_cfg_reply_alloc(0);
}

static inline struct sk_buff *tipc_cfg_reply_unsigned(u32 value)
{
	return tipc_cfg_reply_unsigned_type(TIPC_TLV_UNSIGNED, value);
}

static inline struct sk_buff *tipc_cfg_reply_error_string(char *string)
{
	return tipc_cfg_reply_string_type(TIPC_TLV_ERROR_STRING, string);
}

static inline struct sk_buff *tipc_cfg_reply_ultra_string(char *string)
{
	return tipc_cfg_reply_string_type(TIPC_TLV_ULTRA_STRING, string);
}

struct sk_buff *tipc_cfg_do_cmd(u32 orig_node, u16 cmd,
				const void *req_tlv_area, int req_tlv_space,
				int headroom);

void tipc_cfg_link_event(u32 addr, char *name, int up);
int  tipc_cfg_init(void);
void tipc_cfg_stop(void);

#endif
