

#ifndef _NET_TIPC_MSG_H_
#define _NET_TIPC_MSG_H_

#ifdef __KERNEL__

struct tipc_msg {
	__be32 hdr[15];
};



#define TIPC_CONN_MSG	0
#define TIPC_MCAST_MSG	1
#define TIPC_NAMED_MSG	2
#define TIPC_DIRECT_MSG	3


static inline u32 msg_word(struct tipc_msg *m, u32 pos)
{
	return ntohl(m->hdr[pos]);
}

static inline u32 msg_bits(struct tipc_msg *m, u32 w, u32 pos, u32 mask)
{
	return (msg_word(m, w) >> pos) & mask;
}

static inline u32 msg_importance(struct tipc_msg *m)
{
	return msg_bits(m, 0, 25, 0xf);
}

static inline u32 msg_hdr_sz(struct tipc_msg *m)
{
	return msg_bits(m, 0, 21, 0xf) << 2;
}

static inline int msg_short(struct tipc_msg *m)
{
	return (msg_hdr_sz(m) == 24);
}

static inline u32 msg_size(struct tipc_msg *m)
{
	return msg_bits(m, 0, 0, 0x1ffff);
}

static inline u32 msg_data_sz(struct tipc_msg *m)
{
	return (msg_size(m) - msg_hdr_sz(m));
}

static inline unchar *msg_data(struct tipc_msg *m)
{
	return ((unchar *)m) + msg_hdr_sz(m);
}

static inline u32 msg_type(struct tipc_msg *m)
{
	return msg_bits(m, 1, 29, 0x7);
}

static inline u32 msg_named(struct tipc_msg *m)
{
	return (msg_type(m) == TIPC_NAMED_MSG);
}

static inline u32 msg_mcast(struct tipc_msg *m)
{
	return (msg_type(m) == TIPC_MCAST_MSG);
}

static inline u32 msg_connected(struct tipc_msg *m)
{
	return (msg_type(m) == TIPC_CONN_MSG);
}

static inline u32 msg_errcode(struct tipc_msg *m)
{
	return msg_bits(m, 1, 25, 0xf);
}

static inline u32 msg_prevnode(struct tipc_msg *m)
{
	return msg_word(m, 3);
}

static inline u32 msg_origport(struct tipc_msg *m)
{
	return msg_word(m, 4);
}

static inline u32 msg_destport(struct tipc_msg *m)
{
	return msg_word(m, 5);
}

static inline u32 msg_mc_netid(struct tipc_msg *m)
{
	return msg_word(m, 5);
}

static inline u32 msg_orignode(struct tipc_msg *m)
{
	if (likely(msg_short(m)))
		return msg_prevnode(m);
	return msg_word(m, 6);
}

static inline u32 msg_destnode(struct tipc_msg *m)
{
	return msg_word(m, 7);
}

static inline u32 msg_nametype(struct tipc_msg *m)
{
	return msg_word(m, 8);
}

static inline u32 msg_nameinst(struct tipc_msg *m)
{
	return msg_word(m, 9);
}

static inline u32 msg_namelower(struct tipc_msg *m)
{
	return msg_nameinst(m);
}

static inline u32 msg_nameupper(struct tipc_msg *m)
{
	return msg_word(m, 10);
}

#endif

#endif
