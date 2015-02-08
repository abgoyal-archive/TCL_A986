

#ifndef _TIPC_ADDR_H
#define _TIPC_ADDR_H

static inline u32 own_node(void)
{
	return tipc_node(tipc_own_addr);
}

static inline u32 own_cluster(void)
{
	return tipc_cluster(tipc_own_addr);
}

static inline u32 own_zone(void)
{
	return tipc_zone(tipc_own_addr);
}

static inline int in_own_cluster(u32 addr)
{
	return !((addr ^ tipc_own_addr) >> 12);
}

static inline int is_slave(u32 addr)
{
	return addr & 0x800;
}

static inline int may_route(u32 addr)
{
	return(addr ^ tipc_own_addr) >> 11;
}


static inline int addr_domain(int sc)
{
	if (likely(sc == TIPC_NODE_SCOPE))
		return tipc_own_addr;
	if (sc == TIPC_CLUSTER_SCOPE)
		return tipc_addr(tipc_zone(tipc_own_addr),
				 tipc_cluster(tipc_own_addr), 0);
	return tipc_addr(tipc_zone(tipc_own_addr), 0, 0);
}

int tipc_addr_domain_valid(u32);
int tipc_addr_node_valid(u32 addr);
int tipc_in_scope(u32 domain, u32 addr);
int tipc_addr_scope(u32 domain);
char *tipc_addr_string_fill(char *string, u32 addr);
#endif
