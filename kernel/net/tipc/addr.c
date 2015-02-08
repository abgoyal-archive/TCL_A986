

#include "core.h"
#include "dbg.h"
#include "addr.h"
#include "zone.h"
#include "cluster.h"
#include "net.h"

u32 tipc_get_addr(void)
{
	return tipc_own_addr;
}


int tipc_addr_domain_valid(u32 addr)
{
	u32 n = tipc_node(addr);
	u32 c = tipc_cluster(addr);
	u32 z = tipc_zone(addr);
	u32 max_nodes = tipc_max_nodes;

	if (is_slave(addr))
		max_nodes = LOWEST_SLAVE + tipc_max_slaves;
	if (n > max_nodes)
		return 0;
	if (c > tipc_max_clusters)
		return 0;
	if (z > tipc_max_zones)
		return 0;

	if (n && (!z || !c))
		return 0;
	if (c && !z)
		return 0;
	return 1;
}


int tipc_addr_node_valid(u32 addr)
{
	return (tipc_addr_domain_valid(addr) && tipc_node(addr));
}

int tipc_in_scope(u32 domain, u32 addr)
{
	if (!domain || (domain == addr))
		return 1;
	if (domain == (addr & 0xfffff000u)) /* domain <Z.C.0> */
		return 1;
	if (domain == (addr & 0xff000000u)) /* domain <Z.0.0> */
		return 1;
	return 0;
}


int tipc_addr_scope(u32 domain)
{
	if (likely(!domain))
		return TIPC_ZONE_SCOPE;
	if (tipc_node(domain))
		return TIPC_NODE_SCOPE;
	if (tipc_cluster(domain))
		return TIPC_CLUSTER_SCOPE;
	return TIPC_ZONE_SCOPE;
}

char *tipc_addr_string_fill(char *string, u32 addr)
{
	snprintf(string, 16, "<%u.%u.%u>",
		 tipc_zone(addr), tipc_cluster(addr), tipc_node(addr));
	return string;
}
