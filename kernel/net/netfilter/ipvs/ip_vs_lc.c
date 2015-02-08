

#define KMSG_COMPONENT "IPVS"
#define pr_fmt(fmt) KMSG_COMPONENT ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>

#include <net/ip_vs.h>


static inline unsigned int
ip_vs_lc_dest_overhead(struct ip_vs_dest *dest)
{
	/*
	 * We think the overhead of processing active connections is 256
	 * times higher than that of inactive connections in average. (This
	 * 256 times might not be accurate, we will change it later) We
	 * use the following formula to estimate the overhead now:
	 *		  dest->activeconns*256 + dest->inactconns
	 */
	return (atomic_read(&dest->activeconns) << 8) +
		atomic_read(&dest->inactconns);
}


static struct ip_vs_dest *
ip_vs_lc_schedule(struct ip_vs_service *svc, const struct sk_buff *skb)
{
	struct ip_vs_dest *dest, *least = NULL;
	unsigned int loh = 0, doh;

	IP_VS_DBG(6, "%s(): Scheduling...\n", __func__);

	/*
	 * Simply select the server with the least number of
	 *        (activeconns<<5) + inactconns
	 * Except whose weight is equal to zero.
	 * If the weight is equal to zero, it means that the server is
	 * quiesced, the existing connections to the server still get
	 * served, but no new connection is assigned to the server.
	 */

	list_for_each_entry(dest, &svc->destinations, n_list) {
		if ((dest->flags & IP_VS_DEST_F_OVERLOAD) ||
		    atomic_read(&dest->weight) == 0)
			continue;
		doh = ip_vs_lc_dest_overhead(dest);
		if (!least || doh < loh) {
			least = dest;
			loh = doh;
		}
	}

	if (!least)
		IP_VS_ERR_RL("LC: no destination available\n");
	else
		IP_VS_DBG_BUF(6, "LC: server %s:%u activeconns %d "
			      "inactconns %d\n",
			      IP_VS_DBG_ADDR(svc->af, &least->addr),
			      ntohs(least->port),
			      atomic_read(&least->activeconns),
			      atomic_read(&least->inactconns));

	return least;
}


static struct ip_vs_scheduler ip_vs_lc_scheduler = {
	.name =			"lc",
	.refcnt =		ATOMIC_INIT(0),
	.module =		THIS_MODULE,
	.n_list =		LIST_HEAD_INIT(ip_vs_lc_scheduler.n_list),
	.schedule =		ip_vs_lc_schedule,
};


static int __init ip_vs_lc_init(void)
{
	return register_ip_vs_scheduler(&ip_vs_lc_scheduler) ;
}

static void __exit ip_vs_lc_cleanup(void)
{
	unregister_ip_vs_scheduler(&ip_vs_lc_scheduler);
}

module_init(ip_vs_lc_init);
module_exit(ip_vs_lc_cleanup);
MODULE_LICENSE("GPL");
