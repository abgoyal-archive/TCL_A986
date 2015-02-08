


#define KMSG_COMPONENT "IPVS"
#define pr_fmt(fmt) KMSG_COMPONENT ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>

#include <net/ip_vs.h>


static inline unsigned int
ip_vs_sed_dest_overhead(struct ip_vs_dest *dest)
{
	/*
	 * We only use the active connection number in the cost
	 * calculation here.
	 */
	return atomic_read(&dest->activeconns) + 1;
}


static struct ip_vs_dest *
ip_vs_sed_schedule(struct ip_vs_service *svc, const struct sk_buff *skb)
{
	struct ip_vs_dest *dest, *least;
	unsigned int loh, doh;

	IP_VS_DBG(6, "%s(): Scheduling...\n", __func__);

	/*
	 * We calculate the load of each dest server as follows:
	 *	(server expected overhead) / dest->weight
	 *
	 * Remember -- no floats in kernel mode!!!
	 * The comparison of h1*w2 > h2*w1 is equivalent to that of
	 *		  h1/w1 > h2/w2
	 * if every weight is larger than zero.
	 *
	 * The server with weight=0 is quiesced and will not receive any
	 * new connections.
	 */

	list_for_each_entry(dest, &svc->destinations, n_list) {
		if (!(dest->flags & IP_VS_DEST_F_OVERLOAD) &&
		    atomic_read(&dest->weight) > 0) {
			least = dest;
			loh = ip_vs_sed_dest_overhead(least);
			goto nextstage;
		}
	}
	IP_VS_ERR_RL("SED: no destination available\n");
	return NULL;

	/*
	 *    Find the destination with the least load.
	 */
  nextstage:
	list_for_each_entry_continue(dest, &svc->destinations, n_list) {
		if (dest->flags & IP_VS_DEST_F_OVERLOAD)
			continue;
		doh = ip_vs_sed_dest_overhead(dest);
		if (loh * atomic_read(&dest->weight) >
		    doh * atomic_read(&least->weight)) {
			least = dest;
			loh = doh;
		}
	}

	IP_VS_DBG_BUF(6, "SED: server %s:%u "
		      "activeconns %d refcnt %d weight %d overhead %d\n",
		      IP_VS_DBG_ADDR(svc->af, &least->addr), ntohs(least->port),
		      atomic_read(&least->activeconns),
		      atomic_read(&least->refcnt),
		      atomic_read(&least->weight), loh);

	return least;
}


static struct ip_vs_scheduler ip_vs_sed_scheduler =
{
	.name =			"sed",
	.refcnt =		ATOMIC_INIT(0),
	.module =		THIS_MODULE,
	.n_list =		LIST_HEAD_INIT(ip_vs_sed_scheduler.n_list),
	.schedule =		ip_vs_sed_schedule,
};


static int __init ip_vs_sed_init(void)
{
	return register_ip_vs_scheduler(&ip_vs_sed_scheduler);
}

static void __exit ip_vs_sed_cleanup(void)
{
	unregister_ip_vs_scheduler(&ip_vs_sed_scheduler);
}

module_init(ip_vs_sed_init);
module_exit(ip_vs_sed_cleanup);
MODULE_LICENSE("GPL");
