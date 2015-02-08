


#include <linux/module.h>
#include <linux/moduleparam.h>

#include <net/irda/irda.h>
#include <net/irda/irmod.h>		/* notify_t */
#include <net/irda/irlap.h>		/* irlap_init */
#include <net/irda/irlmp.h>		/* irlmp_init */
#include <net/irda/iriap.h>		/* iriap_init */
#include <net/irda/irttp.h>		/* irttp_init */
#include <net/irda/irda_device.h>	/* irda_device_init */

#ifdef CONFIG_IRDA_DEBUG
unsigned int irda_debug = IRDA_DEBUG_LEVEL;
module_param_named(debug, irda_debug, uint, 0);
MODULE_PARM_DESC(debug, "IRDA debugging level");
EXPORT_SYMBOL(irda_debug);
#endif

static struct packet_type irda_packet_type __read_mostly = {
	.type	= cpu_to_be16(ETH_P_IRDA),
	.func	= irlap_driver_rcv,	/* Packet type handler irlap_frame.c */
};

void irda_notify_init(notify_t *notify)
{
	notify->data_indication = NULL;
	notify->udata_indication = NULL;
	notify->connect_confirm = NULL;
	notify->connect_indication = NULL;
	notify->disconnect_indication = NULL;
	notify->flow_indication = NULL;
	notify->status_indication = NULL;
	notify->instance = NULL;
	strlcpy(notify->name, "Unknown", sizeof(notify->name));
}
EXPORT_SYMBOL(irda_notify_init);

static int __init irda_init(void)
{
	int ret = 0;

	IRDA_DEBUG(0, "%s()\n", __func__);

	/* Lower layer of the stack */
	irlmp_init();
	irlap_init();

	/* Driver/dongle support */
	irda_device_init();

	/* Higher layers of the stack */
	iriap_init();
	irttp_init();
	ret = irsock_init();
	if (ret < 0)
		goto out_err_1;

	/* Add IrDA packet type (Start receiving packets) */
	dev_add_pack(&irda_packet_type);

	/* External APIs */
#ifdef CONFIG_PROC_FS
	irda_proc_register();
#endif
#ifdef CONFIG_SYSCTL
	ret = irda_sysctl_register();
	if (ret < 0)
		goto out_err_2;
#endif

	ret = irda_nl_register();
	if (ret < 0)
		goto out_err_3;

	return 0;

 out_err_3:
#ifdef CONFIG_SYSCTL
	irda_sysctl_unregister();
 out_err_2:
#endif
#ifdef CONFIG_PROC_FS
	irda_proc_unregister();
#endif

	/* Remove IrDA packet type (stop receiving packets) */
	dev_remove_pack(&irda_packet_type);

	/* Remove higher layers */
	irsock_cleanup();
 out_err_1:
	irttp_cleanup();
	iriap_cleanup();

	/* Remove lower layers */
	irda_device_cleanup();
	irlap_cleanup(); /* Must be done before irlmp_cleanup()! DB */

	/* Remove middle layer */
	irlmp_cleanup();


	return ret;
}

static void __exit irda_cleanup(void)
{
	/* Remove External APIs */
	irda_nl_unregister();

#ifdef CONFIG_SYSCTL
	irda_sysctl_unregister();
#endif
#ifdef CONFIG_PROC_FS
	irda_proc_unregister();
#endif

	/* Remove IrDA packet type (stop receiving packets) */
	dev_remove_pack(&irda_packet_type);

	/* Remove higher layers */
	irsock_cleanup();
	irttp_cleanup();
	iriap_cleanup();

	/* Remove lower layers */
	irda_device_cleanup();
	irlap_cleanup(); /* Must be done before irlmp_cleanup()! DB */

	/* Remove middle layer */
	irlmp_cleanup();
}

subsys_initcall(irda_init);
module_exit(irda_cleanup);

MODULE_AUTHOR("Dag Brattli <dagb@cs.uit.no> & Jean Tourrilhes <jt@hpl.hp.com>");
MODULE_DESCRIPTION("The Linux IrDA Protocol Stack");
MODULE_LICENSE("GPL");
MODULE_ALIAS_NETPROTO(PF_IRDA);
