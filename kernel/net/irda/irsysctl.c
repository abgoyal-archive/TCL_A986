

#include <linux/mm.h>
#include <linux/ctype.h>
#include <linux/sysctl.h>
#include <linux/init.h>

#include <net/irda/irda.h>		/* irda_debug */
#include <net/irda/irlmp.h>
#include <net/irda/timer.h>
#include <net/irda/irias_object.h>

extern int  sysctl_discovery;
extern int  sysctl_discovery_slots;
extern int  sysctl_discovery_timeout;
extern int  sysctl_slot_timeout;
extern int  sysctl_fast_poll_increase;
extern char sysctl_devname[];
extern int  sysctl_max_baud_rate;
extern int  sysctl_min_tx_turn_time;
extern int  sysctl_max_tx_data_size;
extern int  sysctl_max_tx_window;
extern int  sysctl_max_noreply_time;
extern int  sysctl_warn_noreply_time;
extern int  sysctl_lap_keepalive_time;

extern struct irlmp_cb *irlmp;

/* this is needed for the proc_dointvec_minmax - Jean II */
static int max_discovery_slots = 16;		/* ??? */
static int min_discovery_slots = 1;
static int max_slot_timeout = 160;
static int min_slot_timeout = 20;
static int max_max_baud_rate = 16000000;	/* See qos.c - IrLAP spec */
static int min_max_baud_rate = 2400;
static int max_min_tx_turn_time = 10000;	/* See qos.c - IrLAP spec */
static int min_min_tx_turn_time;
static int max_max_tx_data_size = 2048;		/* See qos.c - IrLAP spec */
static int min_max_tx_data_size = 64;
static int max_max_tx_window = 7;		/* See qos.c - IrLAP spec */
static int min_max_tx_window = 1;
static int max_max_noreply_time = 40;		/* See qos.c - IrLAP spec */
static int min_max_noreply_time = 3;
static int max_warn_noreply_time = 3;		/* 3s == standard */
static int min_warn_noreply_time = 1;		/* 1s == min WD_TIMER */
static int max_lap_keepalive_time = 10000;	/* 10s */
static int min_lap_keepalive_time = 100;	/* 100us */

static int do_devname(ctl_table *table, int write,
		      void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;

	ret = proc_dostring(table, write, buffer, lenp, ppos);
	if (ret == 0 && write) {
		struct ias_value *val;

		val = irias_new_string_value(sysctl_devname);
		if (val)
			irias_object_change_attribute("Device", "DeviceName", val);
	}
	return ret;
}


static int do_discovery(ctl_table *table, int write,
                    void __user *buffer, size_t *lenp, loff_t *ppos)
{
       int ret;

       ret = proc_dointvec(table, write, buffer, lenp, ppos);
       if (ret)
	       return ret;

       if (irlmp == NULL)
	       return -ENODEV;

       if (sysctl_discovery)
	       irlmp_start_discovery_timer(irlmp, sysctl_discovery_timeout*HZ);
       else
	       del_timer_sync(&irlmp->discovery_timer);

       return ret;
}

/* One file */
static ctl_table irda_table[] = {
	{
		.procname	= "discovery",
		.data		= &sysctl_discovery,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= do_discovery,
	},
	{
		.procname	= "devname",
		.data		= sysctl_devname,
		.maxlen		= 65,
		.mode		= 0644,
		.proc_handler	= do_devname,
	},
#ifdef CONFIG_IRDA_DEBUG
	{
		.procname	= "debug",
		.data		= &irda_debug,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec
	},
#endif
#ifdef CONFIG_IRDA_FAST_RR
	{
		.procname	= "fast_poll_increase",
		.data		= &sysctl_fast_poll_increase,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec
	},
#endif
	{
		.procname	= "discovery_slots",
		.data		= &sysctl_discovery_slots,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec_minmax,
		.extra1		= &min_discovery_slots,
		.extra2		= &max_discovery_slots
	},
	{
		.procname	= "discovery_timeout",
		.data		= &sysctl_discovery_timeout,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec
	},
	{
		.procname	= "slot_timeout",
		.data		= &sysctl_slot_timeout,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec_minmax,
		.extra1		= &min_slot_timeout,
		.extra2		= &max_slot_timeout
	},
	{
		.procname	= "max_baud_rate",
		.data		= &sysctl_max_baud_rate,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec_minmax,
		.extra1		= &min_max_baud_rate,
		.extra2		= &max_max_baud_rate
	},
	{
		.procname	= "min_tx_turn_time",
		.data		= &sysctl_min_tx_turn_time,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec_minmax,
		.extra1		= &min_min_tx_turn_time,
		.extra2		= &max_min_tx_turn_time
	},
	{
		.procname	= "max_tx_data_size",
		.data		= &sysctl_max_tx_data_size,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec_minmax,
		.extra1		= &min_max_tx_data_size,
		.extra2		= &max_max_tx_data_size
	},
	{
		.procname	= "max_tx_window",
		.data		= &sysctl_max_tx_window,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec_minmax,
		.extra1		= &min_max_tx_window,
		.extra2		= &max_max_tx_window
	},
	{
		.procname	= "max_noreply_time",
		.data		= &sysctl_max_noreply_time,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec_minmax,
		.extra1		= &min_max_noreply_time,
		.extra2		= &max_max_noreply_time
	},
	{
		.procname	= "warn_noreply_time",
		.data		= &sysctl_warn_noreply_time,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec_minmax,
		.extra1		= &min_warn_noreply_time,
		.extra2		= &max_warn_noreply_time
	},
	{
		.procname	= "lap_keepalive_time",
		.data		= &sysctl_lap_keepalive_time,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec_minmax,
		.extra1		= &min_lap_keepalive_time,
		.extra2		= &max_lap_keepalive_time
	},
	{ }
};

static struct ctl_path irda_path[] = {
	{ .procname = "net", },
	{ .procname = "irda", },
	{ }
};

static struct ctl_table_header *irda_table_header;

int __init irda_sysctl_register(void)
{
	irda_table_header = register_sysctl_paths(irda_path, irda_table);
	if (!irda_table_header)
		return -ENOMEM;

	return 0;
}

void irda_sysctl_unregister(void)
{
	unregister_sysctl_table(irda_table_header);
}



