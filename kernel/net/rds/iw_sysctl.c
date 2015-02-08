
#include <linux/kernel.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>

#include "iw.h"

static struct ctl_table_header *rds_iw_sysctl_hdr;

unsigned long rds_iw_sysctl_max_send_wr = RDS_IW_DEFAULT_SEND_WR;
unsigned long rds_iw_sysctl_max_recv_wr = RDS_IW_DEFAULT_RECV_WR;
unsigned long rds_iw_sysctl_max_recv_allocation = (128 * 1024 * 1024) / RDS_FRAG_SIZE;
static unsigned long rds_iw_sysctl_max_wr_min = 1;
/* hardware will fail CQ creation long before this */
static unsigned long rds_iw_sysctl_max_wr_max = (u32)~0;

unsigned long rds_iw_sysctl_max_unsig_wrs = 16;
static unsigned long rds_iw_sysctl_max_unsig_wr_min = 1;
static unsigned long rds_iw_sysctl_max_unsig_wr_max = 64;

unsigned long rds_iw_sysctl_max_unsig_bytes = (16 << 20);
static unsigned long rds_iw_sysctl_max_unsig_bytes_min = 1;
static unsigned long rds_iw_sysctl_max_unsig_bytes_max = ~0UL;

unsigned int rds_iw_sysctl_flow_control = 1;

ctl_table rds_iw_sysctl_table[] = {
	{
		.procname       = "max_send_wr",
		.data		= &rds_iw_sysctl_max_send_wr,
		.maxlen         = sizeof(unsigned long),
		.mode           = 0644,
		.proc_handler   = proc_doulongvec_minmax,
		.extra1		= &rds_iw_sysctl_max_wr_min,
		.extra2		= &rds_iw_sysctl_max_wr_max,
	},
	{
		.procname       = "max_recv_wr",
		.data		= &rds_iw_sysctl_max_recv_wr,
		.maxlen         = sizeof(unsigned long),
		.mode           = 0644,
		.proc_handler   = proc_doulongvec_minmax,
		.extra1		= &rds_iw_sysctl_max_wr_min,
		.extra2		= &rds_iw_sysctl_max_wr_max,
	},
	{
		.procname       = "max_unsignaled_wr",
		.data		= &rds_iw_sysctl_max_unsig_wrs,
		.maxlen         = sizeof(unsigned long),
		.mode           = 0644,
		.proc_handler   = proc_doulongvec_minmax,
		.extra1		= &rds_iw_sysctl_max_unsig_wr_min,
		.extra2		= &rds_iw_sysctl_max_unsig_wr_max,
	},
	{
		.procname       = "max_unsignaled_bytes",
		.data		= &rds_iw_sysctl_max_unsig_bytes,
		.maxlen         = sizeof(unsigned long),
		.mode           = 0644,
		.proc_handler   = proc_doulongvec_minmax,
		.extra1		= &rds_iw_sysctl_max_unsig_bytes_min,
		.extra2		= &rds_iw_sysctl_max_unsig_bytes_max,
	},
	{
		.procname       = "max_recv_allocation",
		.data		= &rds_iw_sysctl_max_recv_allocation,
		.maxlen         = sizeof(unsigned long),
		.mode           = 0644,
		.proc_handler   = proc_doulongvec_minmax,
	},
	{
		.procname	= "flow_control",
		.data		= &rds_iw_sysctl_flow_control,
		.maxlen		= sizeof(rds_iw_sysctl_flow_control),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{ }
};

static struct ctl_path rds_iw_sysctl_path[] = {
	{ .procname = "net", },
	{ .procname = "rds", },
	{ .procname = "iw", },
	{ }
};

void rds_iw_sysctl_exit(void)
{
	if (rds_iw_sysctl_hdr)
		unregister_sysctl_table(rds_iw_sysctl_hdr);
}

int __init rds_iw_sysctl_init(void)
{
	rds_iw_sysctl_hdr = register_sysctl_paths(rds_iw_sysctl_path, rds_iw_sysctl_table);
	if (rds_iw_sysctl_hdr == NULL)
		return -ENOMEM;
	return 0;
}
