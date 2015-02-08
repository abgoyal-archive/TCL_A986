
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/sysctl.h>
#include <linux/init.h>
#include <linux/io.h>

static unsigned int isa_membase, isa_portbase, isa_portshift;

static ctl_table ctl_isa_vars[4] = {
	{
		.procname	= "membase",
		.data		= &isa_membase, 
		.maxlen		= sizeof(isa_membase),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	}, {
		.procname	= "portbase",
		.data		= &isa_portbase, 
		.maxlen		= sizeof(isa_portbase),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	}, {
		.procname	= "portshift",
		.data		= &isa_portshift, 
		.maxlen		= sizeof(isa_portshift),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	}, {}
};

static struct ctl_table_header *isa_sysctl_header;

static ctl_table ctl_isa[2] = {
	{
		.procname	= "isa",
		.mode		= 0555,
		.child		= ctl_isa_vars,
	}, {}
};

static ctl_table ctl_bus[2] = {
	{
		.procname	= "bus",
		.mode		= 0555,
		.child		= ctl_isa,
	}, {}
};

void __init
register_isa_ports(unsigned int membase, unsigned int portbase, unsigned int portshift)
{
	isa_membase = membase;
	isa_portbase = portbase;
	isa_portshift = portshift;
	isa_sysctl_header = register_sysctl_table(ctl_bus);
}
