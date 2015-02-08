
 
#ifndef _SNMP_H
#define _SNMP_H

#include <linux/cache.h>
#include <linux/snmp.h>
#include <linux/smp.h>

struct snmp_mib {
	const char *name;
	int entry;
};

#define SNMP_MIB_ITEM(_name,_entry)	{	\
	.name = _name,				\
	.entry = _entry,			\
}

#define SNMP_MIB_SENTINEL {	\
	.name = NULL,		\
	.entry = 0,		\
}


/* IPstats */
#define IPSTATS_MIB_MAX	__IPSTATS_MIB_MAX
struct ipstats_mib {
	unsigned long	mibs[IPSTATS_MIB_MAX];
};

/* ICMP */
#define ICMP_MIB_DUMMY	__ICMP_MIB_MAX
#define ICMP_MIB_MAX	(__ICMP_MIB_MAX + 1)

struct icmp_mib {
	unsigned long	mibs[ICMP_MIB_MAX];
};

#define ICMPMSG_MIB_MAX	__ICMPMSG_MIB_MAX
struct icmpmsg_mib {
	unsigned long	mibs[ICMPMSG_MIB_MAX];
};

/* ICMP6 (IPv6-ICMP) */
#define ICMP6_MIB_MAX	__ICMP6_MIB_MAX
struct icmpv6_mib {
	unsigned long	mibs[ICMP6_MIB_MAX];
};

#define ICMP6MSG_MIB_MAX  __ICMP6MSG_MIB_MAX
struct icmpv6msg_mib {
	unsigned long	mibs[ICMP6MSG_MIB_MAX];
};


/* TCP */
#define TCP_MIB_MAX	__TCP_MIB_MAX
struct tcp_mib {
	unsigned long	mibs[TCP_MIB_MAX];
};

/* UDP */
#define UDP_MIB_MAX	__UDP_MIB_MAX
struct udp_mib {
	unsigned long	mibs[UDP_MIB_MAX];
};

/* Linux */
#define LINUX_MIB_MAX	__LINUX_MIB_MAX
struct linux_mib {
	unsigned long	mibs[LINUX_MIB_MAX];
};

/* Linux Xfrm */
#define LINUX_MIB_XFRMMAX	__LINUX_MIB_XFRMMAX
struct linux_xfrm_mib {
	unsigned long	mibs[LINUX_MIB_XFRMMAX];
};

#define DEFINE_SNMP_STAT(type, name)	\
	__typeof__(type) __percpu *name[2]
#define DECLARE_SNMP_STAT(type, name)	\
	extern __typeof__(type) __percpu *name[2]

#define SNMP_STAT_BHPTR(name)	(name[0])
#define SNMP_STAT_USRPTR(name)	(name[1])

#define SNMP_INC_STATS_BH(mib, field)	\
			__this_cpu_inc(mib[0]->mibs[field])
#define SNMP_INC_STATS_USER(mib, field)	\
			this_cpu_inc(mib[1]->mibs[field])
#define SNMP_INC_STATS(mib, field)	\
			this_cpu_inc(mib[!in_softirq()]->mibs[field])
#define SNMP_DEC_STATS(mib, field)	\
			this_cpu_dec(mib[!in_softirq()]->mibs[field])
#define SNMP_ADD_STATS_BH(mib, field, addend)	\
			__this_cpu_add(mib[0]->mibs[field], addend)
#define SNMP_ADD_STATS_USER(mib, field, addend)	\
			this_cpu_add(mib[1]->mibs[field], addend)
#define SNMP_ADD_STATS(mib, field, addend)	\
			this_cpu_add(mib[!in_softirq()]->mibs[field], addend)
#define SNMP_UPD_PO_STATS(mib, basefield, addend)	\
	do { \
		__typeof__(*mib[0]) *ptr; \
		preempt_disable(); \
		ptr = this_cpu_ptr((mib)[!in_softirq()]); \
		ptr->mibs[basefield##PKTS]++; \
		ptr->mibs[basefield##OCTETS] += addend;\
		preempt_enable(); \
	} while (0)
#define SNMP_UPD_PO_STATS_BH(mib, basefield, addend)	\
	do { \
		__typeof__(*mib[0]) *ptr = \
			__this_cpu_ptr((mib)[!in_softirq()]); \
		ptr->mibs[basefield##PKTS]++; \
		ptr->mibs[basefield##OCTETS] += addend;\
	} while (0)
#endif
