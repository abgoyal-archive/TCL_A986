

#ifndef _NETINET_ICMP_VAR_H_
#define _NETINET_ICMP_VAR_H_

struct icmpstat {
/* statistics related to input messages processed */
	u_long  icps_received;		/* #ICMP packets received */
	u_long	icps_tooshort;		/* packet < ICMP_MINLEN */
	u_long	icps_checksum;		/* bad checksum */
	u_long	icps_notsupp;		/* #ICMP packets not supported */
	u_long  icps_badtype;		/* #with bad type feild */
	u_long	icps_reflect;		/* number of responses */
};

#define	ICMPCTL_MASKREPL	1	/* allow replies to netmask requests */
#define	ICMPCTL_STATS		2	/* statistics (read-only) */
#define ICMPCTL_MAXID		3

#define ICMPCTL_NAMES { \
	{ 0, 0 }, \
	{ "maskrepl", CTLTYPE_INT }, \
	{ "stats", CTLTYPE_STRUCT }, \
}

#ifdef LOG_ENABLED
extern struct icmpstat icmpstat;
#endif

#endif
