

#ifndef _XT_OSF_H
#define _XT_OSF_H

#include <linux/types.h>

#define MAXGENRELEN		32

#define XT_OSF_GENRE		(1<<0)
#define	XT_OSF_TTL		(1<<1)
#define XT_OSF_LOG		(1<<2)
#define XT_OSF_INVERT		(1<<3)

#define XT_OSF_LOGLEVEL_ALL	0	/* log all matched fingerprints */
#define XT_OSF_LOGLEVEL_FIRST	1	/* log only the first matced fingerprint */
#define XT_OSF_LOGLEVEL_ALL_KNOWN	2 /* do not log unknown packets */

#define XT_OSF_TTL_TRUE		0	/* True ip and fingerprint TTL comparison */
#define XT_OSF_TTL_LESS		1	/* Check if ip TTL is less than fingerprint one */
#define XT_OSF_TTL_NOCHECK	2	/* Do not compare ip and fingerprint TTL at all */

struct xt_osf_info {
	char			genre[MAXGENRELEN];
	__u32			len;
	__u32			flags;
	__u32			loglevel;
	__u32			ttl;
};

struct xt_osf_wc {
	__u32			wc;
	__u32			val;
};

struct xt_osf_opt {
	__u16			kind, length;
	struct xt_osf_wc	wc;
};

struct xt_osf_user_finger {
	struct xt_osf_wc	wss;

	__u8			ttl, df;
	__u16			ss, mss;
	__u16			opt_num;

	char			genre[MAXGENRELEN];
	char			version[MAXGENRELEN];
	char			subtype[MAXGENRELEN];

	/* MAX_IPOPTLEN is maximum if all options are NOPs or EOLs */
	struct xt_osf_opt	opt[MAX_IPOPTLEN];
};

struct xt_osf_nlmsg {
	struct xt_osf_user_finger	f;
	struct iphdr		ip;
	struct tcphdr		tcp;
};

/* Defines for IANA option kinds */

enum iana_options {
	OSFOPT_EOL = 0,		/* End of options */
	OSFOPT_NOP, 		/* NOP */
	OSFOPT_MSS, 		/* Maximum segment size */
	OSFOPT_WSO, 		/* Window scale option */
	OSFOPT_SACKP,		/* SACK permitted */
	OSFOPT_SACK,		/* SACK */
	OSFOPT_ECHO,
	OSFOPT_ECHOREPLY,
	OSFOPT_TS,		/* Timestamp option */
	OSFOPT_POCP,		/* Partial Order Connection Permitted */
	OSFOPT_POSP,		/* Partial Order Service Profile */

	/* Others are not used in the current OSF */
	OSFOPT_EMPTY = 255,
};

enum xt_osf_window_size_options {
	OSF_WSS_PLAIN	= 0,
	OSF_WSS_MSS,
	OSF_WSS_MTU,
	OSF_WSS_MODULO,
	OSF_WSS_MAX,
};

enum xt_osf_msg_types {
	OSF_MSG_ADD,
	OSF_MSG_REMOVE,
	OSF_MSG_MAX,
};

enum xt_osf_attr_type {
	OSF_ATTR_UNSPEC,
	OSF_ATTR_FINGER,
	OSF_ATTR_MAX,
};

#endif				/* _XT_OSF_H */
