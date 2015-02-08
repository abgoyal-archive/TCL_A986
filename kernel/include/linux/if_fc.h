
#ifndef _LINUX_IF_FC_H
#define _LINUX_IF_FC_H

#include <linux/types.h>

#define FC_ALEN	6		/* Octets in one ethernet addr	 */
#define FC_HLEN   (sizeof(struct fch_hdr)+sizeof(struct fcllc))
#define FC_ID_LEN 3		/* Octets in a Fibre Channel Address */

/* LLC and SNAP constants */
#define EXTENDED_SAP 0xAA
#define UI_CMD       0x03


struct fch_hdr {
	__u8  daddr[FC_ALEN];		/* destination address */
	__u8  saddr[FC_ALEN];		/* source address */
};

/* This is a Fibre Channel LLC structure */
struct fcllc {
	__u8  dsap;			/* destination SAP */
	__u8  ssap;			/* source SAP */
	__u8  llc;			/* LLC control field */
	__u8  protid[3];		/* protocol id */
	__be16 ethertype;		/* ether type field */
};

#endif	/* _LINUX_IF_FC_H */
