


#define	LLC_U_FMT	3
#define	LLC_GSAP	1
#define	LLC_IG	        1 /* Individual / Group */
#define LLC_S_FMT	1

#define	LLC_U_POLL	0x10
#define	LLC_IS_POLL	0x0100
#define	LLC_XID_FI	0x81

#define	LLC_U_CMD(u)	((u) & 0xef)
#define	LLC_UI		0x03
#define	LLC_UA		0x63
#define	LLC_DISC	0x43
#define	LLC_DM		0x0f
#define	LLC_SABME	0x6f
#define	LLC_TEST	0xe3
#define	LLC_XID		0xaf
#define	LLC_FRMR	0x87

#define	LLC_S_CMD(is)	(((is) >> 2) & 0x03)
#define	LLC_RR		0x0001
#define	LLC_RNR		0x0005
#define	LLC_REJ		0x0009

#define LLC_IS_NR(is)	(((is) >> 9) & 0x7f)
#define LLC_I_NS(is)	(((is) >> 1) & 0x7f)

#ifndef LLCSAP_NULL
#define	LLCSAP_NULL		0x00
#endif
#ifndef LLCSAP_GLOBAL
#define	LLCSAP_GLOBAL		0xff
#endif
#ifndef LLCSAP_8021B_I
#define	LLCSAP_8021B_I		0x02
#endif
#ifndef LLCSAP_8021B_G
#define	LLCSAP_8021B_G		0x03
#endif
#ifndef LLCSAP_SNA
#define	LLCSAP_SNA		0x04
#endif
#ifndef LLCSAP_IP
#define	LLCSAP_IP		0x06
#endif
#ifndef LLCSAP_PROWAYNM
#define	LLCSAP_PROWAYNM		0x0e
#endif
#ifndef LLCSAP_8021D
#define	LLCSAP_8021D		0x42
#endif
#ifndef LLCSAP_RS511
#define	LLCSAP_RS511		0x4e
#endif
#ifndef LLCSAP_ISO8208
#define	LLCSAP_ISO8208		0x7e
#endif
#ifndef LLCSAP_PROWAY
#define	LLCSAP_PROWAY		0x8e
#endif
#ifndef LLCSAP_SNAP
#define	LLCSAP_SNAP		0xaa
#endif
#ifndef LLCSAP_IPX
#define LLCSAP_IPX		0xe0
#endif
#ifndef LLCSAP_NETBEUI
#define LLCSAP_NETBEUI		0xf0
#endif
#ifndef LLCSAP_ISONS
#define	LLCSAP_ISONS		0xfe
#endif

#define	PID_CISCO_CDP		0x2000	/* Cisco Discovery Protocol */
#define	PID_CISCO_VTP		0x2003	/* Cisco VLAN Trunk Protocol */
#define	PID_CISCO_DTP		0x2004	/* Cisco Dynamic Trunk Protocol */

#define PID_RFC2684_ETH_FCS	0x0001	/* Ethernet, with FCS */
#define PID_RFC2684_ETH_NOFCS	0x0007	/* Ethernet, without FCS */
#define PID_RFC2684_802_4_FCS	0x0002	/* 802.4, with FCS */
#define PID_RFC2684_802_4_NOFCS	0x0008	/* 802.4, without FCS */
#define PID_RFC2684_802_5_FCS	0x0003	/* 802.5, with FCS */
#define PID_RFC2684_802_5_NOFCS	0x0009	/* 802.5, without FCS */
#define PID_RFC2684_FDDI_FCS	0x0004	/* FDDI, with FCS */
#define PID_RFC2684_FDDI_NOFCS	0x000a	/* FDDI, without FCS */
#define PID_RFC2684_802_6_FCS	0x0005	/* 802.6, with FCS */
#define PID_RFC2684_802_6_NOFCS	0x000b	/* 802.6, without FCS */
#define PID_RFC2684_BPDU	0x000e	/* BPDUs */
