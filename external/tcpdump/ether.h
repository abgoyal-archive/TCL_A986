
/* @(#) $Header: /tcpdump/master/tcpdump/ether.h,v 1.8 2002/12/11 07:13:51 guy Exp $ (LBL) */

#define	ETHERMTU	1500

#define	ETHER_ADDR_LEN		6

struct	ether_header {
	u_int8_t	ether_dhost[ETHER_ADDR_LEN];
	u_int8_t	ether_shost[ETHER_ADDR_LEN];
	u_int16_t	ether_type;
};

#define ETHER_HDRLEN		14
