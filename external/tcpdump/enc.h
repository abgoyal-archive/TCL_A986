
/* @(#) $Header: /tcpdump/master/tcpdump/enc.h,v 1.1 2003/03/08 08:55:33 guy Exp $ (LBL) */
/* From $OpenBSD: if_enc.h,v 1.8 2001/06/25 05:14:00 angelos Exp $ */

#define ENC_HDRLEN	12

/* From $OpenBSD: mbuf.h,v 1.56 2002/01/25 15:50:23 art Exp $	*/
#define M_CONF		0x0400  /* packet was encrypted (ESP-transport) */
#define M_AUTH		0x0800  /* packet was authenticated (AH) */

struct enchdr {
	u_int32_t af;
	u_int32_t spi;
	u_int32_t flags;
};
