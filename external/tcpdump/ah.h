
/*	$NetBSD: ah.h,v 1.12 2000/07/23 05:23:04 itojun Exp $	*/
/*	$KAME: ah.h,v 1.12 2000/07/20 17:41:01 itojun Exp $	*/



#ifndef _NETINET6_AH_H_
#define _NETINET6_AH_H_

struct ah {
	u_int8_t	ah_nxt;		/* Next Header */
	u_int8_t	ah_len;		/* Length of data, in 32bit */
	u_int16_t	ah_reserve;	/* Reserved for future use */
	u_int32_t	ah_spi;		/* Security parameter index */
	/* variable size, 32bit bound*/	/* Authentication data */
};

struct newah {
	u_int8_t	ah_nxt;		/* Next Header */
	u_int8_t	ah_len;		/* Length of data + 1, in 32bit */
	u_int16_t	ah_reserve;	/* Reserved for future use */
	u_int32_t	ah_spi;		/* Security parameter index */
	u_int32_t	ah_seq;		/* Sequence number field */
	/* variable size, 32bit bound*/	/* Authentication data */
};

#endif /*_NETINET6_AH_H_*/
