
/*	$NetBSD: esp.h,v 1.13 2000/09/26 08:37:38 itojun Exp $	*/
/*	$KAME: esp.h,v 1.15 2000/09/20 18:15:22 itojun Exp $	*/



#ifndef _NETINET6_ESP_H_
#define _NETINET6_ESP_H_

struct esp {
	u_int32_t	esp_spi;	/* ESP */
	/*variable size, 32bit bound*/	/* Initialization Vector */
	/*variable size*/		/* Payload data */
	/*variable size*/		/* padding */
	/*8bit*/			/* pad size */
	/*8bit*/			/* next header */
	/*8bit*/			/* next header */
	/*variable size, 32bit bound*/	/* Authentication data (new IPsec) */
};

struct newesp {
	u_int32_t	esp_spi;	/* ESP */
	u_int32_t	esp_seq;	/* Sequence number */
	/*variable size*/		/* (IV and) Payload data */
	/*variable size*/		/* padding */
	/*8bit*/			/* pad size */
	/*8bit*/			/* next header */
	/*8bit*/			/* next header */
	/*variable size, 32bit bound*/	/* Authentication data */
};

struct esptail {
	u_int8_t	esp_padlen;	/* pad length */
	u_int8_t	esp_nxt;	/* Next header */
	/*variable size, 32bit bound*/	/* Authentication data (new IPsec)*/
};

#endif /*_NETINET6_ESP_H_*/
