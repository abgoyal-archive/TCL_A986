/*	$NetBSD: vendorid.h,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/* Id: vendorid.h,v 1.11 2006/02/17 14:09:10 vanhu Exp */


#ifndef _VENDORID_H
#define _VENDORID_H

/* The unknown vendor ID. */
#define	VENDORID_UNKNOWN	-1


/* Our default vendor ID. */
#define	VENDORID_DEFAULT	VENDORID_IPSEC_TOOLS

#define	VENDORID_IPSEC_TOOLS 0		

#define	VENDORID_GSSAPI_LONG	1
#define	VENDORID_GSSAPI		2
#define	VENDORID_MS_NT5		3
#define	VENDOR_SUPPORTS_GSSAPI(x)					\
	((x) == VENDORID_GSSAPI_LONG ||					\
	 (x) == VENDORID_GSSAPI ||					\
	 (x) == VENDORID_MS_NT5)

/* NAT-T support */
#define VENDORID_NATT_00	4
#define VENDORID_NATT_01	5
#define VENDORID_NATT_02	6
#define VENDORID_NATT_02_N	7
#define VENDORID_NATT_03	8
#define VENDORID_NATT_04	9
#define VENDORID_NATT_05	10
#define VENDORID_NATT_06	11
#define VENDORID_NATT_07	12
#define VENDORID_NATT_08	13
#define VENDORID_NATT_RFC	14

#define VENDORID_NATT_FIRST	VENDORID_NATT_00
#define VENDORID_NATT_LAST	VENDORID_NATT_RFC


#define MAX_NATT_VID_COUNT	(VENDORID_NATT_LAST - VENDORID_NATT_FIRST + 1 )

/* Hybrid auth */
#define VENDORID_XAUTH		15
#define VENDORID_UNITY		16

/* IKE fragmentation */
#define VENDORID_FRAG		17

/* Dead Peer Detection */
#define VENDORID_DPD		18


#define	VENDORID_KAME		19


struct vendor_id {
	int		id;
	const char	*string;
	vchar_t		*hash;
};

vchar_t *set_vendorid __P((int));
int check_vendorid __P((struct isakmp_gen *));

void compute_vendorids __P((void));
const char *vid_string_by_id __P((int id));

#endif /* _VENDORID_H */
