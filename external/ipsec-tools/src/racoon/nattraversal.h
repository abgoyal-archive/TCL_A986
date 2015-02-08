/*	$NetBSD: nattraversal.h,v 1.6 2006/09/09 16:22:09 manu Exp $	*/


#ifndef _NATTRAVERSAL_H
#define _NATTRAVERSAL_H

#include "vendorid.h"

#define	NAT_ANNOUNCED		(1L<<0)
#define	NAT_DETECTED_ME		(1L<<1)
#define	NAT_DETECTED_PEER	(1L<<2)
#define	NAT_PORTS_CHANGED	(1L<<3)
#define	NAT_KA_QUEUED		(1L<<4)
#define	NAT_ADD_NON_ESP_MARKER	(1L<<5)

#define	NATT_AVAILABLE(ph1)	((iph1)->natt_flags & NAT_ANNOUNCED)

#define	NAT_DETECTED	(NAT_DETECTED_ME | NAT_DETECTED_PEER)

#define	NON_ESP_MARKER_LEN	sizeof(u_int32_t)
#define	NON_ESP_MARKER_USE(iph1)	((iph1)->natt_flags & NAT_ADD_NON_ESP_MARKER)

#define NATT_OFF	FLASE	/* = 0 */
#define NATT_ON		TRUE	/* = 1 */
#define NATT_FORCE	2

struct ph1natt_options {
  int		version;
  u_int16_t	float_port;
  u_int16_t	mode_udp_tunnel;
  u_int16_t	mode_udp_transport;
  u_int16_t	encaps_type; /* ESPINUDP / ESPINUDP_NON_IKE */
  u_int16_t	mode_udp_diff;
  u_int16_t	payload_nat_d;
  u_int16_t	payload_nat_oa;
};

struct ph2natt {
  u_int8_t	type;
  u_int16_t	sport;
  u_int16_t	dport;
  struct sockaddr	*oa;
  u_int16_t	frag;
};

int natt_vendorid (int vid);
vchar_t *natt_hash_addr (struct ph1handle *iph1, struct sockaddr *addr);
int natt_compare_addr_hash (struct ph1handle *iph1, vchar_t *natd_received, int natd_seq);
int natt_udp_encap (int encmode);
int natt_fill_options (struct ph1natt_options *opts, int version);
void natt_float_ports (struct ph1handle *iph1);
void natt_handle_vendorid (struct ph1handle *iph1, int vid_numeric);


struct payload_list *
isakmp_plist_append_natt_vids (struct payload_list *plist, vchar_t *vid_natt[MAX_NATT_VID_COUNT]);


/* NAT keepalive functions */
void natt_keepalive_init (void);
int natt_keepalive_add (struct sockaddr *src, struct sockaddr *dst);
int natt_keepalive_add_ph1 (struct ph1handle *iph1);
void natt_keepalive_remove (struct sockaddr *src, struct sockaddr *dst);

/* Walk through all rmconfigs and tell if NAT-T is enabled in at least one. */
int natt_enabled_in_rmconf (void);

#endif /* _NATTRAVERSAL_H */
