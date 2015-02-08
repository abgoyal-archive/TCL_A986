/*	$NetBSD: strnames.h,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/* Id: strnames.h,v 1.7 2005/04/18 10:04:26 manubsd Exp */


#ifndef _STRNAMES_H
#define _STRNAMES_H

extern char *num2str __P((int n));

extern char *s_isakmp_state __P((int, int, int));
extern char *s_isakmp_certtype __P((int));
extern char *s_isakmp_etype __P((int));
extern char *s_isakmp_notify_msg __P((int));
extern char *s_isakmp_nptype __P((int));
extern char *s_ipsecdoi_proto __P((int));
extern char *s_ipsecdoi_trns_isakmp __P((int));
extern char *s_ipsecdoi_trns_ah __P((int));
extern char *s_ipsecdoi_trns_esp __P((int));
extern char *s_ipsecdoi_trns_ipcomp __P((int));
extern char *s_ipsecdoi_trns __P((int, int));
extern char *s_ipsecdoi_attr __P((int));
extern char *s_ipsecdoi_ltype __P((int));
extern char *s_ipsecdoi_encmode __P((int));
extern char *s_ipsecdoi_auth __P((int));
extern char *s_ipsecdoi_attr_v __P((int, int));
extern char *s_ipsecdoi_ident __P((int));
extern char *s_oakley_attr __P((int));
extern char *s_attr_isakmp_enc __P((int));
extern char *s_attr_isakmp_hash __P((int));
extern char *s_oakley_attr_method __P((int));
extern char *s_attr_isakmp_desc __P((int));
extern char *s_attr_isakmp_group __P((int));
extern char *s_attr_isakmp_ltype __P((int));
extern char *s_oakley_attr_v __P((int, int));
extern char *s_ipsec_level __P((int));
extern char *s_algclass __P((int));
extern char *s_algtype __P((int, int));
extern char *s_pfkey_type __P((int));
extern char *s_pfkey_satype __P((int));
extern char *s_direction __P((int));
extern char *s_proto __P((int));
extern char *s_doi __P((int));
extern char *s_etype __P((int));
extern char *s_idtype __P((int));
extern char *s_switch __P((int));
#ifdef ENABLE_HYBRID
extern char *s_isakmp_cfg_type __P((int));
extern char *s_isakmp_cfg_ptype __P((int));
#endif

#endif /* _STRNAMES_H */
