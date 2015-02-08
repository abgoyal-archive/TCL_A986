/*	$NetBSD: isakmp_inf.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/* Id: isakmp_inf.h,v 1.6 2005/05/07 14:15:59 manubsd Exp */


#ifndef _ISAKMP_INF_H
#define _ISAKMP_INF_H

struct saproto;
extern int isakmp_info_recv __P((struct ph1handle *, vchar_t *));
extern int isakmp_info_send_d1 __P((struct ph1handle *));
extern int isakmp_info_send_d2 __P((struct ph2handle *));
extern int isakmp_info_send_nx __P((struct isakmp *,
	struct sockaddr *, struct sockaddr *, int, vchar_t *));
extern int isakmp_info_send_n1 __P((struct ph1handle *, int, vchar_t *));
extern int isakmp_info_send_n2 __P((struct ph2handle *, int, vchar_t *));
extern int isakmp_info_send_common __P((struct ph1handle *,
	vchar_t *, u_int32_t, int));

extern vchar_t * isakmp_add_pl_n __P((vchar_t *, u_int8_t **, int,
	struct saproto *, vchar_t *));

extern void isakmp_check_notify __P((struct isakmp_gen *, struct ph1handle *));

#ifdef ENABLE_DPD
extern int isakmp_sched_r_u __P((struct ph1handle *, int));
#endif

extern void purge_ipsec_spi __P((struct sockaddr *, int,	u_int32_t *, size_t));
extern int tunnel_mode_prop __P((struct saprop *));

#endif /* _ISAKMP_INF_H */
