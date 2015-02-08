

#ifndef DVB_DUMMY_FE_H
#define DVB_DUMMY_FE_H

#include <linux/dvb/frontend.h>
#include "dvb_frontend.h"

#if defined(CONFIG_DVB_DUMMY_FE) || (defined(CONFIG_DVB_DUMMY_FE_MODULE) && \
defined(MODULE))
extern struct dvb_frontend* dvb_dummy_fe_ofdm_attach(void);
extern struct dvb_frontend* dvb_dummy_fe_qpsk_attach(void);
extern struct dvb_frontend* dvb_dummy_fe_qam_attach(void);
#else
static inline struct dvb_frontend *dvb_dummy_fe_ofdm_attach(void)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
static inline struct dvb_frontend *dvb_dummy_fe_qpsk_attach(void)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
static inline struct dvb_frontend *dvb_dummy_fe_qam_attach(void)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif /* CONFIG_DVB_DUMMY_FE */

#endif // DVB_DUMMY_FE_H
