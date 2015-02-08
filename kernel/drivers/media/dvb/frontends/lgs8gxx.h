

#ifndef __LGS8GXX_H__
#define __LGS8GXX_H__

#include <linux/dvb/frontend.h>
#include <linux/i2c.h>

#define LGS8GXX_PROD_LGS8913 0
#define LGS8GXX_PROD_LGS8GL5 1
#define LGS8GXX_PROD_LGS8G42 3
#define LGS8GXX_PROD_LGS8G52 4
#define LGS8GXX_PROD_LGS8G54 5
#define LGS8GXX_PROD_LGS8G75 6

struct lgs8gxx_config {

	/* product type */
	u8 prod;

	/* the demodulator's i2c address */
	u8 demod_address;

	/* parallel or serial transport stream */
	u8 serial_ts;

	/* transport stream polarity*/
	u8 ts_clk_pol;

	/* transport stream clock gated by ts_valid */
	u8 ts_clk_gated;

	/* A/D Clock frequency */
	u32 if_clk_freq; /* in kHz */

	/* IF frequency */
	u32 if_freq; /* in kHz */

	/*Use External ADC*/
	u8 ext_adc;

	/*External ADC output two's complement*/
	u8 adc_signed;

	/*Sample IF data at falling edge of IF_CLK*/
	u8 if_neg_edge;

	/*IF use Negative center frequency*/
	u8 if_neg_center;

	/*8G75 internal ADC input range selection*/
	/*0: 0.8Vpp, 1: 1.0Vpp, 2: 1.6Vpp, 3: 2.0Vpp*/
	u8 adc_vpp;

	/* slave address and configuration of the tuner */
	u8 tuner_address;
};

#if defined(CONFIG_DVB_LGS8GXX) || \
	(defined(CONFIG_DVB_LGS8GXX_MODULE) && defined(MODULE))
extern struct dvb_frontend *lgs8gxx_attach(const struct lgs8gxx_config *config,
					   struct i2c_adapter *i2c);
#else
static inline
struct dvb_frontend *lgs8gxx_attach(const struct lgs8gxx_config *config,
				    struct i2c_adapter *i2c) {
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif /* CONFIG_DVB_LGS8GXX */

#endif /* __LGS8GXX_H__ */
