

#ifndef TDHD1_H
#define TDHD1_H

#include "tda1004x.h"

static int alps_tdhd1_204_request_firmware(struct dvb_frontend *fe, const struct firmware **fw, char *name);

static struct tda1004x_config alps_tdhd1_204a_config = {
	.demod_address = 0x8,
	.invert = 1,
	.invert_oclk = 0,
	.xtal_freq = TDA10046_XTAL_4M,
	.agc_config = TDA10046_AGC_DEFAULT,
	.if_freq = TDA10046_FREQ_3617,
	.request_firmware = alps_tdhd1_204_request_firmware
};

static int alps_tdhd1_204a_tuner_set_params(struct dvb_frontend *fe, struct dvb_frontend_parameters *params)
{
	struct i2c_adapter *i2c = fe->tuner_priv;
	u8 data[4];
	struct i2c_msg msg = { .addr = 0x61, .flags = 0, .buf = data, .len = sizeof(data) };
	u32 div;

	div = (params->frequency + 36166666) / 166666;

	data[0] = (div >> 8) & 0x7f;
	data[1] = div & 0xff;
	data[2] = 0x85;

	if (params->frequency >= 174000000 && params->frequency <= 230000000)
		data[3] = 0x02;
	else if (params->frequency >= 470000000 && params->frequency <= 823000000)
		data[3] = 0x0C;
	else if (params->frequency > 823000000 && params->frequency <= 862000000)
		data[3] = 0x8C;
	else
		return -EINVAL;

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);
	if (i2c_transfer(i2c, &msg, 1) != 1)
		return -EIO;

	return 0;
}

#endif /* TDHD1_H */
