

#ifndef VA1J5JF8007S_H
#define VA1J5JF8007S_H

enum va1j5jf8007s_frequency {
	VA1J5JF8007S_20MHZ,
	VA1J5JF8007S_25MHZ,
};

struct va1j5jf8007s_config {
	u8 demod_address;
	enum va1j5jf8007s_frequency frequency;
};

struct i2c_adapter;

struct dvb_frontend *
va1j5jf8007s_attach(const struct va1j5jf8007s_config *config,
		    struct i2c_adapter *adap);

/* must be called after va1j5jf8007t_attach */
int va1j5jf8007s_prepare(struct dvb_frontend *fe);

#endif
