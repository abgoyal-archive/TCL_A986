

#ifndef VA1J5JF8007T_H
#define VA1J5JF8007T_H

enum va1j5jf8007t_frequency {
	VA1J5JF8007T_20MHZ,
	VA1J5JF8007T_25MHZ,
};

struct va1j5jf8007t_config {
	u8 demod_address;
	enum va1j5jf8007t_frequency frequency;
};

struct i2c_adapter;

struct dvb_frontend *
va1j5jf8007t_attach(const struct va1j5jf8007t_config *config,
		    struct i2c_adapter *adap);

/* must be called after va1j5jf8007s_attach */
int va1j5jf8007t_prepare(struct dvb_frontend *fe);

#endif
