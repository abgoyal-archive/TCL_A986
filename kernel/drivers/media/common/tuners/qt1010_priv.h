

#ifndef QT1010_PRIV_H
#define QT1010_PRIV_H


#define QT1010_STEP         125000 /*  125 kHz used by Windows drivers,
				      hw could be more precise but we don't
				      know how to use */
#define QT1010_MIN_FREQ   48000000 /*   48 MHz */
#define QT1010_MAX_FREQ  860000000 /*  860 MHz */
#define QT1010_OFFSET   1246000000 /* 1246 MHz */

#define QT1010_WR 0
#define QT1010_RD 1
#define QT1010_M1 3

typedef struct {
	u8 oper, reg, val;
} qt1010_i2c_oper_t;

struct qt1010_priv {
	struct qt1010_config *cfg;
	struct i2c_adapter   *i2c;

	u8 reg1f_init_val;
	u8 reg20_init_val;
	u8 reg25_init_val;

	u32 frequency;
	u32 bandwidth;
};

#endif
