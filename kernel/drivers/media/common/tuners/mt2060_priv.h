

#ifndef MT2060_PRIV_H
#define MT2060_PRIV_H

// Uncomment the #define below to enable spurs checking. The results where quite unconvincing.
// #define MT2060_SPURCHECK


#define I2C_ADDRESS 0x60

#define REG_PART_REV   0
#define REG_LO1C1      1
#define REG_LO1C2      2
#define REG_LO2C1      3
#define REG_LO2C2      4
#define REG_LO2C3      5
#define REG_LO_STATUS  6
#define REG_FM_FREQ    7
#define REG_MISC_STAT  8
#define REG_MISC_CTRL  9
#define REG_RESERVED_A 0x0A
#define REG_VGAG       0x0B
#define REG_LO1B1      0x0C
#define REG_LO1B2      0x0D
#define REG_LOTO       0x11

#define PART_REV 0x63 // The current driver works only with PART=6 and REV=3 chips

struct mt2060_priv {
	struct mt2060_config *cfg;
	struct i2c_adapter   *i2c;

	u32 frequency;
	u32 bandwidth;
	u16 if1_freq;
	u8  fmfreq;
};

#endif
