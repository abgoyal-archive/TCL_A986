
#define PN544_MAGIC 0xE9

#define PN544_SET_PWR _IOW(0xE9, 0x01, unsigned int)

struct pn544_i2c_platform_data {
 unsigned int irq_gpio;
 unsigned int ven_gpio;
 unsigned int firm_gpio;
};

