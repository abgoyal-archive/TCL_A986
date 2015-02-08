

#ifndef TPA6130A2_PLAT_H
#define TPA6130A2_PLAT_H

enum tpa_model {
	TPA6130A2,
	TPA6140A2,
};

struct tpa6130a2_platform_data {
	enum tpa_model id;
	int power_gpio;
};

#endif
