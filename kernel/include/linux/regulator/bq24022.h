

struct regulator_init_data;

struct bq24022_mach_info {
	int gpio_nce;
	int gpio_iset2;
	struct regulator_init_data *init_data;
};
