

#ifndef __REGULATOR_FIXED_H
#define __REGULATOR_FIXED_H

struct regulator_init_data;

struct fixed_voltage_config {
	const char *supply_name;
	int microvolts;
	int gpio;
	unsigned startup_delay;
	unsigned enable_high:1;
	unsigned enabled_at_boot:1;
	struct regulator_init_data *init_data;
};

#endif
