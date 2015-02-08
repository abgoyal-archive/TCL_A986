

#ifndef __LINUX_REGULATOR_MACHINE_H_
#define __LINUX_REGULATOR_MACHINE_H_

#include <linux/regulator/consumer.h>
#include <linux/suspend.h>

struct regulator;


#define REGULATOR_CHANGE_VOLTAGE	0x1
#define REGULATOR_CHANGE_CURRENT	0x2
#define REGULATOR_CHANGE_MODE		0x4
#define REGULATOR_CHANGE_STATUS		0x8
#define REGULATOR_CHANGE_DRMS		0x10

struct regulator_state {
	int uV;	/* suspend voltage */
	unsigned int mode; /* suspend regulator operating mode */
	int enabled; /* is regulator enabled in this suspend state */
	int disabled; /* is the regulator disbled in this suspend state */
};

struct regulation_constraints {

	char *name;

	/* voltage output range (inclusive) - for voltage control */
	int min_uV;
	int max_uV;

	/* current output range (inclusive) - for current control */
	int min_uA;
	int max_uA;

	/* valid regulator operating modes for this machine */
	unsigned int valid_modes_mask;

	/* valid operations for regulator on this machine */
	unsigned int valid_ops_mask;

	/* regulator input voltage - only if supply is another regulator */
	int input_uV;

	/* regulator suspend states for global PMIC STANDBY/HIBERNATE */
	struct regulator_state state_disk;
	struct regulator_state state_mem;
	struct regulator_state state_standby;
	suspend_state_t initial_state; /* suspend state to set at init */

	/* mode to set on startup */
	unsigned int initial_mode;

	/* constraint flags */
	unsigned always_on:1;	/* regulator never off when system is on */
	unsigned boot_on:1;	/* bootloader/firmware enabled regulator */
	unsigned apply_uV:1;	/* apply uV constraint if min == max */
};

struct regulator_consumer_supply {
	struct device *dev;	/* consumer */
	const char *dev_name;   /* dev_name() for consumer */
	const char *supply;	/* consumer supply - e.g. "vcc" */
};

/* Initialize struct regulator_consumer_supply */
#define REGULATOR_SUPPLY(_name, _dev_name)			\
{								\
	.supply		= _name,				\
	.dev_name	= _dev_name,				\
}

struct regulator_init_data {
	const char *supply_regulator;        /* or NULL for system supply */
	struct device *supply_regulator_dev; /* or NULL for system supply */

	struct regulation_constraints constraints;

	int num_consumer_supplies;
	struct regulator_consumer_supply *consumer_supplies;

	/* optional regulator machine specific init */
	int (*regulator_init)(void *driver_data);
	void *driver_data;	/* core does not touch this */
};

int regulator_suspend_prepare(suspend_state_t state);

#ifdef CONFIG_REGULATOR
void regulator_has_full_constraints(void);
#else
static inline void regulator_has_full_constraints(void)
{
}
#endif

#endif
