

#ifndef __LINUX_REGULATOR_DRIVER_H_
#define __LINUX_REGULATOR_DRIVER_H_

#include <linux/device.h>
#include <linux/regulator/consumer.h>

struct regulator_dev;
struct regulator_init_data;

enum regulator_status {
	REGULATOR_STATUS_OFF,
	REGULATOR_STATUS_ON,
	REGULATOR_STATUS_ERROR,
	/* fast/normal/idle/standby are flavors of "on" */
	REGULATOR_STATUS_FAST,
	REGULATOR_STATUS_NORMAL,
	REGULATOR_STATUS_IDLE,
	REGULATOR_STATUS_STANDBY,
};

struct regulator_ops {

	/* enumerate supported voltages */
	int (*list_voltage) (struct regulator_dev *, unsigned selector);

	/* get/set regulator voltage */
	int (*set_voltage) (struct regulator_dev *, int min_uV, int max_uV);
	int (*get_voltage) (struct regulator_dev *);

	/* get/set regulator current  */
	int (*set_current_limit) (struct regulator_dev *,
				 int min_uA, int max_uA);
	int (*get_current_limit) (struct regulator_dev *);

	/* enable/disable regulator */
	int (*enable) (struct regulator_dev *);
	int (*disable) (struct regulator_dev *);
	int (*is_enabled) (struct regulator_dev *);

	/* get/set regulator operating mode (defined in regulator.h) */
	int (*set_mode) (struct regulator_dev *, unsigned int mode);
	unsigned int (*get_mode) (struct regulator_dev *);

	/* Time taken to enable the regulator */
	int (*enable_time) (struct regulator_dev *);

	/* report regulator status ... most other accessors report
	 * control inputs, this reports results of combining inputs
	 * from Linux (and other sources) with the actual load.
	 * returns REGULATOR_STATUS_* or negative errno.
	 */
	int (*get_status)(struct regulator_dev *);

	/* get most efficient regulator operating mode for load */
	unsigned int (*get_optimum_mode) (struct regulator_dev *, int input_uV,
					  int output_uV, int load_uA);

	/* the operations below are for configuration of regulator state when
	 * its parent PMIC enters a global STANDBY/HIBERNATE state */

	/* set regulator suspend voltage */
	int (*set_suspend_voltage) (struct regulator_dev *, int uV);

	/* enable/disable regulator in suspend state */
	int (*set_suspend_enable) (struct regulator_dev *);
	int (*set_suspend_disable) (struct regulator_dev *);

	/* set regulator suspend operating mode (defined in regulator.h) */
	int (*set_suspend_mode) (struct regulator_dev *, unsigned int mode);
};

enum regulator_type {
	REGULATOR_VOLTAGE,
	REGULATOR_CURRENT,
};

struct regulator_desc {
	const char *name;
	int id;
	unsigned n_voltages;
	struct regulator_ops *ops;
	int irq;
	enum regulator_type type;
	struct module *owner;
};

struct regulator_dev {
	struct regulator_desc *desc;
	int use_count;
	int open_count;
	int exclusive;

	/* lists we belong to */
	struct list_head list; /* list of all regulators */
	struct list_head slist; /* list of supplied regulators */

	/* lists we own */
	struct list_head consumer_list; /* consumers we supply */
	struct list_head supply_list; /* regulators we supply */

	struct blocking_notifier_head notifier;
	struct mutex mutex; /* consumer lock */
	struct module *owner;
	struct device dev;
	struct regulation_constraints *constraints;
	struct regulator_dev *supply;	/* for tree */

	void *reg_data;		/* regulator_dev data */
};

struct regulator_dev *regulator_register(struct regulator_desc *regulator_desc,
	struct device *dev, struct regulator_init_data *init_data,
	void *driver_data);
void regulator_unregister(struct regulator_dev *rdev);

int regulator_notifier_call_chain(struct regulator_dev *rdev,
				  unsigned long event, void *data);

void *rdev_get_drvdata(struct regulator_dev *rdev);
struct device *rdev_get_dev(struct regulator_dev *rdev);
int rdev_get_id(struct regulator_dev *rdev);

int regulator_mode_to_status(unsigned int);

void *regulator_get_init_drvdata(struct regulator_init_data *reg_init_data);

#endif
