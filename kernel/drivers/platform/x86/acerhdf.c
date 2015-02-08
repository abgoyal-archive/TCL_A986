

#define pr_fmt(fmt) "acerhdf: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/dmi.h>
#include <acpi/acpi_drivers.h>
#include <linux/sched.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>

#undef START_IN_KERNEL_MODE

#define DRV_VER "0.5.22"

#define ACERHDF_TEMP_CRIT 89000
#define ACERHDF_FAN_OFF 0
#define ACERHDF_FAN_AUTO 1

#define ACERHDF_MAX_FANON 80000

#define ACERHDF_MAX_INTERVAL 15

#ifdef START_IN_KERNEL_MODE
static int kernelmode = 1;
#else
static int kernelmode;
#endif

static unsigned int interval = 10;
static unsigned int fanon = 63000;
static unsigned int fanoff = 58000;
static unsigned int verbose;
static unsigned int fanstate = ACERHDF_FAN_AUTO;
static char force_bios[16];
static char force_product[16];
static unsigned int prev_interval;
struct thermal_zone_device *thz_dev;
struct thermal_cooling_device *cl_dev;
struct platform_device *acerhdf_dev;

module_param(kernelmode, uint, 0);
MODULE_PARM_DESC(kernelmode, "Kernel mode fan control on / off");
module_param(interval, uint, 0600);
MODULE_PARM_DESC(interval, "Polling interval of temperature check");
module_param(fanon, uint, 0600);
MODULE_PARM_DESC(fanon, "Turn the fan on above this temperature");
module_param(fanoff, uint, 0600);
MODULE_PARM_DESC(fanoff, "Turn the fan off below this temperature");
module_param(verbose, uint, 0600);
MODULE_PARM_DESC(verbose, "Enable verbose dmesg output");
module_param_string(force_bios, force_bios, 16, 0);
MODULE_PARM_DESC(force_bios, "Force BIOS version and omit BIOS check");
module_param_string(force_product, force_product, 16, 0);
MODULE_PARM_DESC(force_product, "Force BIOS product and omit BIOS check");

struct fancmd {
	u8 cmd_off;
	u8 chk_off;
	u8 cmd_auto;
};

/* BIOS settings */
struct bios_settings_t {
	const char *vendor;
	const char *product;
	const char *version;
	unsigned char fanreg;
	unsigned char tempreg;
	struct fancmd cmd;
};

/* Register addresses and values for different BIOS versions */
static const struct bios_settings_t bios_tbl[] = {
	/* AOA110 */
	{"Acer", "AOA110", "v0.3109", 0x55, 0x58, {0x1f, 0x1f, 0x00} },
	{"Acer", "AOA110", "v0.3114", 0x55, 0x58, {0x1f, 0x1f, 0x00} },
	{"Acer", "AOA110", "v0.3301", 0x55, 0x58, {0xaf, 0xaf, 0x00} },
	{"Acer", "AOA110", "v0.3304", 0x55, 0x58, {0xaf, 0xaf, 0x00} },
	{"Acer", "AOA110", "v0.3305", 0x55, 0x58, {0xaf, 0xaf, 0x00} },
	{"Acer", "AOA110", "v0.3307", 0x55, 0x58, {0xaf, 0xaf, 0x00} },
	{"Acer", "AOA110", "v0.3308", 0x55, 0x58, {0x21, 0x21, 0x00} },
	{"Acer", "AOA110", "v0.3309", 0x55, 0x58, {0x21, 0x21, 0x00} },
	{"Acer", "AOA110", "v0.3310", 0x55, 0x58, {0x21, 0x21, 0x00} },
	/* AOA150 */
	{"Acer", "AOA150", "v0.3114", 0x55, 0x58, {0x20, 0x20, 0x00} },
	{"Acer", "AOA150", "v0.3301", 0x55, 0x58, {0x20, 0x20, 0x00} },
	{"Acer", "AOA150", "v0.3304", 0x55, 0x58, {0x20, 0x20, 0x00} },
	{"Acer", "AOA150", "v0.3305", 0x55, 0x58, {0x20, 0x20, 0x00} },
	{"Acer", "AOA150", "v0.3307", 0x55, 0x58, {0x20, 0x20, 0x00} },
	{"Acer", "AOA150", "v0.3308", 0x55, 0x58, {0x20, 0x20, 0x00} },
	{"Acer", "AOA150", "v0.3309", 0x55, 0x58, {0x20, 0x20, 0x00} },
	{"Acer", "AOA150", "v0.3310", 0x55, 0x58, {0x20, 0x20, 0x00} },
	/* Acer 1410 */
	{"Acer", "Aspire 1410", "v0.3120", 0x55, 0x58, {0x9e, 0x9e, 0x00} },
	{"Acer", "Aspire 1410", "v1.3303", 0x55, 0x58, {0x9e, 0x9e, 0x00} },
	/* Acer 1810xx */
	{"Acer", "Aspire 1810TZ", "v0.3120", 0x55, 0x58, {0x9e, 0x9e, 0x00} },
	{"Acer", "Aspire 1810T", "v0.3120", 0x55, 0x58, {0x9e, 0x9e, 0x00} },
	{"Acer", "Aspire 1810T", "v1.3303", 0x55, 0x58, {0x9e, 0x9e, 0x00} },
	{"Acer", "Aspire 1810TZ", "v1.3303", 0x55, 0x58, {0x9e, 0x9e, 0x00} },
	/* Gateway */
	{"Gateway", "AOA110", "v0.3103", 0x55, 0x58, {0x21, 0x21, 0x00} },
	{"Gateway", "AOA150", "v0.3103", 0x55, 0x58, {0x20, 0x20, 0x00} },
	{"Gateway", "LT31", "v1.3103", 0x55, 0x58, {0x10, 0x0f, 0x00} },
	{"Gateway", "LT31", "v1.3201", 0x55, 0x58, {0x10, 0x0f, 0x00} },
	{"Gateway", "LT31", "v1.3302", 0x55, 0x58, {0x10, 0x0f, 0x00} },
	/* Packard Bell */
	{"Packard Bell", "DOA150", "v0.3104", 0x55, 0x58, {0x21, 0x21, 0x00} },
	{"Packard Bell", "DOA150", "v0.3105", 0x55, 0x58, {0x20, 0x20, 0x00} },
	{"Packard Bell", "AOA110", "v0.3105", 0x55, 0x58, {0x21, 0x21, 0x00} },
	{"Packard Bell", "AOA150", "v0.3105", 0x55, 0x58, {0x20, 0x20, 0x00} },
	{"Packard Bell", "DOTMU", "v1.3303", 0x55, 0x58, {0x9e, 0x9e, 0x00} },
	{"Packard Bell", "DOTMU", "v0.3120", 0x55, 0x58, {0x9e, 0x9e, 0x00} },
	/* pewpew-terminator */
	{"", "", "", 0, 0, {0, 0, 0} }
};

static const struct bios_settings_t *bios_cfg __read_mostly;

static int acerhdf_get_temp(int *temp)
{
	u8 read_temp;

	if (ec_read(bios_cfg->tempreg, &read_temp))
		return -EINVAL;

	*temp = read_temp * 1000;

	return 0;
}

static int acerhdf_get_fanstate(int *state)
{
	u8 fan;

	if (ec_read(bios_cfg->fanreg, &fan))
		return -EINVAL;

	if (fan != bios_cfg->cmd.chk_off)
		*state = ACERHDF_FAN_AUTO;
	else
		*state = ACERHDF_FAN_OFF;

	return 0;
}

static void acerhdf_change_fanstate(int state)
{
	unsigned char cmd;

	if (verbose)
		pr_notice("fan %s\n", (state == ACERHDF_FAN_OFF) ?
				"OFF" : "ON");

	if ((state != ACERHDF_FAN_OFF) && (state != ACERHDF_FAN_AUTO)) {
		pr_err("invalid fan state %d requested, setting to auto!\n",
			state);
		state = ACERHDF_FAN_AUTO;
	}

	cmd = (state == ACERHDF_FAN_OFF) ? bios_cfg->cmd.cmd_off
					 : bios_cfg->cmd.cmd_auto;
	fanstate = state;

	ec_write(bios_cfg->fanreg, cmd);
}

static void acerhdf_check_param(struct thermal_zone_device *thermal)
{
	if (fanon > ACERHDF_MAX_FANON) {
		pr_err("fanon temperature too high, set to %d\n",
				ACERHDF_MAX_FANON);
		fanon = ACERHDF_MAX_FANON;
	}

	if (kernelmode && prev_interval != interval) {
		if (interval > ACERHDF_MAX_INTERVAL) {
			pr_err("interval too high, set to %d\n",
				ACERHDF_MAX_INTERVAL);
			interval = ACERHDF_MAX_INTERVAL;
		}
		if (verbose)
			pr_notice("interval changed to: %d\n",
					interval);
		thermal->polling_delay = interval*1000;
		prev_interval = interval;
	}
}

static int acerhdf_get_ec_temp(struct thermal_zone_device *thermal,
			       unsigned long *t)
{
	int temp, err = 0;

	acerhdf_check_param(thermal);

	err = acerhdf_get_temp(&temp);
	if (err)
		return err;

	if (verbose)
		pr_notice("temp %d\n", temp);

	*t = temp;
	return 0;
}

static int acerhdf_bind(struct thermal_zone_device *thermal,
			struct thermal_cooling_device *cdev)
{
	/* if the cooling device is the one from acerhdf bind it */
	if (cdev != cl_dev)
		return 0;

	if (thermal_zone_bind_cooling_device(thermal, 0, cdev)) {
		pr_err("error binding cooling dev\n");
		return -EINVAL;
	}
	return 0;
}

static int acerhdf_unbind(struct thermal_zone_device *thermal,
			  struct thermal_cooling_device *cdev)
{
	if (cdev != cl_dev)
		return 0;

	if (thermal_zone_unbind_cooling_device(thermal, 0, cdev)) {
		pr_err("error unbinding cooling dev\n");
		return -EINVAL;
	}
	return 0;
}

static inline void acerhdf_revert_to_bios_mode(void)
{
	acerhdf_change_fanstate(ACERHDF_FAN_AUTO);
	kernelmode = 0;
	if (thz_dev)
		thz_dev->polling_delay = 0;
	pr_notice("kernel mode fan control OFF\n");
}
static inline void acerhdf_enable_kernelmode(void)
{
	kernelmode = 1;

	thz_dev->polling_delay = interval*1000;
	thermal_zone_device_update(thz_dev);
	pr_notice("kernel mode fan control ON\n");
}

static int acerhdf_get_mode(struct thermal_zone_device *thermal,
			    enum thermal_device_mode *mode)
{
	if (verbose)
		pr_notice("kernel mode fan control %d\n", kernelmode);

	*mode = (kernelmode) ? THERMAL_DEVICE_ENABLED
			     : THERMAL_DEVICE_DISABLED;

	return 0;
}

static int acerhdf_set_mode(struct thermal_zone_device *thermal,
			    enum thermal_device_mode mode)
{
	if (mode == THERMAL_DEVICE_DISABLED && kernelmode)
		acerhdf_revert_to_bios_mode();
	else if (mode == THERMAL_DEVICE_ENABLED && !kernelmode)
		acerhdf_enable_kernelmode();

	return 0;
}

static int acerhdf_get_trip_type(struct thermal_zone_device *thermal, int trip,
				 enum thermal_trip_type *type)
{
	if (trip == 0)
		*type = THERMAL_TRIP_ACTIVE;

	return 0;
}

static int acerhdf_get_trip_temp(struct thermal_zone_device *thermal, int trip,
				 unsigned long *temp)
{
	if (trip == 0)
		*temp = fanon;

	return 0;
}

static int acerhdf_get_crit_temp(struct thermal_zone_device *thermal,
				 unsigned long *temperature)
{
	*temperature = ACERHDF_TEMP_CRIT;
	return 0;
}

/* bind callback functions to thermalzone */
struct thermal_zone_device_ops acerhdf_dev_ops = {
	.bind = acerhdf_bind,
	.unbind = acerhdf_unbind,
	.get_temp = acerhdf_get_ec_temp,
	.get_mode = acerhdf_get_mode,
	.set_mode = acerhdf_set_mode,
	.get_trip_type = acerhdf_get_trip_type,
	.get_trip_temp = acerhdf_get_trip_temp,
	.get_crit_temp = acerhdf_get_crit_temp,
};


static int acerhdf_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	*state = 1;

	return 0;
}

static int acerhdf_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	int err = 0, tmp;

	err = acerhdf_get_fanstate(&tmp);
	if (err)
		return err;

	*state = (tmp == ACERHDF_FAN_AUTO) ? 1 : 0;
	return 0;
}

/* change current fan state - is overwritten when running in kernel mode */
static int acerhdf_set_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long state)
{
	int cur_temp, cur_state, err = 0;

	if (!kernelmode)
		return 0;

	err = acerhdf_get_temp(&cur_temp);
	if (err) {
		pr_err("error reading temperature, hand off control to BIOS\n");
		goto err_out;
	}

	err = acerhdf_get_fanstate(&cur_state);
	if (err) {
		pr_err("error reading fan state, hand off control to BIOS\n");
		goto err_out;
	}

	if (state == 0) {
		/* turn fan off only if below fanoff temperature */
		if ((cur_state == ACERHDF_FAN_AUTO) &&
		    (cur_temp < fanoff))
			acerhdf_change_fanstate(ACERHDF_FAN_OFF);
	} else {
		if (cur_state == ACERHDF_FAN_OFF)
			acerhdf_change_fanstate(ACERHDF_FAN_AUTO);
	}
	return 0;

err_out:
	acerhdf_revert_to_bios_mode();
	return -EINVAL;
}

/* bind fan callbacks to fan device */
struct thermal_cooling_device_ops acerhdf_cooling_ops = {
	.get_max_state = acerhdf_get_max_state,
	.get_cur_state = acerhdf_get_cur_state,
	.set_cur_state = acerhdf_set_cur_state,
};

/* suspend / resume functionality */
static int acerhdf_suspend(struct device *dev)
{
	if (kernelmode)
		acerhdf_change_fanstate(ACERHDF_FAN_AUTO);

	if (verbose)
		pr_notice("going suspend\n");

	return 0;
}

static int __devinit acerhdf_probe(struct platform_device *device)
{
	return 0;
}

static int acerhdf_remove(struct platform_device *device)
{
	return 0;
}

static const struct dev_pm_ops acerhdf_pm_ops = {
	.suspend = acerhdf_suspend,
	.freeze  = acerhdf_suspend,
};

static struct platform_driver acerhdf_driver = {
	.driver = {
		.name  = "acerhdf",
		.owner = THIS_MODULE,
		.pm    = &acerhdf_pm_ops,
	},
	.probe = acerhdf_probe,
	.remove = acerhdf_remove,
};

/* checks if str begins with start */
static int str_starts_with(const char *str, const char *start)
{
	unsigned long str_len = 0, start_len = 0;

	str_len = strlen(str);
	start_len = strlen(start);

	if (str_len >= start_len &&
			!strncmp(str, start, start_len))
		return 1;

	return 0;
}

/* check hardware */
static int acerhdf_check_hardware(void)
{
	char const *vendor, *version, *product;
	const struct bios_settings_t *bt = NULL;

	/* get BIOS data */
	vendor  = dmi_get_system_info(DMI_SYS_VENDOR);
	version = dmi_get_system_info(DMI_BIOS_VERSION);
	product = dmi_get_system_info(DMI_PRODUCT_NAME);


	pr_info("Acer Aspire One Fan driver, v.%s\n", DRV_VER);

	if (force_bios[0]) {
		version = force_bios;
		pr_info("forcing BIOS version: %s\n", version);
		kernelmode = 0;
	}

	if (force_product[0]) {
		product = force_product;
		pr_info("forcing BIOS product: %s\n", product);
		kernelmode = 0;
	}

	if (verbose)
		pr_info("BIOS info: %s %s, product: %s\n",
			vendor, version, product);

	/* search BIOS version and vendor in BIOS settings table */
	for (bt = bios_tbl; bt->vendor[0]; bt++) {
		/*
		 * check if actual hardware BIOS vendor, product and version
		 * IDs start with the strings of BIOS table entry
		 */
		if (str_starts_with(vendor, bt->vendor) &&
				str_starts_with(product, bt->product) &&
				str_starts_with(version, bt->version)) {
			bios_cfg = bt;
			break;
		}
	}

	if (!bios_cfg) {
		pr_err("unknown (unsupported) BIOS version %s/%s/%s, "
			"please report, aborting!\n", vendor, product, version);
		return -EINVAL;
	}

	/*
	 * if started with kernel mode off, prevent the kernel from switching
	 * off the fan
	 */
	if (!kernelmode) {
		pr_notice("Fan control off, to enable do:\n");
		pr_notice("echo -n \"enabled\" > "
			"/sys/class/thermal/thermal_zone0/mode\n");
	}

	return 0;
}

static int acerhdf_register_platform(void)
{
	int err = 0;

	err = platform_driver_register(&acerhdf_driver);
	if (err)
		return err;

	acerhdf_dev = platform_device_alloc("acerhdf", -1);
	platform_device_add(acerhdf_dev);

	return 0;
}

static void acerhdf_unregister_platform(void)
{
	if (!acerhdf_dev)
		return;

	platform_device_del(acerhdf_dev);
	platform_driver_unregister(&acerhdf_driver);
}

static int acerhdf_register_thermal(void)
{
	cl_dev = thermal_cooling_device_register("acerhdf-fan", NULL,
						 &acerhdf_cooling_ops);

	if (IS_ERR(cl_dev))
		return -EINVAL;

	thz_dev = thermal_zone_device_register("acerhdf", 1, NULL,
					      &acerhdf_dev_ops, 0, 0, 0,
					      (kernelmode) ? interval*1000 : 0);
	if (IS_ERR(thz_dev))
		return -EINVAL;

	return 0;
}

static void acerhdf_unregister_thermal(void)
{
	if (cl_dev) {
		thermal_cooling_device_unregister(cl_dev);
		cl_dev = NULL;
	}

	if (thz_dev) {
		thermal_zone_device_unregister(thz_dev);
		thz_dev = NULL;
	}
}

static int __init acerhdf_init(void)
{
	int err = 0;

	err = acerhdf_check_hardware();
	if (err)
		goto out_err;

	err = acerhdf_register_platform();
	if (err)
		goto err_unreg;

	err = acerhdf_register_thermal();
	if (err)
		goto err_unreg;

	return 0;

err_unreg:
	acerhdf_unregister_thermal();
	acerhdf_unregister_platform();

out_err:
	return -ENODEV;
}

static void __exit acerhdf_exit(void)
{
	acerhdf_change_fanstate(ACERHDF_FAN_AUTO);
	acerhdf_unregister_thermal();
	acerhdf_unregister_platform();
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter Feuerer");
MODULE_DESCRIPTION("Aspire One temperature and fan driver");
MODULE_ALIAS("dmi:*:*Acer*:pnAOA*:");
MODULE_ALIAS("dmi:*:*Acer*:pnAspire 1410*:");
MODULE_ALIAS("dmi:*:*Acer*:pnAspire 1810*:");
MODULE_ALIAS("dmi:*:*Gateway*:pnAOA*:");
MODULE_ALIAS("dmi:*:*Gateway*:pnLT31*:");
MODULE_ALIAS("dmi:*:*Packard Bell*:pnAOA*:");
MODULE_ALIAS("dmi:*:*Packard Bell*:pnDOA*:");
MODULE_ALIAS("dmi:*:*Packard Bell*:pnDOTMU*:");

module_init(acerhdf_init);
module_exit(acerhdf_exit);
