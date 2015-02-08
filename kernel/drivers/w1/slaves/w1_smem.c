

#include <asm/types.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/types.h>

#include "../w1.h"
#include "../w1_int.h"
#include "../w1_family.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Evgeniy Polyakov <johnpol@2ka.mipt.ru>");
MODULE_DESCRIPTION("Driver for 1-wire Dallas network protocol, 64bit memory family.");

static struct w1_family w1_smem_family_01 = {
	.fid = W1_FAMILY_SMEM_01,
};

static struct w1_family w1_smem_family_81 = {
	.fid = W1_FAMILY_SMEM_81,
};

static int __init w1_smem_init(void)
{
	int err;

	err = w1_register_family(&w1_smem_family_01);
	if (err)
		return err;

	err = w1_register_family(&w1_smem_family_81);
	if (err) {
		w1_unregister_family(&w1_smem_family_01);
		return err;
	}

	return 0;
}

static void __exit w1_smem_fini(void)
{
	w1_unregister_family(&w1_smem_family_01);
	w1_unregister_family(&w1_smem_family_81);
}

module_init(w1_smem_init);
module_exit(w1_smem_fini);
