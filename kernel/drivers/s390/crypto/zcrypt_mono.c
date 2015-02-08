

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/compat.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>

#include "ap_bus.h"
#include "zcrypt_api.h"
#include "zcrypt_pcica.h"
#include "zcrypt_pcicc.h"
#include "zcrypt_pcixcc.h"
#include "zcrypt_cex2a.h"

static int __init zcrypt_init(void)
{
	int rc;

	rc = ap_module_init();
	if (rc)
		goto out;
	rc = zcrypt_api_init();
	if (rc)
		goto out_ap;
	rc = zcrypt_pcica_init();
	if (rc)
		goto out_api;
	rc = zcrypt_pcicc_init();
	if (rc)
		goto out_pcica;
	rc = zcrypt_pcixcc_init();
	if (rc)
		goto out_pcicc;
	rc = zcrypt_cex2a_init();
	if (rc)
		goto out_pcixcc;
	return 0;

out_pcixcc:
	zcrypt_pcixcc_exit();
out_pcicc:
	zcrypt_pcicc_exit();
out_pcica:
	zcrypt_pcica_exit();
out_api:
	zcrypt_api_exit();
out_ap:
	ap_module_exit();
out:
	return rc;
}

static void __exit zcrypt_exit(void)
{
	zcrypt_cex2a_exit();
	zcrypt_pcixcc_exit();
	zcrypt_pcicc_exit();
	zcrypt_pcica_exit();
	zcrypt_api_exit();
	ap_module_exit();
}

module_init(zcrypt_init);
module_exit(zcrypt_exit);
