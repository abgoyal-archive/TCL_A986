

#include <linux/module.h>
#include <linux/device.h>
#include <scsi/scsi_scan.h>

static int __init wait_scan_init(void)
{
	/*
	 * First we need to wait for device probing to finish;
	 * the drivers we just loaded might just still be probing
	 * and might not yet have reached the scsi async scanning
	 */
	wait_for_device_probe();
	/*
	 * and then we wait for the actual asynchronous scsi scan
	 * to finish.
	 */
	scsi_complete_async_scans();
	return 0;
}

static void __exit wait_scan_exit(void)
{
}

MODULE_DESCRIPTION("SCSI wait for scans");
MODULE_AUTHOR("James Bottomley");
MODULE_LICENSE("GPL");

late_initcall(wait_scan_init);
module_exit(wait_scan_exit);
