

#ifndef _BT_DRV_H
#define _BT_DRV_H

/* Bluetooth Driver Version */
#define VERSION               "1.0"

#define BT_REGISTER_TIMEOUT   msecs_to_jiffies(6000)	/* 6 sec */

/* BT driver's local status */
#define BT_DRV_RUNNING        0
#define BT_ST_REGISTERED      1

/* BT driver operation structure */
struct hci_st {

	/* hci device pointer which binds to bt driver */
	struct hci_dev *hdev;

	/* used locally,to maintain various BT driver status */
	unsigned long flags;

	/* to hold ST registration callback  status */
	char streg_cbdata;

	/* write function pointer of ST driver */
	long (*st_write) (struct sk_buff *);

	/* Wait on comepletion handler needed to synchronize
	 * hci_st_open() and hci_st_registration_completion_cb()
	 * functions.*/
	struct completion wait_for_btdrv_reg_completion;
};

#endif
