

#include "usb.h"
#include "transport.h"

int usb_stor_euscsi_init(struct us_data *us);

int usb_stor_ucr61s2b_init(struct us_data *us);

/* This places the HUAWEI E220 devices in multi-port mode */
int usb_stor_huawei_e220_init(struct us_data *us);
