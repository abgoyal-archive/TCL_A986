
#ifndef _MMC_CORE_HOST_H
#define _MMC_CORE_HOST_H

int mmc_register_host_class(void);
void mmc_unregister_host_class(void);

void mmc_host_deeper_disable(struct work_struct *work);

#endif

