

#ifndef EFX_MAC_H
#define EFX_MAC_H

#include "net_driver.h"

extern struct efx_mac_operations falcon_gmac_operations;
extern struct efx_mac_operations falcon_xmac_operations;
extern struct efx_mac_operations efx_mcdi_mac_operations;
extern void falcon_reconfigure_xmac_core(struct efx_nic *efx);
extern int efx_mcdi_mac_stats(struct efx_nic *efx, dma_addr_t dma_addr,
			      u32 dma_len, int enable, int clear);

#endif
