

#ifndef __ETHERNET_DEFINES_H__
#define __ETHERNET_DEFINES_H__

#include "cvmx-config.h"


#define OCTEON_ETHERNET_VERSION "1.9"

#ifndef CONFIG_CAVIUM_RESERVE32
#define CONFIG_CAVIUM_RESERVE32 0
#endif

#define USE_SKBUFFS_IN_HW           1
#ifdef CONFIG_NETFILTER
#define REUSE_SKBUFFS_WITHOUT_FREE  0
#else
#define REUSE_SKBUFFS_WITHOUT_FREE  1
#endif

#define USE_HW_TCPUDP_CHECKSUM      1

/* Enable Random Early Dropping under load */
#define USE_RED                     1
#define USE_ASYNC_IOBDMA            (CONFIG_CAVIUM_OCTEON_CVMSEG_SIZE > 0)

#define USE_10MBPS_PREAMBLE_WORKAROUND 1
#define DONT_WRITEBACK(x)           (x)
/* Use this to not have FPA frees control L2 */
/*#define DONT_WRITEBACK(x)         0   */

/* Maximum number of SKBs to try to free per xmit packet. */
#define MAX_OUT_QUEUE_DEPTH 1000

#define FAU_TOTAL_TX_TO_CLEAN (CVMX_FAU_REG_END - sizeof(uint32_t))
#define FAU_NUM_PACKET_BUFFERS_TO_FREE (FAU_TOTAL_TX_TO_CLEAN - sizeof(uint32_t))

#define TOTAL_NUMBER_OF_PORTS       (CVMX_PIP_NUM_INPUT_PORTS+1)


#endif /* __ETHERNET_DEFINES_H__ */
