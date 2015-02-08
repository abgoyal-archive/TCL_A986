

#ifndef _INC_PMCC4_DEFS_H_
#define _INC_PMCC4_DEFS_H_



#define MAX_BOARDS          8
#define MAX_CHANS_USED      128

#ifdef  SBE_PMCC4_ENABLE
#define MUSYCC_NPORTS       4     /* CN8474 */
#endif
#ifdef SBE_WAN256T3_ENABLE
#define MUSYCC_NPORTS       8     /* CN8478 */
#endif
#define MUSYCC_NCHANS       32    /* actually, chans per port */

#define MUSYCC_NIQD         0x1000    /* power of 2 */
#define MUSYCC_MRU          2048  /* default */
#define MUSYCC_MTU          2048  /* default */
#define MUSYCC_TXDESC_MIN   10    /* HDLC mode default */
#define MUSYCC_RXDESC_MIN   18    /* HDLC mode default */
#define MUSYCC_TXDESC_TRANS 4     /* Transparent mode minumum # of TX descriptors */
#define MUSYCC_RXDESC_TRANS 12    /* Transparent mode minumum # of RX descriptors */

#define MAX_DEFAULT_IFQLEN  32    /* network qlen */


#define SBE_IFACETMPL        "pmcc4-%d"
#ifdef IFNAMSIZ
#define SBE_IFACETMPL_SIZE    IFNAMSIZ
#else
#define SBE_IFACETMPL_SIZE    16
#endif

/* we want the PMCC4 watchdog to fire off every 250ms */
#define WATCHDOG_TIMEOUT      250000

#define WATCHDOG_UTIMEOUT     (WATCHDOG_TIMEOUT+300000)

#if !defined(SBE_ISR_TASKLET) && !defined(SBE_ISR_IMMEDIATE) && !defined(SBE_ISR_INLINE)
#define SBE_ISR_TASKLET
#endif

#endif   /*** _INC_PMCC4_DEFS_H_ ***/

