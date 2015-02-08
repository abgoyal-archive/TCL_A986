
#ifndef _QEMU_TCPDUMP_H
#define _QEMU_TCPDUMP_H

#include <stdint.h>

/* global flag, set to 1 when packet captupe is active */
extern int  qemu_tcpdump_active;

extern int  qemu_tcpdump_start( const char*  filepath );

/* stop the current packet capture, if any */
extern void qemu_tcpdump_stop( void );

/* send an ethernet packet to the packet capture file, if any */
extern void qemu_tcpdump_packet( const void*  base, int  len );

extern void  qemu_tcpdump_stats( uint64_t  *pcount, uint64_t*  psize );

#endif /* _QEMU_TCPDUMP_H */
