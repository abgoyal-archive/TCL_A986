
#ifndef _CHARPIPE_H
#define _CHARPIPE_H

#include "qemu-common.h"

extern int  qemu_chr_open_charpipe( CharDriverState* *pfirst, CharDriverState* *psecond );

extern CharDriverState*  qemu_chr_open_buffer( CharDriverState*  endpoint );

/* must be called from the main event loop to poll all charpipes */
extern void charpipe_poll( void );

#endif /* _CHARPIPE_H */
