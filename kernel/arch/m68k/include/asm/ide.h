

/* Copyright(c) 1996 Kars de Jong */
/* Based on the ide driver from 1.2.13pl8 */


#ifndef _M68K_IDE_H
#define _M68K_IDE_H

#ifdef __KERNEL__
#include <asm/setup.h>
#include <asm/io.h>
#include <asm/irq.h>

#undef readb
#undef readw
#undef writeb
#undef writew

#define readb				in_8
#define readw				in_be16
#define __ide_mm_insw(port, addr, n)	raw_insw((u16 *)port, addr, n)
#define __ide_mm_insl(port, addr, n)	raw_insl((u32 *)port, addr, n)
#define writeb(val, port)		out_8(port, val)
#define writew(val, port)		out_be16(port, val)
#define __ide_mm_outsw(port, addr, n)	raw_outsw((u16 *)port, addr, n)
#define __ide_mm_outsl(port, addr, n)	raw_outsl((u32 *)port, addr, n)

#endif /* __KERNEL__ */
#endif /* _M68K_IDE_H */
