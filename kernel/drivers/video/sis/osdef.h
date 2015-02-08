
/* $XFree86$ */
/* $XdotOrg$ */

#ifndef _SIS_OSDEF_H_
#define _SIS_OSDEF_H_

/* The choices are: */
#define SIS_LINUX_KERNEL		/* Linux kernel framebuffer */
#undef  SIS_XORG_XF86			/* XFree86/X.org */

#ifdef OutPortByte
#undef OutPortByte
#endif

#ifdef OutPortWord
#undef OutPortWord
#endif

#ifdef OutPortLong
#undef OutPortLong
#endif

#ifdef InPortByte
#undef InPortByte
#endif

#ifdef InPortWord
#undef InPortWord
#endif

#ifdef InPortLong
#undef InPortLong
#endif

/**********************************************************************/
/*  LINUX KERNEL                                                      */
/**********************************************************************/

#ifdef SIS_LINUX_KERNEL

#ifdef CONFIG_FB_SIS_300
#define SIS300
#endif

#ifdef CONFIG_FB_SIS_315
#define SIS315H
#endif

#if !defined(SIS300) && !defined(SIS315H)
#warning Neither CONFIG_FB_SIS_300 nor CONFIG_FB_SIS_315 is set
#warning sisfb will not work!
#endif

#define OutPortByte(p,v) outb((u8)(v),(SISIOADDRESS)(p))
#define OutPortWord(p,v) outw((u16)(v),(SISIOADDRESS)(p))
#define OutPortLong(p,v) outl((u32)(v),(SISIOADDRESS)(p))
#define InPortByte(p)    inb((SISIOADDRESS)(p))
#define InPortWord(p)    inw((SISIOADDRESS)(p))
#define InPortLong(p)    inl((SISIOADDRESS)(p))
#define SiS_SetMemory(MemoryAddress,MemorySize,value) memset_io(MemoryAddress, value, MemorySize)

#endif /* LINUX_KERNEL */

/**********************************************************************/
/*  XFree86/X.org                                                    */
/**********************************************************************/

#ifdef SIS_XORG_XF86

#define SIS300
#define SIS315H

#define OutPortByte(p,v) outSISREG((IOADDRESS)(p),(CARD8)(v))
#define OutPortWord(p,v) outSISREGW((IOADDRESS)(p),(CARD16)(v))
#define OutPortLong(p,v) outSISREGL((IOADDRESS)(p),(CARD32)(v))
#define InPortByte(p)    inSISREG((IOADDRESS)(p))
#define InPortWord(p)    inSISREGW((IOADDRESS)(p))
#define InPortLong(p)    inSISREGL((IOADDRESS)(p))
#define SiS_SetMemory(MemoryAddress,MemorySize,value) memset(MemoryAddress, value, MemorySize)

#endif /* XF86 */

#endif  /* _OSDEF_H_ */
