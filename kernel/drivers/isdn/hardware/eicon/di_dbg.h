

#ifndef __DIVA_DI_DBG_INC__
#define __DIVA_DI_DBG_INC__
#if !defined (dtrc)
#define dtrc(a)
#endif
#if !defined (dbug)
#define dbug(a)
#endif
#if !defined USE_EXTENDED_DEBUGS
extern void (*dprintf)(char*, ...);
#endif
#endif
