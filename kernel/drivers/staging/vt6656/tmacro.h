

#ifndef __TMACRO_H__
#define __TMACRO_H__

#include "ttype.h"

/****** Common helper macros ***********************************************/

#if !defined(LOBYTE)
#define LOBYTE(w)           ((BYTE)(w))
#endif
#if !defined(HIBYTE)
#define HIBYTE(w)           ((BYTE)(((WORD)(w) >> 8) & 0xFF))
#endif

#if !defined(LOWORD)
#define LOWORD(d)           ((WORD)(d))
#endif
#if !defined(HIWORD)
#define HIWORD(d)           ((WORD)((((DWORD)(d)) >> 16) & 0xFFFF))
#endif

#define LODWORD(q)          ((q).u.dwLowDword)
#define HIDWORD(q)          ((q).u.dwHighDword)

#if !defined(MAKEWORD)
#define MAKEWORD(lb, hb)    ((WORD)(((BYTE)(lb)) | (((WORD)((BYTE)(hb))) << 8)))
#endif
#if !defined(MAKEDWORD)
#define MAKEDWORD(lw, hw)   ((DWORD)(((WORD)(lw)) | (((DWORD)((WORD)(hw))) << 16)))
#endif

#endif /* __TMACRO_H__ */
