
/* $XFree86: xc/lib/Xv/Xvlibint.h,v 1.5 2001/07/25 15:04:53 dawes Exp $ */

#ifndef XVLIBINT_H
#define XVLIBINT_H

#define NEED_REPLIES

#include <X11/Xlibint.h>
#include "../extensions/Xvproto.h"
#include "../extensions/Xvlib.h"

#if !defined(UNIXCPP)
#define XvGetReq(name, req) \
        WORD64ALIGN\
	if ((dpy->bufptr + SIZEOF(xv##name##Req)) > dpy->bufmax)\
		_XFlush(dpy);\
	req = (xv##name##Req *)(dpy->last_req = dpy->bufptr);\
	req->reqType = info->codes->major_opcode;\
        req->xvReqType = xv_##name; \
        req->length = (SIZEOF(xv##name##Req))>>2;\
	dpy->bufptr += SIZEOF(xv##name##Req);\
	dpy->request++

#else  /* non-ANSI C uses empty comment instead of "##" for token concatenation */
#define XvGetReq(name, req) \
        WORD64ALIGN\
	if ((dpy->bufptr + SIZEOF(xv/**/name/**/Req)) > dpy->bufmax)\
		_XFlush(dpy);\
	req = (xv/**/name/**/Req *)(dpy->last_req = dpy->bufptr);\
	req->reqType = info->codes->major_opcode;\
	req->xvReqType = xv_/**/name;\
	req->length = (SIZEOF(xv/**/name/**/Req))>>2;\
	dpy->bufptr += SIZEOF(xv/**/name/**/Req);\
	dpy->request++
#endif


#endif /* XVLIBINT_H */
