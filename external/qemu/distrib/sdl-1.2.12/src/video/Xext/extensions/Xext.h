
/* $XFree86: xc/include/extensions/Xext.h,v 1.7 2005/01/27 03:03:09 dawes Exp $ */

#ifndef _XEXT_H_
#define _XEXT_H_

#include <X11/Xfuncproto.h>

_XFUNCPROTOBEGIN

typedef int (*XExtensionErrorHandler)(Display *, _Xconst char *,
				      _Xconst char *);

extern XExtensionErrorHandler XSetExtensionErrorHandler(
	XExtensionErrorHandler handler
);

extern int XMissingExtension(
    Display*		/* dpy */,
    _Xconst char*	/* ext_name */
);

_XFUNCPROTOEND

#define X_EXTENSION_UNKNOWN "unknown"
#define X_EXTENSION_MISSING "missing"

#endif /* _XEXT_H_ */
