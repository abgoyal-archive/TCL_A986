
/* $XFree86: xc/include/extensions/extutil.h,v 1.9 2001/12/14 19:53:28 dawes Exp $ */

#ifndef _EXTUTIL_H_
#define _EXTUTIL_H_

#include "SDL_stdinc.h"		/* For portable string functions */

#include "./Xext.h"

typedef struct _XExtDisplayInfo {
    struct _XExtDisplayInfo *next;	/* keep a linked list */
    Display *display;			/* which display this is */
    XExtCodes *codes;			/* the extension protocol codes */
    XPointer data;			/* extra data for extension to use */
} XExtDisplayInfo;

typedef struct _XExtensionInfo {
    XExtDisplayInfo *head;		/* start of list */
    XExtDisplayInfo *cur;		/* most recently used */
    int ndisplays;			/* number of displays */
} XExtensionInfo;

typedef struct _XExtensionHooks {
    int (*create_gc)(
#if NeedNestedPrototypes
	      Display*			/* display */,
	      GC			/* gc */,
	      XExtCodes*		/* codes */
#endif
);
    int (*copy_gc)(
#if NeedNestedPrototypes
	      Display*			/* display */,
              GC			/* gc */,
              XExtCodes*		/* codes */
#endif
);
    int (*flush_gc)(
#if NeedNestedPrototypes
	      Display*			/* display */,
              GC			/* gc */,
              XExtCodes*		/* codes */
#endif
);
    int (*free_gc)(
#if NeedNestedPrototypes
	      Display*			/* display */,
              GC			/* gc */,
              XExtCodes*		/* codes */
#endif
);
    int (*create_font)(
#if NeedNestedPrototypes
	      Display*			/* display */,
              XFontStruct*		/* fs */,
              XExtCodes*		/* codes */
#endif
);
    int (*free_font)(
#if NeedNestedPrototypes
	      Display*			/* display */,
              XFontStruct*		/* fs */,
              XExtCodes*		/* codes */
#endif
);
    int (*close_display)(
#if NeedNestedPrototypes
	      Display*			/* display */,
              XExtCodes*		/* codes */
#endif
);
    Bool (*wire_to_event)(
#if NeedNestedPrototypes
	       Display*			/* display */,
               XEvent*			/* re */,
               xEvent*			/* event */
#endif
);
    Status (*event_to_wire)(
#if NeedNestedPrototypes
	      Display*			/* display */,
              XEvent*			/* re */,
              xEvent*			/* event */
#endif
);
    int (*error)(
#if NeedNestedPrototypes
	      Display*			/* display */,
              xError*			/* err */,
              XExtCodes*		/* codes */,
              int*			/* ret_code */
#endif
);
    char *(*error_string)(
#if NeedNestedPrototypes
	        Display*		/* display */,
                int			/* code */,
                XExtCodes*		/* codes */,
                char*			/* buffer */,
                int			/* nbytes */
#endif
);
} XExtensionHooks;

extern XExtensionInfo *XextCreateExtension(
#if NeedFunctionPrototypes
    void
#endif
);
extern void XextDestroyExtension(
#if NeedFunctionPrototypes
    XExtensionInfo*	/* info */
#endif
);
extern XExtDisplayInfo *XextAddDisplay(
#if NeedFunctionPrototypes
    XExtensionInfo*	/* extinfo */,
    Display*		/* dpy */,
    char*		/* ext_name */,
    XExtensionHooks*	/* hooks */,
    int			/* nevents */,
    XPointer		/* data */
#endif
);
extern int XextRemoveDisplay(
#if NeedFunctionPrototypes
    XExtensionInfo*	/* extinfo */,
    Display*		/* dpy */
#endif
);
extern XExtDisplayInfo *XextFindDisplay(
#if NeedFunctionPrototypes
    XExtensionInfo*	/* extinfo */,
    Display*		/* dpy */
#endif
);

#define XextHasExtension(i) ((i) && ((i)->codes))
#define XextCheckExtension(dpy,i,name,val) \
  if (!XextHasExtension(i)) { XMissingExtension (dpy, name); return val; }
#define XextSimpleCheckExtension(dpy,i,name) \
  if (!XextHasExtension(i)) { XMissingExtension (dpy, name); return; }


#define XEXT_GENERATE_FIND_DISPLAY(proc,extinfo,extname,hooks,nev,data) \
XExtDisplayInfo *proc (Display *dpy) \
{ \
    XExtDisplayInfo *dpyinfo; \
    if (!extinfo) { if (!(extinfo = XextCreateExtension())) return NULL; } \
    if (!(dpyinfo = XextFindDisplay (extinfo, dpy))) \
      dpyinfo = XextAddDisplay (extinfo,dpy,extname,hooks,nev,data); \
    return dpyinfo; \
}

#define XEXT_FIND_DISPLAY_PROTO(proc) \
	XExtDisplayInfo *proc(Display *dpy)

#define XEXT_GENERATE_CLOSE_DISPLAY(proc,extinfo) \
int proc (Display *dpy, XExtCodes *codes) \
{ \
    return XextRemoveDisplay (extinfo, dpy); \
}

#define XEXT_CLOSE_DISPLAY_PROTO(proc) \
	int proc(Display *dpy, XExtCodes *codes)

#define XEXT_GENERATE_ERROR_STRING(proc,extname,nerr,errl) \
char *proc (Display *dpy, int code, XExtCodes *codes, char *buf, int n) \
{  \
    code -= codes->first_error;  \
    if (code >= 0 && code < nerr) { \
	char tmp[256]; \
	SDL_snprintf (tmp, SDL_arraysize(tmp), "%s.%d", extname, code); \
	XGetErrorDatabaseText (dpy, "XProtoError", tmp, errl[code], buf, n); \
	return buf; \
    } \
    return (char *)0; \
}

#define XEXT_ERROR_STRING_PROTO(proc) \
	char *proc(Display *dpy, int code, XExtCodes *codes, char *buf, int n)
#endif
