


#ifndef _XME_H_INCLUDED 
#define _XME_H_INCLUDED

typedef struct {        
  short		        x;
  short		        y;
  unsigned short	w;
  unsigned short	h;
} XiGMiscViewInfo;

typedef struct {        
  unsigned short        width;
  unsigned short        height;
  int                   refresh;
} XiGMiscResolutionInfo;

extern Bool XiGMiscQueryVersion(Display *dpy, int *major, int *minor);
extern int XiGMiscQueryViews(Display *dpy, int screen, 
			     XiGMiscViewInfo **pviews);
extern int XiGMiscQueryResolutions(Display *dpy, int screen, int view, 
			    int *pactive, 
			    XiGMiscResolutionInfo **presolutions);
extern void XiGMiscChangeResolution(Display *dpy, int screen, int view, 
			     int width, int height, int refresh);

/* SDL addition from Ryan: free memory used by xme. */
extern void XiGMiscDestroy(void);

#endif /* _XME_H_INCLUDED */


