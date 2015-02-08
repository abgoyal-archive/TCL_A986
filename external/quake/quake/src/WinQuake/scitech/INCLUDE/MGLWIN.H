
#ifndef	__MGLWIN_H
#define	__MGLWIN_H

#ifndef MGLWIN
#define	MGLWIN
#endif

/*---------------------- Macros and type definitions ----------------------*/

typedef HWND		MGL_HWND;
typedef	HDC			MGL_HDC;
typedef	HINSTANCE	MGL_HINSTANCE;

/*------------------------- Function Prototypes ---------------------------*/

#ifdef	__cplusplus
extern "C" {			/* Use "C" linkage when in C++ mode	*/
#endif

/* Initialise the MGL for fullscreen output */

bool	MGLAPI MGL_init(m_int *driver,m_int *mode,const char *mglpath);

/* Initialise the MGL just for Windowed output, not full screen */

bool	MGLAPI MGL_initWindowed(const char *mglpath);


void 	MGLAPI MGL_registerFullScreenWindow(HWND hwndFullScreen);


void 	MGLAPI MGL_registerEventProc(WNDPROC userWndProc);


bool	MGLAPI MGL_changeDisplayMode(m_int mode);

/* Obtain the handle to the MGL fullscreen window when in fullscreen modes */

MGL_HWND MGLAPI MGL_getFullScreenWindow(void);

/* Tell the MGL what your applications main window is */

void	MGLAPI MGL_setMainWindow(MGL_HWND hwnd);

/* Tell the MGL your applications instance handle (call before all funcs!) */

void	MGLAPI MGL_setAppInstance(MGL_HINSTANCE hInstApp);

/* Device context management */

MGLDC	* MGLAPI MGL_createDisplayDC(m_int numBuffers);
MGLDC	* MGLAPI MGL_createSrollingDC(m_int virtualX,m_int virtualY,m_int numBuffers);
MGLDC	* MGLAPI MGL_createOffscreenDC(void);
MGLDC	* MGLAPI MGL_createLinearOffscreenDC(void);
MGLDC	* MGLAPI MGL_createWindowedDC(MGL_HWND hwnd);
MGLDC 	* MGLAPI MGL_createMemoryDC(m_int xSize,m_int ySize,m_int bitsPerPixel,pixel_format_t *pf);
bool	MGLAPI MGL_destroyDC(MGLDC *dc);


HDC		MGLAPI MGL_getWinDC(MGLDC *dc);

/* Associate a Window manager DC with the MGLDC for painting */

bool	MGLAPI MGL_setWinDC(MGLDC *dc,MGL_HDC hdc);

/* Activate the WindowDC's palette */

bool	MGLAPI MGL_activatePalette(MGLDC *dc,bool unrealize);


void	MGLAPI MGL_appActivate(MGLDC *winDC,bool active);

/* Generic helper functions */

ulong	MGLAPI MGL_getTicks(void);
ulong	MGLAPI MGL_getTickResolution(void);
void	MGLAPI MGL_delay(m_int millseconds);
void	MGLAPI MGL_beep(m_int freq,m_int milliseconds);

/* Fullscreen specific routines */

void	MGLAPI MGL_setPaletteSnowLevel(MGLDC *dc,m_int level);
m_int	MGLAPI MGL_getPaletteSnowLevel(MGLDC *dc);

/* Determine if a specific scancode'ed key is held down (PC specific) */

bool	MGLAPI EVT_isKeyDown(uchar scanCode);

#ifdef	__cplusplus
}						/* End of "C" linkage for C++	*/
#endif

#endif	/* __MGLWIN_H */
