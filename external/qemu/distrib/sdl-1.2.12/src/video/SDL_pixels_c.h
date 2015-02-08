
#include "SDL_config.h"

/* Useful functions and variables from SDL_pixel.c */

#include "SDL_blit.h"

/* Pixel format functions */
extern SDL_PixelFormat *SDL_AllocFormat(int bpp,
		Uint32 Rmask, Uint32 Gmask, Uint32 Bmask, Uint32 Amask);
extern SDL_PixelFormat *SDL_ReallocFormat(SDL_Surface *surface, int bpp,
		Uint32 Rmask, Uint32 Gmask, Uint32 Bmask, Uint32 Amask);
extern void SDL_FormatChanged(SDL_Surface *surface);
extern void SDL_FreeFormat(SDL_PixelFormat *format);

/* Blit mapping functions */
extern SDL_BlitMap *SDL_AllocBlitMap(void);
extern void SDL_InvalidateMap(SDL_BlitMap *map);
extern int SDL_MapSurface (SDL_Surface *src, SDL_Surface *dst);
extern void SDL_FreeBlitMap(SDL_BlitMap *map);

/* Miscellaneous functions */
extern Uint16 SDL_CalculatePitch(SDL_Surface *surface);
extern void SDL_DitherColors(SDL_Color *colors, int bpp);
extern Uint8 SDL_FindColor(SDL_Palette *pal, Uint8 r, Uint8 g, Uint8 b);
extern void SDL_ApplyGamma(Uint16 *gamma, SDL_Color *colors, SDL_Color *output, int ncolors);
