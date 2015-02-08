
#ifndef _ANDROID_SKIN_IMAGE_H
#define _ANDROID_SKIN_IMAGE_H

#include "android/android.h"
#include <SDL.h>
#include "android/skin/rect.h"

/* helper functions */

extern SDL_Surface*    sdl_surface_from_argb32( void*  base, int  w, int  h );

/* skin image file objects */

typedef struct SkinImage   SkinImage;

/* a descriptor for a given skin image */
typedef struct SkinImageDesc {
    const char*      path;      /* image file path (must be .png) */
    AndroidRotation  rotation;  /* rotation */
    int              blend;     /* blending, 0..256 value */
} SkinImageDesc;

#define  SKIN_BLEND_NONE   0
#define  SKIN_BLEND_HALF   128
#define  SKIN_BLEND_FULL   256

/* a special value returned when an image cannot be properly loaded */
extern SkinImage*    SKIN_IMAGE_NONE;

/* return the SDL_Surface* pointer of a given skin image */
extern SDL_Surface*  skin_image_surface( SkinImage*  image );
extern int           skin_image_w      ( SkinImage*  image );
extern int           skin_image_h      ( SkinImage*  image );
extern int           skin_image_org_w  ( SkinImage*  image );
extern int           skin_image_org_h  ( SkinImage*  image );

extern SkinImage*    skin_image_find( SkinImageDesc*  desc );

extern SkinImage*    skin_image_find_simple( const char*  path );

extern SkinImage*    skin_image_ref( SkinImage*  image );

extern void          skin_image_unref( SkinImage**  pimage );

extern SkinImage*    skin_image_rotate( SkinImage*  source, SkinRotation  rotation );

extern SkinImage*    skin_image_clone( SkinImage*  source );

extern SkinImage*    skin_image_clone_full( SkinImage*       source,
                                            SkinRotation     rotation,
                                            int              blend );

/* apply blending to a source skin image and copy the result to a target clone image */
extern void          skin_image_blend_clone( SkinImage*  clone, SkinImage*  source, int  blend );

#endif /* _ANDROID_SKIN_IMAGE_H */
