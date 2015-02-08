
#ifndef _ANDROID_SKIN_SCALER_H
#define _ANDROID_SKIN_SCALER_H

#include "android/skin/image.h"

typedef struct SkinScaler   SkinScaler;

/* create a new image scaler. by default, it uses a scale of 1.0 */
extern SkinScaler*  skin_scaler_create( void );

extern int          skin_scaler_set( SkinScaler*  scaler,
                                     double       scale,
                                     double       xDisp,
                                     double       yDisp );

extern void         skin_scaler_free( SkinScaler*  scaler );

extern void         skin_scaler_scale( SkinScaler*   scaler,
                                       SDL_Surface*  dst,
                                       SDL_Surface*  src,
                                       int           sx,
                                       int           sy,
                                       int           sw,
                                       int           sh );

#endif /* _ANDROID_SKIN_SCALER_H */
