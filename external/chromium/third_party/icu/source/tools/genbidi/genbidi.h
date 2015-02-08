
#ifndef __GENBIDI_H__
#define __GENBIDI_H__

#include "unicode/utypes.h"
#include "propsvec.h"

U_CDECL_BEGIN

/* genbidi ------------------------------------------------------------------ */

/* global flags */
extern UBool beVerbose, haveCopyright;

/* properties vectors in genbidi.c */
extern UPropsVectors *pv;

/* prototypes */
U_CFUNC void
writeUCDFilename(char *basename, const char *filename, const char *suffix);

extern void
setUnicodeVersion(const char *v);

extern void
addMirror(UChar32 src, UChar32 mirror);

extern void
generateData(const char *dataDir, UBool csource);

U_CDECL_END

#endif
