
#if defined(__GNUC__) && defined(__i386__) && !defined(__MINGW32__)
#define FASTCALL __attribute__((regparm(3)))
#define PTRFASTCALL __attribute__((regparm(3)))
#endif


/* Make sure all of these are defined if they aren't already. */

#ifndef FASTCALL
#define FASTCALL
#endif

#ifndef PTRCALL
#define PTRCALL
#endif

#ifndef PTRFASTCALL
#define PTRFASTCALL
#endif

#ifndef XML_MIN_SIZE
#if !defined(__cplusplus) && !defined(inline)
#ifdef __GNUC__
#define inline __inline
#endif /* __GNUC__ */
#endif
#endif /* XML_MIN_SIZE */

#ifdef __cplusplus
#define inline inline
#else
#ifndef inline
#define inline
#endif
#endif
