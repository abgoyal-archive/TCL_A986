/*	$NetBSD: debugrm.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/* Id: debugrm.h,v 1.4 2006/04/06 14:00:06 manubsd Exp */


#ifndef _DEBUGRM_H
#define _DEBUGRM_H

#define DRMDUMPFILE	"/var/tmp/debugrm.dump"

#ifdef NONEED_DRM
#ifndef racoon_malloc
#define	racoon_malloc(sz)	malloc((sz))
#endif
#ifndef racoon_calloc
#define	racoon_calloc(cnt, sz)	calloc((cnt), (sz))
#endif
#ifndef racoon_realloc
#define	racoon_realloc(old, sz)	realloc((old), (sz))
#endif
#ifndef racoon_free
#define	racoon_free(p)		free((p))
#endif
#ifndef racoon_strdup
#define	racoon_strdup(p)	strdup((p))
#endif
#else /*!NONEED_DRM*/
#ifndef racoon_malloc
#define	racoon_malloc(sz)	\
	DRM_malloc(__FILE__, __LINE__, __func__, (sz))
#endif
#ifndef racoon_calloc
#define	racoon_calloc(cnt, sz)	\
	DRM_calloc(__FILE__, __LINE__, __func__, (cnt), (sz))
#endif
#ifndef racoon_realloc
#define	racoon_realloc(old, sz)	\
	DRM_realloc(__FILE__, __LINE__, __func__, (old), (sz))
#endif
#ifndef racoon_free
#define	racoon_free(p)		\
	DRM_free(__FILE__, __LINE__, __func__, (p))
#endif
#ifndef racoon_strdup
#define	racoon_strdup(p)	\
	DRM_strdup(__FILE__, __LINE__, __func__, (p))
#endif
#endif /*NONEED_DRM*/

extern void DRM_init __P((void));
extern void DRM_dump __P((void));
extern void *DRM_malloc __P((char *, int, char *, size_t));
extern void *DRM_calloc __P((char *, int, char *, size_t, size_t));
extern void *DRM_realloc __P((char *, int, char *, void *, size_t));
extern void DRM_free __P((char *, int, char *, void *));
extern char *DRM_strdup __P((char *, int, char *, const char *));

#ifndef NONEED_DRM
#define	vmalloc(sz)	\
	DRM_vmalloc(__FILE__, __LINE__, __func__, (sz))
#define	vdup(old)	\
	DRM_vdup(__FILE__, __LINE__, __func__, (old))
#define	vrealloc(old, sz)	\
	DRM_vrealloc(__FILE__, __LINE__, __func__, (old), (sz))
#define	vfree(p)		\
	DRM_vfree(__FILE__, __LINE__, __func__, (p))
#endif

extern void *DRM_vmalloc __P((char *, int, char *, size_t));
extern void *DRM_vrealloc __P((char *, int, char *, void *, size_t));
extern void DRM_vfree __P((char *, int, char *, void *));
extern void *DRM_vdup __P((char *, int, char *, void *));

#endif /* _DEBUGRM_H */
