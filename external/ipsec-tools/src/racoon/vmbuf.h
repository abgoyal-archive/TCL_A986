/*	$NetBSD: vmbuf.h,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/* Id: vmbuf.h,v 1.4 2005/10/30 10:28:44 vanhu Exp */


#ifndef _VMBUF_H
#define _VMBUF_H

typedef struct _vchar_t_ {
#if notyet
	u_int32_t t;	/* type of the value */
	vchar_t *n;	/* next vchar_t buffer */
	size_t bl;	/* length of the buffer */
	caddr_t bp;	/* pointer to the buffer */
#endif
	size_t l;	/* length of the value */
	caddr_t v;	/* place holder to the pointer to the value */
} vchar_t;

#define VPTRINIT(p) \
do { \
	if (p) { \
		vfree(p); \
		(p) = NULL; \
	} \
} while(0);

#if defined(__APPLE__) && defined(__MACH__)
/* vfree is already defined in Apple's system libraries */
#define vfree   vmbuf_free
#endif

extern vchar_t *vmalloc __P((size_t));
extern vchar_t *vrealloc __P((vchar_t *, size_t));
extern void vfree __P((vchar_t *));
extern vchar_t *vdup __P((vchar_t *));

#endif /* _VMBUF_H */
