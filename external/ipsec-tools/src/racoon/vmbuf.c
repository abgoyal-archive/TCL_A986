/*	$NetBSD: vmbuf.c,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/*	$KAME: vmbuf.c,v 1.11 2001/11/26 16:54:29 sakane Exp $	*/


#include "config.h"

#define NONEED_DRM

#include <sys/types.h>
#include <sys/param.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "var.h"
#include "misc.h"
#include "vmbuf.h"
#include "debug.h"
#include "plog.h"
#include "gcmalloc.h"

vchar_t *
vmalloc(size)
	size_t size;
{
	vchar_t *var;

	if ((var = (vchar_t *)racoon_malloc(sizeof(*var))) == NULL)
		return NULL;

	var->l = size;
	if (size == 0) {
		var->v = NULL;
	} else {
		var->v = (caddr_t)racoon_calloc(1, size);
		if (var->v == NULL) {
			(void)racoon_free(var);
			return NULL;
		}
	}

	return var;
}

vchar_t *
vrealloc(ptr, size)
	vchar_t *ptr;
	size_t size;
{
	caddr_t v;
	
	if (ptr != NULL) {
		if (ptr->l == 0) {
			(void)vfree(ptr);
			return vmalloc(size); /* zero-fill it? */
		}

		if ((v = (caddr_t)racoon_realloc(ptr->v, size)) == NULL) {
			(void)vfree(ptr);
			return NULL;
		}

		if ( size > ptr->l)
			memset(v + ptr->l, 0, size - ptr->l);
		ptr->v = v;
		ptr->l = size;
	} else {
		if ((ptr = vmalloc(size)) == NULL)
			return NULL;
	}

	return ptr;
}

void
vfree(var)
	vchar_t *var;
{
	if (var == NULL)
		return;

	if (var->v)
		(void)racoon_free(var->v);

	(void)racoon_free(var);

	return;
}

vchar_t *
vdup(src)
	vchar_t *src;
{
	vchar_t *new;

	if (src == NULL) {
		plog(LLV_ERROR, LOCATION, NULL, "vdup(NULL) called\n");
		return NULL;
	}

	if ((new = vmalloc(src->l)) == NULL)
		return NULL;

	memcpy(new->v, src->v, src->l);

	return new;
}
