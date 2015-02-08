
#ifndef _TRACE_PAGE_ALLOC_H
#define _TRACE_PAGE_ALLOC_H

#include <linux/tracepoint.h>

DECLARE_TRACE(page_alloc,
	TP_PROTO(struct page *page, unsigned int order),
	TP_ARGS(page, order));
DECLARE_TRACE(page_free,
	TP_PROTO(struct page *page, unsigned int order),
	TP_ARGS(page, order));

#endif
