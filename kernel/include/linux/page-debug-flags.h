
#ifndef LINUX_PAGE_DEBUG_FLAGS_H
#define  LINUX_PAGE_DEBUG_FLAGS_H


enum page_debug_flags {
	PAGE_DEBUG_FLAG_POISON,		/* Page is poisoned */
};


#ifdef CONFIG_WANT_PAGE_DEBUG_FLAGS
#if !defined(CONFIG_PAGE_POISONING) \
/* && !defined(CONFIG_PAGE_DEBUG_SOMETHING_ELSE) && ... */
#error WANT_PAGE_DEBUG_FLAGS is turned on with no debug features!
#endif
#endif /* CONFIG_WANT_PAGE_DEBUG_FLAGS */

#endif /* LINUX_PAGE_DEBUG_FLAGS_H */
