
/* $Id: eshlibld.h,v 1.2 2001/02/23 13:47:33 bjornw Exp $ */

#ifndef _cris_relocate_h
#define _cris_relocate_h


#include <linux/limits.h>

/* Maybe do sanity checking if file input. */
#undef SANITYCHECK_RELOC

/* Maybe output debug messages. */
#undef RELOC_DEBUG

#ifndef SHARE_LIB_CORE
# if (defined(__KERNEL__) || !defined(RELOC_DEBUG)) \
     && !defined(CONFIG_SHARE_SHLIB_CORE)
#  define SHARE_LIB_CORE 0
# else
#  define SHARE_LIB_CORE 1
# endif /* __KERNEL__ etc */
#endif /* SHARE_LIB_CORE */


extern int
perform_cris_aout_relocations(unsigned long text, unsigned long tlength,
			      unsigned long data, unsigned long dlength,
			      unsigned long baddr, unsigned long blength,

			      /* These may be zero when there's "perfect"
				 position-independent code. */
			      unsigned char *trel, unsigned long tsrel,
			      unsigned long dsrel,

			      /* These will be zero at a first try, to see
				 if code is statically linked.  Else a
				 second try, with the symbol table and
				 string table nonzero should be done. */
			      unsigned char *symbols, unsigned long symlength,
			      unsigned char *strings, unsigned long stringlength,

			      /* These will only be used when symbol table
			       information is present. */
			      char **env, int envc,
			      int euid, int is_suid);


#ifdef RELOC_DEBUG
/* Task-specific debug stuff. */
struct task_reloc_debug {
	struct memdebug *alloclast;
	unsigned long alloc_total;
	unsigned long export_total;
};
#endif /* RELOC_DEBUG */

#if SHARE_LIB_CORE


struct shlibdep;

extern void
shlibmod_exit(struct shlibdep **deps);

/* Returns 0 if failure, nonzero for ok. */
extern int
shlibmod_fork(struct shlibdep **deps);

#else  /* ! SHARE_LIB_CORE */
# define shlibmod_exit(x)
# define shlibmod_fork(x) 1
#endif /* ! SHARE_LIB_CORE */

#endif _cris_relocate_h
/********************** END OF FILE eshlibld.h *****************************/

