
#ifndef _LIBEBLP_H
#define _LIBEBLP_H 1

#include <gelf.h>
#include <libebl.h>
//#include <libintl.h>


/* Type of the initialization functions in the backend modules.  */
typedef int (*ebl_bhinit_t) (Elf *, GElf_Half, Ebl *, size_t);


/* gettext helper macros.  */
#define _(Str) dgettext ("elfutils", Str)

#endif	/* libeblP.h */
