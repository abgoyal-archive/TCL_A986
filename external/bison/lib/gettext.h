
#ifndef _LIBGETTEXT_H
#define _LIBGETTEXT_H 1

/* NLS can be disabled through the configure --disable-nls option.  */
#if ENABLE_NLS

/* Get declarations of GNU message catalog functions.  */
# include <libintl.h>

#else

#if defined(__sun)
# include <locale.h>
#endif

#if defined(__cplusplus) && defined(__GNUG__) && (__GNUC__ >= 3)
# include <cstdlib>
# if (__GLIBC__ >= 2) || _GLIBCXX_HAVE_LIBINTL_H
#  include <libintl.h>
# endif
#endif

# define gettext(Msgid) ((const char *) (Msgid))
# define dgettext(Domainname, Msgid) ((const char *) (Msgid))
# define dcgettext(Domainname, Msgid, Category) ((const char *) (Msgid))
# define ngettext(Msgid1, Msgid2, N) \
    ((N) == 1 ? (const char *) (Msgid1) : (const char *) (Msgid2))
# define dngettext(Domainname, Msgid1, Msgid2, N) \
    ((N) == 1 ? (const char *) (Msgid1) : (const char *) (Msgid2))
# define dcngettext(Domainname, Msgid1, Msgid2, N, Category) \
    ((N) == 1 ? (const char *) (Msgid1) : (const char *) (Msgid2))
# define textdomain(Domainname) ((const char *) (Domainname))
# define bindtextdomain(Domainname, Dirname) ((const char *) (Dirname))
# define bind_textdomain_codeset(Domainname, Codeset) ((const char *) (Codeset))

#endif

#define gettext_noop(String) String

#endif /* _LIBGETTEXT_H */
