

#if !defined _LIBINTL_H || !defined _LIBGETTEXT_H
#ifndef _LIBINTL_H
# define _LIBINTL_H	1
#endif
#define _LIBGETTEXT_H	1

#define __USE_GNU_GETTEXT 1

#include <sys/types.h>

#if HAVE_LOCALE_H
# include <locale.h>
#endif


#ifdef __cplusplus
extern "C" {
#endif

/* @@ end of prolog @@ */

#ifndef PARAMS
# if __STDC__ || defined __cplusplus
#  define PARAMS(args) args
# else
#  define PARAMS(args) ()
# endif
#endif

#ifndef NULL
# if !defined __cplusplus || defined __GNUC__
#  define NULL ((void *) 0)
# else
#  define NULL (0)
# endif
#endif

#if !HAVE_LC_MESSAGES
# define LC_MESSAGES (-1)
#endif


struct _msg_ent
{
  const char *_msg;
  int _msg_number;
};


#if HAVE_CATGETS
extern const struct _msg_ent _msg_tbl[];
extern int _msg_tbl_length;
#endif


#define gettext_noop(Str) (Str)

extern char *gettext PARAMS ((const char *__msgid));
extern char *gettext__ PARAMS ((const char *__msgid));

extern char *dgettext PARAMS ((const char *__domainname, const char *__msgid));
extern char *dgettext__ PARAMS ((const char *__domainname,
				 const char *__msgid));

extern char *dcgettext PARAMS ((const char *__domainname, const char *__msgid,
				int __category));
extern char *dcgettext__ PARAMS ((const char *__domainname,
				  const char *__msgid, int __category));


extern char *textdomain PARAMS ((const char *__domainname));
extern char *textdomain__ PARAMS ((const char *__domainname));

extern char *bindtextdomain PARAMS ((const char *__domainname,
				  const char *__dirname));
extern char *bindtextdomain__ PARAMS ((const char *__domainname,
				    const char *__dirname));

#if ENABLE_NLS

# if !HAVE_CATGETS && (!HAVE_GETTEXT || HAVE_DCGETTEXT)

#  define gettext(Msgid)						      \
     dgettext (NULL, Msgid)

#  define dgettext(Domainname, Msgid)					      \
     dcgettext (Domainname, Msgid, LC_MESSAGES)

#  if defined __GNUC__ && __GNUC__ == 2 && __GNUC_MINOR__ >= 7
extern int _nl_msg_cat_cntr;

#   define dcgettext(Domainname, Msgid, Category)			      \
  (__extension__							      \
   ({									      \
     char *__result;							      \
     if (__builtin_constant_p (Msgid))					      \
       {								      \
	 static char *__translation__;					      \
	 static int __catalog_counter__;				      \
	 if (! __translation__ || __catalog_counter__ != _nl_msg_cat_cntr)    \
	   {								      \
	     __translation__ =						      \
	       dcgettext__ (Domainname, Msgid, Category);		      \
	     __catalog_counter__ = _nl_msg_cat_cntr;			      \
	   }								      \
	 __result = __translation__;					      \
       }								      \
     else								      \
       __result = dcgettext__ (Domainname, Msgid, Category);		      \
     __result;								      \
    }))
#  endif
# endif

#else

# define gettext(Msgid) (Msgid)
# define dgettext(Domainname, Msgid) (Msgid)
# define dcgettext(Domainname, Msgid, Category) (Msgid)
# define textdomain(Domainname) ((char *) Domainname)
# define bindtextdomain(Domainname, Dirname) ((char *) Dirname)

#endif

/* @@ begin of epilog @@ */

#ifdef __cplusplus
}
#endif

#endif
