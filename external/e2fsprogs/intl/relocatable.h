
#ifndef _RELOCATABLE_H
#define _RELOCATABLE_H

#ifdef __cplusplus
extern "C" {
#endif


/* This can be enabled through the configure --enable-relocatable option.  */
#if ENABLE_RELOCATABLE

#if defined _MSC_VER && BUILDING_DLL
# define RELOCATABLE_DLL_EXPORTED __declspec(dllexport)
#else
# define RELOCATABLE_DLL_EXPORTED
#endif

extern RELOCATABLE_DLL_EXPORTED void
       set_relocation_prefix (const char *orig_prefix,
			      const char *curr_prefix);

extern const char * relocate (const char *pathname);


extern const char * compute_curr_prefix (const char *orig_installprefix,
					 const char *orig_installdir,
					 const char *curr_pathname);

#else

/* By default, we use the hardwired pathnames.  */
#define relocate(pathname) (pathname)

#endif


#ifdef __cplusplus
}
#endif

#endif /* _RELOCATABLE_H */
