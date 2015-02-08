
#ifndef _PROFILE_H
#define _PROFILE_H

typedef struct _profile_t *profile_t;

typedef void (*profile_syntax_err_cb_t)(const char *file, long err, 
					int line_num);

#define PROFILE_ITER_LIST_SECTION	0x0001
#define PROFILE_ITER_SECTIONS_ONLY	0x0002
#define PROFILE_ITER_RELATIONS_ONLY	0x0004

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

long profile_init
	(const char * *files, profile_t *ret_profile);

void profile_release
	(profile_t profile);

long profile_set_default
	(profile_t profile, const char *def_string);

long profile_get_string
	(profile_t profile, const char *name, const char *subname, 
			const char *subsubname, const char *def_val,
			char **ret_string);
long profile_get_integer
	(profile_t profile, const char *name, const char *subname,
			const char *subsubname, int def_val,
			int *ret_default);

long profile_get_uint
	(profile_t profile, const char *name, const char *subname,
		const char *subsubname, unsigned int def_val, 
		unsigned int *ret_int);

long profile_get_boolean
	(profile_t profile, const char *name, const char *subname,
			const char *subsubname, int def_val,
			int *ret_default);

long profile_iterator_create
	(profile_t profile, const char *const *names,
		   int flags, void **ret_iter);

void profile_iterator_free
	(void **iter_p);
	
long profile_iterator
	(void	**iter_p, char **ret_name, char **ret_value);

profile_syntax_err_cb_t profile_set_syntax_err_cb(profile_syntax_err_cb_t hook);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _KRB5_H */
