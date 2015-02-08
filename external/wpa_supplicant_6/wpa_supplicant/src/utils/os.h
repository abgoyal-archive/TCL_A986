

#ifndef OS_H
#define OS_H

typedef long os_time_t;

void os_sleep(os_time_t sec, os_time_t usec);

struct os_time {
	os_time_t sec;
	os_time_t usec;
};

int os_get_time(struct os_time *t);


/* Helper macros for handling struct os_time */

#define os_time_before(a, b) \
	((a)->sec < (b)->sec || \
	 ((a)->sec == (b)->sec && (a)->usec < (b)->usec))

#define os_time_sub(a, b, res) do { \
	(res)->sec = (a)->sec - (b)->sec; \
	(res)->usec = (a)->usec - (b)->usec; \
	if ((res)->usec < 0) { \
		(res)->sec--; \
		(res)->usec += 1000000; \
	} \
} while (0)

int os_mktime(int year, int month, int day, int hour, int min, int sec,
	      os_time_t *t);


int os_daemonize(const char *pid_file);

void os_daemonize_terminate(const char *pid_file);

int os_get_random(unsigned char *buf, size_t len);

unsigned long os_random(void);

char * os_rel2abs_path(const char *rel_path);

int os_program_init(void);

void os_program_deinit(void);

int os_setenv(const char *name, const char *value, int overwrite);

int os_unsetenv(const char *name);

char * os_readfile(const char *name, size_t *len);

void * os_zalloc(size_t size);



#ifdef OS_NO_C_LIB_DEFINES

void * os_malloc(size_t size);

void * os_realloc(void *ptr, size_t size);

void os_free(void *ptr);

void * os_memcpy(void *dest, const void *src, size_t n);

void * os_memmove(void *dest, const void *src, size_t n);

void * os_memset(void *s, int c, size_t n);

int os_memcmp(const void *s1, const void *s2, size_t n);

char * os_strdup(const char *s);

size_t os_strlen(const char *s);

int os_strcasecmp(const char *s1, const char *s2);

int os_strncasecmp(const char *s1, const char *s2, size_t n);

char * os_strchr(const char *s, int c);

char * os_strrchr(const char *s, int c);

int os_strcmp(const char *s1, const char *s2);

int os_strncmp(const char *s1, const char *s2, size_t n);

char * os_strncpy(char *dest, const char *src, size_t n);

char * os_strstr(const char *haystack, const char *needle);

int os_snprintf(char *str, size_t size, const char *format, ...);

#else /* OS_NO_C_LIB_DEFINES */

#ifndef os_malloc
#define os_malloc(s) malloc((s))
#endif
#ifndef os_realloc
#define os_realloc(p, s) realloc((p), (s))
#endif
#ifndef os_free
#define os_free(p) free((p))
#endif

#ifndef os_memcpy
#define os_memcpy(d, s, n) memcpy((d), (s), (n))
#endif
#ifndef os_memmove
#define os_memmove(d, s, n) memmove((d), (s), (n))
#endif
#ifndef os_memset
#define os_memset(s, c, n) memset(s, c, n)
#endif
#ifndef os_memcmp
#define os_memcmp(s1, s2, n) memcmp((s1), (s2), (n))
#endif

#ifndef os_strdup
#ifdef _MSC_VER
#define os_strdup(s) _strdup(s)
#else
#define os_strdup(s) strdup(s)
#endif
#endif
#ifndef os_strlen
#define os_strlen(s) strlen(s)
#endif
#ifndef os_strcasecmp
#ifdef _MSC_VER
#define os_strcasecmp(s1, s2) _stricmp((s1), (s2))
#else
#define os_strcasecmp(s1, s2) strcasecmp((s1), (s2))
#endif
#endif
#ifndef os_strncasecmp
#ifdef _MSC_VER
#define os_strncasecmp(s1, s2, n) _strnicmp((s1), (s2), (n))
#else
#define os_strncasecmp(s1, s2, n) strncasecmp((s1), (s2), (n))
#endif
#endif
#ifndef os_strchr
#define os_strchr(s, c) strchr((s), (c))
#endif
#ifndef os_strcmp
#define os_strcmp(s1, s2) strcmp((s1), (s2))
#endif
#ifndef os_strncmp
#define os_strncmp(s1, s2, n) strncmp((s1), (s2), (n))
#endif
#ifndef os_strncpy
#define os_strncpy(d, s, n) strncpy((d), (s), (n))
#endif
#ifndef os_strrchr
#define os_strrchr(s, c) strrchr((s), (c))
#endif
#ifndef os_strstr
#define os_strstr(h, n) strstr((h), (n))
#endif

#ifndef os_snprintf
#ifdef _MSC_VER
#define os_snprintf _snprintf
#else
#define os_snprintf snprintf
#endif
#endif

#endif /* OS_NO_C_LIB_DEFINES */


size_t os_strlcpy(char *dest, const char *src, size_t siz);


#ifdef OS_REJECT_C_LIB_FUNCTIONS
#define malloc OS_DO_NOT_USE_malloc
#define realloc OS_DO_NOT_USE_realloc
#define free OS_DO_NOT_USE_free
#define memcpy OS_DO_NOT_USE_memcpy
#define memmove OS_DO_NOT_USE_memmove
#define memset OS_DO_NOT_USE_memset
#define memcmp OS_DO_NOT_USE_memcmp
#undef strdup
#define strdup OS_DO_NOT_USE_strdup
#define strlen OS_DO_NOT_USE_strlen
#define strcasecmp OS_DO_NOT_USE_strcasecmp
#define strncasecmp OS_DO_NOT_USE_strncasecmp
#undef strchr
#define strchr OS_DO_NOT_USE_strchr
#undef strcmp
#define strcmp OS_DO_NOT_USE_strcmp
#undef strncmp
#define strncmp OS_DO_NOT_USE_strncmp
#undef strncpy
#define strncpy OS_DO_NOT_USE_strncpy
#define strrchr OS_DO_NOT_USE_strrchr
#define strstr OS_DO_NOT_USE_strstr
#undef snprintf
#define snprintf OS_DO_NOT_USE_snprintf

#define strcpy OS_DO_NOT_USE_strcpy
#endif /* OS_REJECT_C_LIB_FUNCTIONS */

#endif /* OS_H */
