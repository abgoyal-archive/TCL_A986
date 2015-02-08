
#ifndef OPD_COOKIE_H
#define OPD_COOKIE_H

typedef unsigned long long cookie_t;

#define INVALID_COOKIE ~0LLU
#define NO_COOKIE 0LLU

#define DCOOKIE_SHIFT 7

char const * find_cookie(cookie_t cookie);

/** return true if this cookie should be ignored */
int is_cookie_ignored(cookie_t cookie);

/** give a textual description of the cookie */
char const * verbose_cookie(cookie_t cookie);

void cookie_init(void);

#endif /* OPD_COOKIE_H */
