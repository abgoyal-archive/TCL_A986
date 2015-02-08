/*	$OpenBSD: misc.h,v 1.12 2002/03/19 10:49:35 markus Exp $	*/


/* actually from atomicio, but is only used in scp code */
#define vwrite (ssize_t (*)(int, void *, size_t))write

char	*chop(char *);
char	*strdelim(char **);
void	 set_nonblock(int);
void	 unset_nonblock(int);
void	 set_nodelay(int);
int	 a2port(const char *);
char	*cleanhostname(char *);
char	*colon(char *);
long	 convtime(const char *);

struct passwd *pwcopy(struct passwd *);

typedef struct arglist arglist;
struct arglist {
	char    **list;
	int     num;
	int     nalloc;
};
void	 addargs(arglist *, char *, ...);
void	 replacearg(arglist *, u_int, char *, ...);
void	 freeargs(arglist *);

/* from xmalloc.h */
void	*xmalloc(size_t);
void	*xrealloc(void *, size_t);
void     xfree(void *);
char	*xstrdup(const char *);

char *ssh_get_progname(char *);
void fatal(char* fmt,...);
void sanitise_stdfd(void);

/* Required for non-BSD platforms, from OpenSSH's defines.h */
#ifndef timersub
#define timersub(a, b, result)                  \
   do {                             \
      (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;     \
      (result)->tv_usec = (a)->tv_usec - (b)->tv_usec;      \
      if ((result)->tv_usec < 0) {              \
     --(result)->tv_sec;                    \
     (result)->tv_usec += 1000000;              \
      }                             \
   } while (0)
#endif

#ifndef TIMEVAL_TO_TIMESPEC
#define TIMEVAL_TO_TIMESPEC(tv, ts) {                   \
    (ts)->tv_sec = (tv)->tv_sec;                    \
    (ts)->tv_nsec = (tv)->tv_usec * 1000;               \
}
#endif

