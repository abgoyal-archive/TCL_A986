

/* Prototypes missing in SunOS 5 */
#if defined(_STDIO_H) && defined(HAVE_SETLINEBUF)
int	setlinebuf(FILE *);
#endif
char    *strerror(int);
int	snprintf(char *, size_t, const char *, ...);
int	strcasecmp(const char *, const char *);
