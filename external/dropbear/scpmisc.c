
/*RCSID("OpenBSD: misc.c,v 1.22 2003/09/18 08:49:45 markus Exp ");*/


/*RCSID("OpenBSD: xmalloc.c,v 1.16 2001/07/23 18:21:46 stevesk Exp ");*/

#include "includes.h"
#include "scpmisc.h"

void *
xmalloc(size_t size)
{
	void *ptr;

	if (size == 0) {
		fprintf(stderr, "xmalloc: zero size\n");
		exit(EXIT_FAILURE);
	}
	ptr = malloc(size);
	if (ptr == NULL) {
		fprintf(stderr, "xmalloc: out of memory (allocating %lu bytes)\n", (u_long) size);
		exit(EXIT_FAILURE);
	}
	return ptr;
}

void *
xrealloc(void *ptr, size_t new_size)
{
	void *new_ptr;

	if (new_size == 0) {
		fprintf(stderr, "xrealloc: zero size\n");
		exit(EXIT_FAILURE);
	}
	if (ptr == NULL)
		new_ptr = malloc(new_size);
	else
		new_ptr = realloc(ptr, new_size);
	if (new_ptr == NULL) {
		fprintf(stderr, "xrealloc: out of memory (new_size %lu bytes)\n", (u_long) new_size);
		exit(EXIT_FAILURE);
	}
	return new_ptr;
}

void
xfree(void *ptr)
{
	if (ptr == NULL) {
		fprintf(stderr, "xfree: NULL pointer given as argument\n");
		exit(EXIT_FAILURE);
	}
	free(ptr);
}

char *
xstrdup(const char *str)
{
	size_t len;
	char *cp;

	len = strlen(str) + 1;
	cp = xmalloc(len);
	strncpy(cp, str, len);
	return cp;
}

char *
cleanhostname(char *host)
{
	if (*host == '[' && host[strlen(host) - 1] == ']') {
		host[strlen(host) - 1] = '\0';
		return (host + 1);
	} else
		return host;
}

char *
colon(char *cp)
{
	int flag = 0;

	if (*cp == ':')		/* Leading colon is part of file name. */
		return (0);
	if (*cp == '[')
		flag = 1;

	for (; *cp; ++cp) {
		if (*cp == '@' && *(cp+1) == '[')
			flag = 1;
		if (*cp == ']' && *(cp+1) == ':' && flag)
			return (cp+1);
		if (*cp == ':' && !flag)
			return (cp);
		if (*cp == '/')
			return (0);
	}
	return (0);
}

/* function to assist building execv() arguments */
void
addargs(arglist *args, char *fmt, ...)
{
	va_list ap;
	char *cp;
	u_int nalloc;
	int r;

	va_start(ap, fmt);
	r = vasprintf(&cp, fmt, ap);
	va_end(ap);
	if (r == -1)
		fatal("addargs: argument too long");

	nalloc = args->nalloc;
	if (args->list == NULL) {
		nalloc = 32;
		args->num = 0;
	} else if (args->num+2 >= nalloc)
		nalloc *= 2;

	args->list = xrealloc(args->list, nalloc * sizeof(char *));
	args->nalloc = nalloc;
	args->list[args->num++] = cp;
	args->list[args->num] = NULL;
}

void
replacearg(arglist *args, u_int which, char *fmt, ...)
{
	va_list ap;
	char *cp;
	int r;

	va_start(ap, fmt);
	r = vasprintf(&cp, fmt, ap);
	va_end(ap);
	if (r == -1)
		fatal("replacearg: argument too long");

	if (which >= args->num)
		fatal("replacearg: tried to replace invalid arg %d >= %d",
		    which, args->num);
	xfree(args->list[which]);
	args->list[which] = cp;
}

void
freeargs(arglist *args)
{
	u_int i;

	if (args->list != NULL) {
		for (i = 0; i < args->num; i++)
			xfree(args->list[i]);
		xfree(args->list);
		args->nalloc = args->num = 0;
		args->list = NULL;
	}
}

char *ssh_get_progname(char *argv0)
{
	char *p;

	if (argv0 == NULL)
		return ("unknown");	/* XXX */
	p = strrchr(argv0, '/');
	if (p == NULL)
		p = argv0;
	else
		p++;

	return (xstrdup(p));
}

void fatal(char* fmt,...)
{
	va_list args;
	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);
	exit(255);
}

void
sanitise_stdfd(void)
{
	int nullfd, dupfd;

	if ((nullfd = dupfd = open(_PATH_DEVNULL, O_RDWR)) == -1) {
		fprintf(stderr, "Couldn't open /dev/null: %s", strerror(errno));
		exit(1);
	}
	while (++dupfd <= 2) {
		/* Only clobber closed fds */
		if (fcntl(dupfd, F_GETFL, 0) >= 0)
			continue;
		if (dup2(nullfd, dupfd) == -1) {
			fprintf(stderr, "dup2: %s", strerror(errno));
			exit(1);
		}
	}
	if (nullfd > 2)
		close(nullfd);
}
