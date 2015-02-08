
#include "includes.h"

#ifndef HAVE_GETUSERSHELL
static char **curshell, **shells, *strings;
static char **initshells();
#endif

#ifndef HAVE_STRLCPY
size_t strlcpy(char *dst, const char *src, size_t size) {

	size_t i;

	/* this is undefined, though size==0 -> return 0 */
	if (size < 1) {
		return 0;
	}

	for (i = 0; i < size-1; i++) {
		if (src[i] == '\0') {
			break;
		} else {
			dst[i] = src[i];
		}
	}

	dst[i] = '\0';
	return strlen(src);

}
#endif /* HAVE_STRLCPY */

#ifndef HAVE_STRLCAT
/* taken from openbsd-compat for OpenSSH 3.6.1p1 */
	size_t
strlcat(dst, src, siz)
	char *dst;
	const char *src;
	size_t siz;
{
	register char *d = dst;
	register const char *s = src;
	register size_t n = siz;
	size_t dlen;

	/* Find the end of dst and adjust bytes left but don't go past end */
	while (n-- != 0 && *d != '\0')
		d++;
	dlen = d - dst;
	n = siz - dlen;

	if (n == 0)
		return(dlen + strlen(s));
	while (*s != '\0') {
		if (n != 1) {
			*d++ = *s;
			n--;
		}
		s++;
	}
	*d = '\0';

	return(dlen + (s - src));	/* count does not include NUL */
}
#endif /* HAVE_STRLCAT */

#ifndef HAVE_DAEMON
/* From NetBSD - daemonise a process */

int daemon(int nochdir, int noclose) {

	int fd;

	switch (fork()) {
		case -1:
			return (-1);
		case 0:
			break;
		default:
			_exit(0);
	}

	if (setsid() == -1)
		return -1;

	if (!nochdir)
		(void)chdir("/");

	if (!noclose && (fd = open(_PATH_DEVNULL, O_RDWR, 0)) != -1) {
		(void)dup2(fd, STDIN_FILENO);
		(void)dup2(fd, STDOUT_FILENO);
		(void)dup2(fd, STDERR_FILENO);
		if (fd > STDERR_FILENO)
			(void)close(fd);
	}
	return 0;
}
#endif /* HAVE_DAEMON */

#ifndef HAVE_BASENAME

char *basename(const char *path) {

	char *foo = strrchr(path, '/');
	return ++foo;
}

#endif /* HAVE_BASENAME */

#ifndef HAVE_GETUSERSHELL

char * getusershell() {
	char *ret;

	if (curshell == NULL)
		curshell = initshells();
	ret = *curshell;
	if (ret != NULL)
		curshell++;
	return (ret);
}

void endusershell() {

	if (shells != NULL)
		free(shells);
	shells = NULL;
	if (strings != NULL)
		free(strings);
	strings = NULL;
	curshell = NULL;
}

void setusershell() {
	curshell = initshells();
}

static char **initshells() {
	/* don't touch this list. */
	const char *okshells[] = { "/bin/sh", "/bin/csh", NULL };
	register char **sp, *cp;
	register FILE *fp;
	struct stat statb;
	int flen;

	if (shells != NULL)
		free(shells);
	shells = NULL;
	if (strings != NULL)
		free(strings);
	strings = NULL;
	if ((fp = fopen("/etc/shells", "rc")) == NULL)
		return (char **) okshells;
	if (fstat(fileno(fp), &statb) == -1) {
		(void)fclose(fp);
		return (char **) okshells;
	}
	if ((strings = malloc((u_int)statb.st_size + 1)) == NULL) {
		(void)fclose(fp);
		return (char **) okshells;
	}
	shells = calloc((unsigned)statb.st_size / 3, sizeof (char *));
	if (shells == NULL) {
		(void)fclose(fp);
		free(strings);
		strings = NULL;
		return (char **) okshells;
	}
	sp = shells;
	cp = strings;
	flen = statb.st_size;
	while (fgets(cp, flen - (cp - strings), fp) != NULL) {
		while (*cp != '#' && *cp != '/' && *cp != '\0')
			cp++;
		if (*cp == '#' || *cp == '\0')
			continue;
		*sp++ = cp;
		while (!isspace(*cp) && *cp != '#' && *cp != '\0')
			cp++;
		*cp++ = '\0';
	}
	*sp = NULL;
	(void)fclose(fp);
	return (shells);
}

#endif /* HAVE_GETUSERSHELL */
