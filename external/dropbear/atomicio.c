
/* RCSID("OpenBSD: atomicio.c,v 1.10 2001/05/08 22:48:07 markus Exp "); */

#include "atomicio.h"

ssize_t
atomicio(f, fd, _s, n)
	ssize_t (*f) ();
	int fd;
	void *_s;
	size_t n;
{
	char *s = _s;
	ssize_t res;
	size_t pos = 0;

	while (n > pos) {
		res = (f) (fd, s + pos, n - pos);
		switch (res) {
		case -1:
#ifdef EWOULDBLOCK
			if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
#else
			if (errno == EINTR || errno == EAGAIN)
#endif
				continue;
		case 0:
			return (res);
		default:
			pos += res;
		}
	}
	return (pos);
}
