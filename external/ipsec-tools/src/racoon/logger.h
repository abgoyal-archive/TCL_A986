/*	$NetBSD: logger.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/* Id: logger.h,v 1.3 2004/06/11 16:00:16 ludvigm Exp */


#ifndef _LOGGER_H
#define _LOGGER_H

struct log {
	int head;
	int siz;
	char **buf;
	time_t *tbuf;
	char *fname;
};

extern struct log *log_open __P((size_t, char *));
extern void log_add __P((struct log *, char *));
extern int log_print __P((struct log *, char *));
extern int log_vprint __P((struct log *, const char *, ...));
extern int log_vaprint __P((struct log *, const char *, va_list));
extern int log_close __P((struct log *));
extern void log_free __P((struct log *));

#endif /* _LOGGER_H */
