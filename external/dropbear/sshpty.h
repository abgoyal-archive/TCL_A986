/*	$OpenBSD: sshpty.h,v 1.4 2002/03/04 17:27:39 stevesk Exp $	*/


#ifndef SSHPTY_H
#define SSHPTY_H

int	 pty_allocate(int *, int *, char *, int);
void	 pty_release(const char *);
void	 pty_make_controlling_tty(int *, const char *);
void	 pty_change_window_size(int, int, int, int, int);
void	 pty_setowner(struct passwd *, const char *);

#endif /* SSHPTY_H */
