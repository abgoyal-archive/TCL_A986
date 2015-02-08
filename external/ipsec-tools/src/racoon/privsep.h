/*	$NetBSD: privsep.h,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/* Id: privsep.h,v 1.5 2005/06/07 12:22:11 fredsen Exp */


#ifndef _PRIVSEP_H
#define _PRIVSEP_H

#define PRIVSEP_EAY_GET_PKCS1PRIVKEY	0x0801	/* admin_com_bufs follows */
#define PRIVSEP_SCRIPT_EXEC		0x0803	/* admin_com_bufs follows */
#define PRIVSEP_GETPSK			0x0804	/* admin_com_bufs follows */
#define PRIVSEP_XAUTH_LOGIN_SYSTEM	0x0805	/* admin_com_bufs follows */
#define PRIVSEP_ACCOUNTING_PAM		0x0806	/* admin_com_bufs follows */
#define PRIVSEP_XAUTH_LOGIN_PAM		0x0807	/* admin_com_bufs follows */
#define PRIVSEP_CLEANUP_PAM		0x0808	/* admin_com_bufs follows */
#define PRIVSEP_ACCOUNTING_SYSTEM	0x0809	/* admin_com_bufs follows */

#define PRIVSEP_NBUF_MAX 24
#define PRIVSEP_BUFLEN_MAX 4096
struct admin_com_bufs {
	size_t buflen[PRIVSEP_NBUF_MAX];
	/* Followed by the buffers */
};

struct privsep_com_msg {
	struct admin_com hdr;
	struct admin_com_bufs bufs;
};

int privsep_init __P((void));

vchar_t *privsep_eay_get_pkcs1privkey __P((char *));
int privsep_pfkey_open __P((void));
void privsep_pfkey_close __P((int));
int privsep_script_exec __P((char *, int, char * const *));
vchar_t *privsep_getpsk __P((const char *, const int));
int privsep_xauth_login_system __P((char *, char *));
#ifdef HAVE_LIBPAM
int privsep_accounting_pam __P((int, int));
int privsep_xauth_login_pam __P((int, struct sockaddr *, char *, char *));
void privsep_cleanup_pam __P((int));
#endif
int privsep_accounting_system __P((int, struct sockaddr *, char *, int));
#endif /* _PRIVSEP_H */
