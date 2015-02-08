
/* Validates a user password */

#include "includes.h"
#include "session.h"
#include "buffer.h"
#include "dbutil.h"
#include "auth.h"

#ifdef ENABLE_SVR_PASSWORD_AUTH

void svr_auth_password() {
	
#ifdef HAVE_SHADOW_H
	struct spwd *spasswd = NULL;
#endif
	char * passwdcrypt = NULL; /* the crypt from /etc/passwd or /etc/shadow */
	char * testcrypt = NULL; /* crypt generated from the user's password sent */
	unsigned char * password;
	unsigned int passwordlen;

	unsigned int changepw;

	passwdcrypt = ses.authstate.pw->pw_passwd;
#ifdef HAVE_SHADOW_H
	/* get the shadow password if possible */
	spasswd = getspnam(ses.authstate.printableuser);
	if (spasswd != NULL && spasswd->sp_pwdp != NULL) {
		passwdcrypt = spasswd->sp_pwdp;
	}
#endif

#ifdef DEBUG_HACKCRYPT
	/* debugging crypt for non-root testing with shadows */
	passwdcrypt = DEBUG_HACKCRYPT;
#endif

	/* check for empty password - need to do this again here
	 * since the shadow password may differ to that tested
	 * in auth.c */
	if (passwdcrypt[0] == '\0') {
		dropbear_log(LOG_WARNING, "user '%s' has blank password, rejected",
				ses.authstate.printableuser);
		send_msg_userauth_failure(0, 1);
		return;
	}

	/* check if client wants to change password */
	changepw = buf_getbool(ses.payload);
	if (changepw) {
		/* not implemented by this server */
		send_msg_userauth_failure(0, 1);
		return;
	}

	password = buf_getstring(ses.payload, &passwordlen);

	/* the first bytes of passwdcrypt are the salt */
	testcrypt = crypt((char*)password, passwdcrypt);
	m_burn(password, passwordlen);
	m_free(password);

	if (strcmp(testcrypt, passwdcrypt) == 0) {
		/* successful authentication */
		dropbear_log(LOG_NOTICE, 
				"password auth succeeded for '%s' from %s",
				ses.authstate.printableuser,
				svr_ses.addrstring);
		send_msg_userauth_success();
	} else {
		dropbear_log(LOG_WARNING,
				"bad password attempt for '%s' from %s",
				ses.authstate.printableuser,
				svr_ses.addrstring);
		send_msg_userauth_failure(0, 1);
	}

}

#endif
