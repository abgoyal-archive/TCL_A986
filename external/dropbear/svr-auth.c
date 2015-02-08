

#include "includes.h"
#include "dbutil.h"
#include "session.h"
#include "buffer.h"
#include "ssh.h"
#include "packet.h"
#include "auth.h"
#include "runopts.h"

static void authclear();
static int checkusername(unsigned char *username, unsigned int userlen);
static void send_msg_userauth_banner();

/* initialise the first time for a session, resetting all parameters */
void svr_authinitialise() {

	ses.authstate.failcount = 0;
	authclear();
	
}

static void authclear() {
	
	memset(&ses.authstate, 0, sizeof(ses.authstate));
#ifdef ENABLE_SVR_PUBKEY_AUTH
	ses.authstate.authtypes |= AUTH_TYPE_PUBKEY;
#endif
#if defined(ENABLE_SVR_PASSWORD_AUTH) || defined(ENABLE_SVR_PAM_AUTH)
	if (!svr_opts.noauthpass) {
		ses.authstate.authtypes |= AUTH_TYPE_PASSWORD;
	}
#endif

}

static void send_msg_userauth_banner() {

	TRACE(("enter send_msg_userauth_banner"))
	if (svr_opts.banner == NULL) {
		TRACE(("leave send_msg_userauth_banner: banner is NULL"))
		return;
	}

	CHECKCLEARTOWRITE();

	buf_putbyte(ses.writepayload, SSH_MSG_USERAUTH_BANNER);
	buf_putstring(ses.writepayload, buf_getptr(svr_opts.banner,
				svr_opts.banner->len), svr_opts.banner->len);
	buf_putstring(ses.writepayload, "en", 2);

	encrypt_packet();
	buf_free(svr_opts.banner);
	svr_opts.banner = NULL;

	TRACE(("leave send_msg_userauth_banner"))
}

void recv_msg_userauth_request() {

	unsigned char *username = NULL, *servicename = NULL, *methodname = NULL;
	unsigned int userlen, servicelen, methodlen;

	TRACE(("enter recv_msg_userauth_request"))

	/* ignore packets if auth is already done */
	if (ses.authstate.authdone == 1) {
		TRACE(("leave recv_msg_userauth_request: authdone already"))
		return;
	}

	/* send the banner if it exists, it will only exist once */
	if (svr_opts.banner) {
		send_msg_userauth_banner();
	}

	
	username = buf_getstring(ses.payload, &userlen);
	servicename = buf_getstring(ses.payload, &servicelen);
	methodname = buf_getstring(ses.payload, &methodlen);

	/* only handle 'ssh-connection' currently */
	if (servicelen != SSH_SERVICE_CONNECTION_LEN
			&& (strncmp(servicename, SSH_SERVICE_CONNECTION,
					SSH_SERVICE_CONNECTION_LEN) != 0)) {
		
		/* TODO - disconnect here */
		m_free(username);
		m_free(servicename);
		m_free(methodname);
		dropbear_exit("unknown service in auth");
	}

	/* user wants to know what methods are supported */
	if (methodlen == AUTH_METHOD_NONE_LEN &&
			strncmp(methodname, AUTH_METHOD_NONE,
				AUTH_METHOD_NONE_LEN) == 0) {
		TRACE(("recv_msg_userauth_request: 'none' request"))
		send_msg_userauth_failure(0, 0);
		goto out;
	}
	
	/* check username is good before continuing */
	if (checkusername(username, userlen) == DROPBEAR_FAILURE) {
		/* username is invalid/no shell/etc - send failure */
		TRACE(("sending checkusername failure"))
		send_msg_userauth_failure(0, 1);
		goto out;
	}

#ifdef ENABLE_SVR_PASSWORD_AUTH
	if (!svr_opts.noauthpass &&
			!(svr_opts.norootpass && ses.authstate.pw->pw_uid == 0) ) {
		/* user wants to try password auth */
		if (methodlen == AUTH_METHOD_PASSWORD_LEN &&
				strncmp(methodname, AUTH_METHOD_PASSWORD,
					AUTH_METHOD_PASSWORD_LEN) == 0) {
			svr_auth_password();
			goto out;
		}
	}
#endif

#ifdef ENABLE_SVR_PAM_AUTH
	if (!svr_opts.noauthpass &&
			!(svr_opts.norootpass && ses.authstate.pw->pw_uid == 0) ) {
		/* user wants to try password auth */
		if (methodlen == AUTH_METHOD_PASSWORD_LEN &&
				strncmp(methodname, AUTH_METHOD_PASSWORD,
					AUTH_METHOD_PASSWORD_LEN) == 0) {
			svr_auth_pam();
			goto out;
		}
	}
#endif

#ifdef ENABLE_SVR_PUBKEY_AUTH
	/* user wants to try pubkey auth */
	if (methodlen == AUTH_METHOD_PUBKEY_LEN &&
			strncmp(methodname, AUTH_METHOD_PUBKEY,
				AUTH_METHOD_PUBKEY_LEN) == 0) {
		svr_auth_pubkey();
		goto out;
	}
#endif

	/* nothing matched, we just fail */
	send_msg_userauth_failure(0, 1);

out:

	m_free(username);
	m_free(servicename);
	m_free(methodname);
}

static int checkusername(unsigned char *username, unsigned int userlen) {

	char* listshell = NULL;
	char* usershell = NULL;
	
	TRACE(("enter checkusername"))
	if (userlen > MAX_USERNAME_LEN) {
		return DROPBEAR_FAILURE;
	}

	/* new user or username has changed */
	if (ses.authstate.username == NULL ||
		strcmp(username, ses.authstate.username) != 0) {
			/* the username needs resetting */
			if (ses.authstate.username != NULL) {
				dropbear_log(LOG_WARNING, "client trying multiple usernames from %s",
							svr_ses.addrstring);
				m_free(ses.authstate.username);
			}
			authclear();
			ses.authstate.pw = getpwnam((char*)username);
			ses.authstate.username = m_strdup(username);
			m_free(ses.authstate.printableuser);
	}

	/* check that user exists */
	if (ses.authstate.pw == NULL) {
		TRACE(("leave checkusername: user '%s' doesn't exist", username))
		dropbear_log(LOG_WARNING,
				"login attempt for nonexistent user from %s",
				svr_ses.addrstring);
		send_msg_userauth_failure(0, 1);
		return DROPBEAR_FAILURE;
	}

	/* We can set it once we know its a real user */
	ses.authstate.printableuser = m_strdup(ses.authstate.pw->pw_name);

	/* check for non-root if desired */
	if (svr_opts.norootlogin && ses.authstate.pw->pw_uid == 0) {
		TRACE(("leave checkusername: root login disabled"))
		dropbear_log(LOG_WARNING, "root login rejected");
		send_msg_userauth_failure(0, 1);
		return DROPBEAR_FAILURE;
	}

	/* check for an empty password */
	if (ses.authstate.pw->pw_passwd[0] == '\0') {
		TRACE(("leave checkusername: empty pword"))
		dropbear_log(LOG_WARNING, "user '%s' has blank password, rejected",
				ses.authstate.printableuser);
		send_msg_userauth_failure(0, 1);
		return DROPBEAR_FAILURE;
	}

	TRACE(("shell is %s", ses.authstate.pw->pw_shell))

	/* check that the shell is set */
	usershell = ses.authstate.pw->pw_shell;
	if (usershell[0] == '\0') {
		/* empty shell in /etc/passwd means /bin/sh according to passwd(5) */
		usershell = "/bin/sh";
	}

	/* check the shell is valid. If /etc/shells doesn't exist, getusershell()
	 * should return some standard shells like "/bin/sh" and "/bin/csh" (this
	 * is platform-specific) */
	setusershell();
	while ((listshell = getusershell()) != NULL) {
		TRACE(("test shell is '%s'", listshell))
		if (strcmp(listshell, usershell) == 0) {
			/* have a match */
			goto goodshell;
		}
	}
	/* no matching shell */
	endusershell();
	TRACE(("no matching shell"))
	dropbear_log(LOG_WARNING, "user '%s' has invalid shell, rejected",
				ses.authstate.printableuser);
	send_msg_userauth_failure(0, 1);
	return DROPBEAR_FAILURE;
	
goodshell:
	endusershell();
	TRACE(("matching shell"))

	TRACE(("uid = %d", ses.authstate.pw->pw_uid))
	TRACE(("leave checkusername"))
	return DROPBEAR_SUCCESS;

}

void send_msg_userauth_failure(int partial, int incrfail) {

	buffer *typebuf = NULL;

	TRACE(("enter send_msg_userauth_failure"))

	CHECKCLEARTOWRITE();
	
	buf_putbyte(ses.writepayload, SSH_MSG_USERAUTH_FAILURE);

	/* put a list of allowed types */
	typebuf = buf_new(30); /* long enough for PUBKEY and PASSWORD */

	if (ses.authstate.authtypes & AUTH_TYPE_PUBKEY) {
		buf_putbytes(typebuf, AUTH_METHOD_PUBKEY, AUTH_METHOD_PUBKEY_LEN);
		if (ses.authstate.authtypes & AUTH_TYPE_PASSWORD) {
			buf_putbyte(typebuf, ',');
		}
	}
	
	if (ses.authstate.authtypes & AUTH_TYPE_PASSWORD) {
		buf_putbytes(typebuf, AUTH_METHOD_PASSWORD, AUTH_METHOD_PASSWORD_LEN);
	}

	buf_setpos(typebuf, 0);
	buf_putstring(ses.writepayload, buf_getptr(typebuf, typebuf->len),
			typebuf->len);

	TRACE(("auth fail: methods %d, '%s'", ses.authstate.authtypes,
				buf_getptr(typebuf, typebuf->len)));

	buf_free(typebuf);

	buf_putbyte(ses.writepayload, partial ? 1 : 0);
	encrypt_packet();

	if (incrfail) {
		usleep(300000); /* XXX improve this */
		ses.authstate.failcount++;
	}

	if (ses.authstate.failcount >= MAX_AUTH_TRIES) {
		char * userstr;
		/* XXX - send disconnect ? */
		TRACE(("Max auth tries reached, exiting"))

		if (ses.authstate.printableuser == NULL) {
			userstr = "is invalid";
		} else {
			userstr = ses.authstate.printableuser;
		}
		dropbear_exit("Max auth tries reached - user '%s' from %s",
				userstr, svr_ses.addrstring);
	}
	
	TRACE(("leave send_msg_userauth_failure"))
}

/* Send a success message to the user, and set the "authdone" flag */
void send_msg_userauth_success() {

	TRACE(("enter send_msg_userauth_success"))

	CHECKCLEARTOWRITE();

	buf_putbyte(ses.writepayload, SSH_MSG_USERAUTH_SUCCESS);
	encrypt_packet();

	ses.authstate.authdone = 1;
	ses.connecttimeout = 0;


	if (ses.authstate.pw->pw_uid == 0) {
		ses.allowprivport = 1;
	}

	/* Remove from the list of pre-auth sockets. Should be m_close(), since if
	 * we fail, we might end up leaking connection slots, and disallow new
	 * logins - a nasty situation. */							
	m_close(svr_ses.childpipe);

	TRACE(("leave send_msg_userauth_success"))

}
