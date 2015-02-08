
#include "includes.h"
#include "buffer.h"
#include "dbutil.h"
#include "session.h"
#include "ssh.h"
#include "runopts.h"

#ifdef ENABLE_CLI_PASSWORD_AUTH

#ifdef ENABLE_CLI_ASKPASS_HELPER
/* Returns 1 if we want to use the askpass program, 0 otherwise */
static int want_askpass()
{
	char* askpass_prog = NULL;

	askpass_prog = getenv("SSH_ASKPASS");
	return askpass_prog && !isatty(STDIN_FILENO) && getenv("DISPLAY");
}

static char *gui_getpass(const char *prompt) {

	pid_t pid;
	int p[2], maxlen, len, status;
	static char buf[DROPBEAR_MAX_CLI_PASS + 1];
	char* helper = NULL;

	TRACE(("enter gui_getpass"))

	helper = getenv("SSH_ASKPASS");
	if (!helper)
	{
		TRACE(("leave gui_getpass: no askpass program"))
		return NULL;
	}

	if (pipe(p) < 0) {
		TRACE(("error creating child pipe"))
		return NULL;
	}

	pid = fork();

	if (pid < 0) {
		TRACE(("fork error"))
		return NULL;
	}

	if (!pid) {
		/* child */
		close(p[0]);
		if (dup2(p[1], STDOUT_FILENO) < 0) {
			TRACE(("error redirecting stdout"))
			exit(1);
		}
		close(p[1]);
		execlp(helper, helper, prompt, (char *)0);
		TRACE(("execlp error"))
		exit(1);
	}

	close(p[1]);
	maxlen = sizeof(buf);
	while (maxlen > 0) {
		len = read(p[0], buf + sizeof(buf) - maxlen, maxlen);
		if (len > 0) {
			maxlen -= len;
		} else {
			if (errno != EINTR)
				break;
		}
	}

	close(p[0]);

	while (waitpid(pid, &status, 0) < 0 && errno == EINTR)
		;
	if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
		return(NULL);

	len = sizeof(buf) - maxlen;
	buf[len] = '\0';
	if (len > 0 && buf[len - 1] == '\n')
		buf[len - 1] = '\0';

	TRACE(("leave gui_getpass"))
	return(buf);
}
#endif /* ENABLE_CLI_ASKPASS_HELPER */

void cli_auth_password() {

	char* password = NULL;
	char prompt[80];

	TRACE(("enter cli_auth_password"))
	CHECKCLEARTOWRITE();

	snprintf(prompt, sizeof(prompt), "%s@%s's password: ", 
				cli_opts.username, cli_opts.remotehost);
#ifdef ENABLE_CLI_ASKPASS_HELPER
	if (want_askpass())
		password = gui_getpass(prompt);
	else
#endif
		password = getpass_or_cancel(prompt);

	buf_putbyte(ses.writepayload, SSH_MSG_USERAUTH_REQUEST);

	buf_putstring(ses.writepayload, cli_opts.username,
			strlen(cli_opts.username));

	buf_putstring(ses.writepayload, SSH_SERVICE_CONNECTION, 
			SSH_SERVICE_CONNECTION_LEN);

	buf_putstring(ses.writepayload, AUTH_METHOD_PASSWORD, 
			AUTH_METHOD_PASSWORD_LEN);

	buf_putbyte(ses.writepayload, 0); /* FALSE - so says the spec */

	buf_putstring(ses.writepayload, password, strlen(password));

	encrypt_packet();
	m_burn(password, strlen(password));

	TRACE(("leave cli_auth_password"))
}
#endif	/* ENABLE_CLI_PASSWORD_AUTH */
