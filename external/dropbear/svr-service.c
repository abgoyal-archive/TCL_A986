
#include "includes.h"
#include "dbutil.h"
#include "service.h"
#include "session.h"
#include "packet.h"
#include "ssh.h"
#include "auth.h"

static void send_msg_service_accept(unsigned char *name, int len);

void recv_msg_service_request() {

	unsigned char * name;
	unsigned int len;

	TRACE(("enter recv_msg_service_request"))

	name = buf_getstring(ses.payload, &len);

	/* ssh-userauth */
	if (len == SSH_SERVICE_USERAUTH_LEN && 
			strncmp(SSH_SERVICE_USERAUTH, name, len) == 0) {

		send_msg_service_accept(name, len);
		m_free(name);
		TRACE(("leave recv_msg_service_request: done ssh-userauth"))
		return;
	}

	/* ssh-connection */
	if (len == SSH_SERVICE_CONNECTION_LEN &&
			(strncmp(SSH_SERVICE_CONNECTION, name, len) == 0)) {
		if (ses.authstate.authdone != 1) {
			dropbear_exit("request for connection before auth");
		}

		send_msg_service_accept(name, len);
		m_free(name);
		TRACE(("leave recv_msg_service_request: done ssh-connection"))
		return;
	}

	m_free(name);
	/* TODO this should be a MSG_DISCONNECT */
	dropbear_exit("unrecognised SSH_MSG_SERVICE_REQUEST");


}

static void send_msg_service_accept(unsigned char *name, int len) {

	TRACE(("accepting service %s", name))

	CHECKCLEARTOWRITE();

	buf_putbyte(ses.writepayload, SSH_MSG_SERVICE_ACCEPT);
	buf_putstring(ses.writepayload, name, len);

	encrypt_packet();

}
