
#include "includes.h"
#include "packet.h"
#include "session.h"
#include "dbutil.h"
#include "ssh.h"
#include "algo.h"
#include "buffer.h"
#include "kex.h"
#include "random.h"
#include "service.h"
#include "auth.h"
#include "channel.h"

#define MAX_UNAUTH_PACKET_TYPE SSH_MSG_USERAUTH_PK_OK

static void recv_unimplemented();

/* process a decrypted packet, call the appropriate handler */
void process_packet() {

	unsigned char type;
	unsigned int i;

	TRACE(("enter process_packet"))

	type = buf_getbyte(ses.payload);
	TRACE(("process_packet: packet type = %d", type))

	ses.lastpacket = type;

	/* These packets we can receive at any time */
	switch(type) {

		case SSH_MSG_IGNORE:
		case SSH_MSG_DEBUG:
			TRACE(("received SSH_MSG_IGNORE or SSH_MSG_DEBUG"))
			goto out;

		case SSH_MSG_UNIMPLEMENTED:
			/* debugging XXX */
			TRACE(("SSH_MSG_UNIMPLEMENTED"))
			dropbear_exit("received SSH_MSG_UNIMPLEMENTED");
			
		case SSH_MSG_DISCONNECT:
			/* TODO cleanup? */
			dropbear_close("Disconnect received");
	}


	/* This applies for KEX, where the spec says the next packet MUST be
	 * NEWKEYS */
	if (ses.requirenext != 0) {
		if (ses.requirenext != type) {
			/* TODO send disconnect? */
			dropbear_exit("unexpected packet type %d, expected %d", type,
					ses.requirenext);
		} else {
			/* Got what we expected */
			ses.requirenext = 0;
		}
	}

	/* Check if we should ignore this packet. Used currently only for
	 * KEX code, with first_kex_packet_follows */
	if (ses.ignorenext) {
		TRACE(("Ignoring packet, type = %d", type))
		ses.ignorenext = 0;
		goto out;
	}


	/* Kindly the protocol authors gave all the preauth packets type values
	 * less-than-or-equal-to 60 ( == MAX_UNAUTH_PACKET_TYPE ).
	 * NOTE: if the protocol changes and new types are added, revisit this 
	 * assumption */
	if ( !ses.authstate.authdone && type > MAX_UNAUTH_PACKET_TYPE ) {
		dropbear_exit("received message %d before userauth", type);
	}

	for (i = 0; ; i++) {
		if (ses.packettypes[i].type == 0) {
			/* end of list */
			break;
		}

		if (ses.packettypes[i].type == type) {
			ses.packettypes[i].handler();
			goto out;
		}
	}

	
	/* TODO do something more here? */
	TRACE(("preauth unknown packet"))
	recv_unimplemented();

out:
	buf_free(ses.payload);
	ses.payload = NULL;

	TRACE(("leave process_packet"))
}



static void recv_unimplemented() {

	CHECKCLEARTOWRITE();

	buf_putbyte(ses.writepayload, SSH_MSG_UNIMPLEMENTED);
	/* the decryption routine increments the sequence number, we must
	 * decrement */
	buf_putint(ses.writepayload, ses.recvseq - 1);

	encrypt_packet();
}
