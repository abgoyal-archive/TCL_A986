
/* Process a pubkey auth request */

#include "includes.h"
#include "session.h"
#include "dbutil.h"
#include "buffer.h"
#include "signkey.h"
#include "auth.h"
#include "ssh.h"
#include "packet.h"
#include "algo.h"

#ifdef ENABLE_SVR_PUBKEY_AUTH

#define MIN_AUTHKEYS_LINE 10 /* "ssh-rsa AB" - short but doesn't matter */
#define MAX_AUTHKEYS_LINE 4200 /* max length of a line in authkeys */

static int checkpubkey(unsigned char* algo, unsigned int algolen,
		unsigned char* keyblob, unsigned int keybloblen);
static int checkpubkeyperms();
static void send_msg_userauth_pk_ok(unsigned char* algo, unsigned int algolen,
		unsigned char* keyblob, unsigned int keybloblen);
static int checkfileperm(char * filename);

void svr_auth_pubkey() {

	unsigned char testkey; /* whether we're just checking if a key is usable */
	unsigned char* algo = NULL; /* pubkey algo */
	unsigned int algolen;
	unsigned char* keyblob = NULL;
	unsigned int keybloblen;
	buffer * signbuf = NULL;
	sign_key * key = NULL;
	char* fp = NULL;
	int type = -1;

	TRACE(("enter pubkeyauth"))

	/* 0 indicates user just wants to check if key can be used, 1 is an
	 * actual attempt*/
	testkey = (buf_getbool(ses.payload) == 0);

	algo = buf_getstring(ses.payload, &algolen);
	keybloblen = buf_getint(ses.payload);
	keyblob = buf_getptr(ses.payload, keybloblen);

	/* check if the key is valid */
	if (checkpubkey(algo, algolen, keyblob, keybloblen) == DROPBEAR_FAILURE) {
		send_msg_userauth_failure(0, 0);
		goto out;
	}

	/* let them know that the key is ok to use */
	if (testkey) {
		send_msg_userauth_pk_ok(algo, algolen, keyblob, keybloblen);
		goto out;
	}

	/* now we can actually verify the signature */
	
	/* get the key */
	key = new_sign_key();
	type = DROPBEAR_SIGNKEY_ANY;
	if (buf_get_pub_key(ses.payload, key, &type) == DROPBEAR_FAILURE) {
		send_msg_userauth_failure(0, 1);
		goto out;
	}

	/* create the data which has been signed - this a string containing
	 * session_id, concatenated with the payload packet up to the signature */
	signbuf = buf_new(ses.payload->pos + 4 + SHA1_HASH_SIZE);
	buf_putstring(signbuf, ses.session_id, SHA1_HASH_SIZE);
	buf_putbytes(signbuf, ses.payload->data, ses.payload->pos);
	buf_setpos(signbuf, 0);

	/* ... and finally verify the signature */
	fp = sign_key_fingerprint(keyblob, keybloblen);
	if (buf_verify(ses.payload, key, buf_getptr(signbuf, signbuf->len),
				signbuf->len) == DROPBEAR_SUCCESS) {
		dropbear_log(LOG_NOTICE,
				"pubkey auth succeeded for '%s' with key %s from %s",
				ses.authstate.printableuser, fp, svr_ses.addrstring);
		send_msg_userauth_success();
	} else {
		dropbear_log(LOG_WARNING,
				"pubkey auth bad signature for '%s' with key %s from %s",
				ses.authstate.printableuser, fp, svr_ses.addrstring);
		send_msg_userauth_failure(0, 1);
	}
	m_free(fp);

out:
	/* cleanup stuff */
	if (signbuf) {
		buf_free(signbuf);
	}
	if (algo) {
		m_free(algo);
	}
	if (key) {
		sign_key_free(key);
		key = NULL;
	}
	TRACE(("leave pubkeyauth"))
}

static void send_msg_userauth_pk_ok(unsigned char* algo, unsigned int algolen,
		unsigned char* keyblob, unsigned int keybloblen) {

	TRACE(("enter send_msg_userauth_pk_ok"))
	CHECKCLEARTOWRITE();

	buf_putbyte(ses.writepayload, SSH_MSG_USERAUTH_PK_OK);
	buf_putstring(ses.writepayload, algo, algolen);
	buf_putstring(ses.writepayload, keyblob, keybloblen);

	encrypt_packet();
	TRACE(("leave send_msg_userauth_pk_ok"))

}

/* Returns DROPBEAR_SUCCESS if key is ok for auth, DROPBEAR_FAILURE otherwise */
static int checkpubkey(unsigned char* algo, unsigned int algolen,
		unsigned char* keyblob, unsigned int keybloblen) {

	FILE * authfile = NULL;
	char * filename = NULL;
	int ret = DROPBEAR_FAILURE;
	buffer * line = NULL;
	unsigned int len, pos;
	
	TRACE(("enter checkpubkey"))

	/* check that we can use the algo */
	if (have_algo(algo, algolen, sshhostkey) == DROPBEAR_FAILURE) {
		dropbear_log(LOG_WARNING,
				"pubkey auth attempt with unknown algo for '%s' from %s",
				ses.authstate.printableuser, svr_ses.addrstring);
		goto out;
	}

	/* check file permissions, also whether file exists */
	if (checkpubkeyperms() == DROPBEAR_FAILURE) {
		TRACE(("bad authorized_keys permissions, or file doesn't exist"))
		goto out;
	}

	/* we don't need to check pw and pw_dir for validity, since
	 * its been done in checkpubkeyperms. */
	len = strlen(ses.authstate.pw->pw_dir);
	/* allocate max required pathname storage,
	 * = path + "/.ssh/authorized_keys" + '\0' = pathlen + 22 */
	filename = m_malloc(len + 22);
	snprintf(filename, len + 22, "%s/.ssh/authorized_keys", 
				ses.authstate.pw->pw_dir);

	/* open the file */
	authfile = fopen(filename, "r");
	if (authfile == NULL) {
		goto out;
	}
	TRACE(("checkpubkey: opened authorized_keys OK"))

	line = buf_new(MAX_AUTHKEYS_LINE);

	/* iterate through the lines */
	do {

		if (buf_getline(line, authfile) == DROPBEAR_FAILURE) {
			/* EOF reached */
			TRACE(("checkpubkey: authorized_keys EOF reached"))
			break;
		}

		if (line->len < MIN_AUTHKEYS_LINE) {
			TRACE(("checkpubkey: line too short"))
			continue; /* line is too short for it to be a valid key */
		}

		/* check the key type - this also stops us from using keys
		 * which have options with them */
		if (strncmp(buf_getptr(line, algolen), algo, algolen) != 0) {
			continue;
		}
		buf_incrpos(line, algolen);
		
		/* check for space (' ') character */
		if (buf_getbyte(line) != ' ') {
			TRACE(("checkpubkey: space character expected, isn't there"))
			continue;
		}

		/* truncate the line at the space after the base64 data */
		pos = line->pos;
		for (len = 0; line->pos < line->len; len++) {
			if (buf_getbyte(line) == ' ') break;
		}	
		buf_setpos(line, pos);
		buf_setlen(line, line->pos + len);

		TRACE(("checkpubkey: line pos = %d len = %d", line->pos, line->len))

		ret = cmp_base64_key(keyblob, keybloblen, algo, algolen, line, NULL);
		if (ret == DROPBEAR_SUCCESS) {
			break;
		}

		/* We continue to the next line otherwise */
		
	} while (1);

out:
	if (authfile) {
		fclose(authfile);
	}
	if (line) {
		buf_free(line);
	}
	m_free(filename);
	TRACE(("leave checkpubkey: ret=%d", ret))
	return ret;
}


static int checkpubkeyperms() {

	char* filename = NULL; 
	int ret = DROPBEAR_FAILURE;
	unsigned int len;

	TRACE(("enter checkpubkeyperms"))

	if (ses.authstate.pw->pw_dir == NULL) {
		goto out;
	}

	if ((len = strlen(ses.authstate.pw->pw_dir)) == 0) {
		goto out;
	}

	/* allocate max required pathname storage,
	 * = path + "/.ssh/authorized_keys" + '\0' = pathlen + 22 */
	filename = m_malloc(len + 22);
	strncpy(filename, ses.authstate.pw->pw_dir, len+1);

	/* check ~ */
	if (checkfileperm(filename) != DROPBEAR_SUCCESS) {
		goto out;
	}

	/* check ~/.ssh */
	strncat(filename, "/.ssh", 5); /* strlen("/.ssh") == 5 */
	if (checkfileperm(filename) != DROPBEAR_SUCCESS) {
		goto out;
	}

	/* now check ~/.ssh/authorized_keys */
	strncat(filename, "/authorized_keys", 16);
	if (checkfileperm(filename) != DROPBEAR_SUCCESS) {
		goto out;
	}

	/* file looks ok, return success */
	ret = DROPBEAR_SUCCESS;
	
out:
	m_free(filename);

	TRACE(("leave checkpubkeyperms"))
	return ret;
}

/* returns DROPBEAR_SUCCESS or DROPBEAR_FAILURE */
static int checkfileperm(char * filename) {
	struct stat filestat;
	int badperm = 0;

	TRACE(("enter checkfileperm(%s)", filename))

	if (stat(filename, &filestat) != 0) {
		TRACE(("leave checkfileperm: stat() != 0"))
		return DROPBEAR_FAILURE;
	}
	/* check ownership - user or root only*/
	if (filestat.st_uid != ses.authstate.pw->pw_uid
			&& filestat.st_uid != 0) {
		badperm = 1;
		TRACE(("wrong ownership"))
	}
	/* check permissions - don't want group or others +w */
	if (filestat.st_mode & (S_IWGRP | S_IWOTH)) {
		badperm = 1;
		TRACE(("wrong perms"))
	}
	if (badperm) {
		if (!ses.authstate.perm_warn) {
			ses.authstate.perm_warn = 1;
			dropbear_log(LOG_INFO, "%s must be owned by user or root, and not writable by others", filename);
		}
		TRACE(("leave checkfileperm: failure perms/owner"))
		return DROPBEAR_FAILURE;
	}

	TRACE(("leave checkfileperm: success"))
	return DROPBEAR_SUCCESS;
}


#endif 
