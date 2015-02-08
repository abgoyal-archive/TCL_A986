
#include "includes.h"
#include "dbutil.h"
#include "algo.h"
#include "buffer.h"
#include "session.h"
#include "kex.h"
#include "ssh.h"
#include "packet.h"
#include "bignum.h"
#include "random.h"
#include "runopts.h"


static void send_msg_kexdh_reply(mp_int *dh_e);

void recv_msg_kexdh_init() {

	DEF_MP_INT(dh_e);

	TRACE(("enter recv_msg_kexdh_init"))
	if (!ses.kexstate.recvkexinit) {
		dropbear_exit("Premature kexdh_init message received");
	}

	m_mp_init(&dh_e);
	if (buf_getmpint(ses.payload, &dh_e) != DROPBEAR_SUCCESS) {
		dropbear_exit("Failed to get kex value");
	}

	send_msg_kexdh_reply(&dh_e);

	mp_clear(&dh_e);

	send_msg_newkeys();
	ses.requirenext = SSH_MSG_NEWKEYS;
	TRACE(("leave recv_msg_kexdh_init"))
}
	
static void send_msg_kexdh_reply(mp_int *dh_e) {

	DEF_MP_INT(dh_y);
	DEF_MP_INT(dh_f);

	TRACE(("enter send_msg_kexdh_reply"))
	m_mp_init_multi(&dh_y, &dh_f, NULL);
	
	gen_kexdh_vals(&dh_f, &dh_y);

	kexdh_comb_key(&dh_f, &dh_y, dh_e, svr_opts.hostkey);
	mp_clear(&dh_y);

	/* we can start creating the kexdh_reply packet */
	CHECKCLEARTOWRITE();
	buf_putbyte(ses.writepayload, SSH_MSG_KEXDH_REPLY);
	buf_put_pub_key(ses.writepayload, svr_opts.hostkey,
			ses.newkeys->algo_hostkey);

	/* put f */
	buf_putmpint(ses.writepayload, &dh_f);
	mp_clear(&dh_f);

	/* calc the signature */
	buf_put_sign(ses.writepayload, svr_opts.hostkey, 
			ses.newkeys->algo_hostkey, ses.hash, SHA1_HASH_SIZE);

	/* the SSH_MSG_KEXDH_REPLY is done */
	encrypt_packet();

	TRACE(("leave send_msg_kexdh_reply"))
}

