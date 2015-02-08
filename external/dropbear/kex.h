
#ifndef _KEX_H_
#define _KEX_H_

#include "includes.h"
#include "algo.h"

void send_msg_kexinit();
void recv_msg_kexinit();
void send_msg_newkeys();
void recv_msg_newkeys();
void kexfirstinitialise();
void gen_kexdh_vals(mp_int *dh_pub, mp_int *dh_priv);
void kexdh_comb_key(mp_int *dh_pub_us, mp_int *dh_priv, mp_int *dh_pub_them,
		sign_key *hostkey);

void recv_msg_kexdh_init(); /* server */

void send_msg_kexdh_init(); /* client */
void recv_msg_kexdh_reply(); /* client */

struct KEXState {

	unsigned sentkexinit : 1; /*set when we've sent/recv kexinit packet */
	unsigned recvkexinit : 1;
	unsigned firstfollows : 1; /* true when first_kex_packet_follows is set */
	unsigned sentnewkeys : 1; /* set once we've send/recv'ed MSG_NEWKEYS*/
	unsigned recvnewkeys : 1;

	unsigned donefirstkex : 1; /* Set to 1 after the first kex has completed,
								  ie the transport layer has been set up */

	long lastkextime; /* time of the last kex */
	unsigned int datatrans; /* data transmitted since last kex */
	unsigned int datarecv; /* data received since last kex */

};

#define MAX_KEXHASHBUF 2000

#endif /* _KEX_H_ */
