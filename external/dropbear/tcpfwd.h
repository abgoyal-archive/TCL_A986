#ifndef _TCPFWD_H
#define _TCPFWD_H

#include "channel.h"

struct TCPListener {

	/* For a direct-tcpip request, it's the addr/port we want the other
	 * end to connect to */
	unsigned char *sendaddr;
	unsigned int sendport;

	/* This is the address/port that we listen on. The address has special
	 * meanings as per the rfc, "" for all interfaces, "localhost" for 
	 * localhost, or a normal interface name. */
	unsigned char *listenaddr;
	unsigned int listenport;

	const struct ChanType *chantype;
	enum {direct, forwarded} tcp_type;
};

/* A link in a list of forwards */
struct TCPFwdList {

	const unsigned char* connectaddr;
	unsigned int connectport;
	unsigned int listenport;
	struct TCPFwdList * next;

};

/* Server */
void recv_msg_global_request_remotetcp();
extern const struct ChanType svr_chan_tcpdirect;

/* Client */
void setup_localtcp();
void setup_remotetcp();
extern const struct ChanType cli_chan_tcpremote;

/* Common */
int listen_tcpfwd(struct TCPListener* tcpinfo);


#endif
