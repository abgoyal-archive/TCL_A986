
#ifndef _RUNOPTS_H_
#define _RUNOPTS_H_

#include "includes.h"
#include "signkey.h"
#include "buffer.h"
#include "auth.h"
#include "tcpfwd.h"

typedef struct runopts {

#if defined(ENABLE_SVR_REMOTETCPFWD) || defined(ENABLE_CLI_LOCALTCPFWD)
	int listen_fwd_all;
#endif

} runopts;

extern runopts opts;

int readhostkey(const char * filename, sign_key * hostkey, int *type);

typedef struct svr_runopts {

	char * rsakeyfile;
	char * dsskeyfile;
	char * bannerfile;

	int forkbg;
	int usingsyslog;

	/* ports is an array of the portcount listening ports */
	char *ports[DROPBEAR_MAX_PORTS];
	unsigned int portcount;
	char *addresses[DROPBEAR_MAX_PORTS];

	int inetdmode;

	/* Flags indicating whether to use ipv4 and ipv6 */
	/* not used yet
	int ipv4;
	int ipv6;
	*/

#ifdef DO_MOTD
	/* whether to print the MOTD */
	int domotd;
#endif

	int norootlogin;

	int noauthpass;
	int norootpass;

#ifdef ENABLE_SVR_REMOTETCPFWD
	int noremotetcp;
#endif
#ifdef ENABLE_SVR_LOCALTCPFWD
	int nolocaltcp;
#endif

	sign_key *hostkey;
	buffer * banner;
	char * pidfile;

} svr_runopts;

extern svr_runopts svr_opts;

void svr_getopts(int argc, char ** argv);
void loadhostkeys();

typedef struct cli_runopts {

	char *progname;
	char *remotehost;
	char *remoteport;

	char *username;

	char *cmd;
	int wantpty;
	int always_accept_key;
	int no_cmd;
	int backgrounded;
#ifdef ENABLE_CLI_PUBKEY_AUTH
	struct SignKeyList *privkeys; /* Keys to use for public-key auth */
#endif
#ifdef ENABLE_CLI_REMOTETCPFWD
	struct TCPFwdList * remotefwds;
#endif
#ifdef ENABLE_CLI_LOCALTCPFWD
	struct TCPFwdList * localfwds;
#endif

} cli_runopts;

extern cli_runopts cli_opts;
void cli_getopts(int argc, char ** argv);

#endif /* _RUNOPTS_H_ */
