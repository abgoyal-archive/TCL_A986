
#ifndef _CHANSESSION_H_
#define _CHANSESSION_H_

#include "loginrec.h"
#include "channel.h"
#include "listener.h"

struct exitinfo {

	int exitpid; /* -1 if not exited */
	int exitstatus;
	int exitsignal;
	int exitcore;
};

struct ChanSess {

	unsigned char * cmd; /* command to exec */
	pid_t pid; /* child process pid */

	/* pty details */
	int master; /* the master terminal fd*/
	int slave;
	unsigned char * tty;
	unsigned char * term;

	/* exit details */
	struct exitinfo exit;
	
#ifndef DISABLE_X11FWD
	struct Listener * x11listener;
	int x11port;
	char * x11authprot;
	char * x11authcookie;
	unsigned int x11screennum;
	unsigned char x11singleconn;
#endif

#ifndef DISABLE_AGENTFWD
	struct Listener * agentlistener;
	char * agentfile;
	char * agentdir;
#endif
};

struct ChildPid {
	pid_t pid;
	struct ChanSess * chansess;
};


void addnewvar(const char* param, const char* var);

void cli_send_chansess_request();
void cli_tty_cleanup();
void cli_chansess_winchange();

void svr_chansessinitialise();
extern const struct ChanType svrchansess;

struct SigMap {
	int signal;
	char* name;
};

extern const struct SigMap signames[];

#endif /* _CHANSESSION_H_ */
