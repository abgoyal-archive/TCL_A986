#ifndef _AGENTFWD_H_
#define _AGENTFWD_H_
#ifndef DISABLE_AGENTFWD

#include "includes.h"
#include "chansession.h"
#include "channel.h"

int agentreq(struct ChanSess * chansess);
void agentsetauth(struct ChanSess *chansess);
void agentcleanup(struct ChanSess * chansess);
void agentset(struct ChanSess *chansess);

#ifdef __hpux
#define seteuid(a)       setresuid(-1, (a), -1)
#define setegid(a)       setresgid(-1, (a), -1)
#endif

#endif /* DROPBEAR_AGENTFWD */
#endif /* _AGENTFWD_H_ */
