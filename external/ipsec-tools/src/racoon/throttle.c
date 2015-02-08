/*	$NetBSD: throttle.c,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/* Id: throttle.c,v 1.5 2006/04/05 20:54:50 manubsd Exp */


#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if TIME_WITH_SYS_TIME
# include <sys/time.h>
# include <time.h> 
#else 
# if HAVE_SYS_TIME_H
#  include <sys/time.h>
# else
#  include <time.h>
# endif 
#endif
#include <sys/param.h>
#include <sys/queue.h>
#include <sys/socket.h>

#include <netinet/in.h>
#include <resolv.h>

#include "vmbuf.h"
#include "misc.h"
#include "plog.h"
#include "throttle.h"
#include "sockmisc.h"
#include "libpfkey.h"
#include "isakmp_var.h"
#include "isakmp.h"
#include "isakmp_xauth.h"
#include "isakmp_cfg.h"
#include "gcmalloc.h"

struct throttle_list throttle_list = TAILQ_HEAD_INITIALIZER(throttle_list);


struct throttle_entry *
throttle_add(addr)
	struct sockaddr *addr;
{
	struct throttle_entry *te;
	size_t len;

	len = sizeof(*te) 
	    - sizeof(struct sockaddr_storage) 
	    + sysdep_sa_len(addr);

	if ((te = racoon_malloc(len)) == NULL)
		return NULL;

	te->penalty = time(NULL) + isakmp_cfg_config.auth_throttle;
	memcpy(&te->host, addr, sysdep_sa_len(addr));
	TAILQ_INSERT_HEAD(&throttle_list, te, next);

	return te;
}

int
throttle_host(addr, authfail) 
	struct sockaddr *addr;
	int authfail;
{
	struct throttle_entry *te;
	int found = 0;
	time_t now;

	if (isakmp_cfg_config.auth_throttle == 0)
		return 0;

	now = time(NULL);

restart:
	RACOON_TAILQ_FOREACH_REVERSE(te, &throttle_list, throttle_list, next) {
	  /*
	   * Remove outdated entries 
	   */
		if (te->penalty < now) {
			TAILQ_REMOVE(&throttle_list, te, next);
			racoon_free(te);
			goto restart;
		}
			
		if (cmpsaddrwop(addr, (struct sockaddr *)&te->host) == 0) {
			found = 1;
			break;
		}
	}

	/* 
	 * No match, if auth failed, allocate a new throttle entry
	 * give no penalty even on error: this is the first time
	 * and we are indulgent.
	 */
	if (!found) {
		if (authfail) {
			if ((te = throttle_add(addr)) == NULL) {
				plog(LLV_ERROR, LOCATION, NULL, 
				    "Throttle insertion failed\n");
				return (time(NULL) 
				    + isakmp_cfg_config.auth_throttle);
			}
		}
		return 0;
	} else {
		/*
		 * We had a match and auth failed, increase penalty.
		 */
		if (authfail) {
			time_t remaining;
			time_t new;

			remaining = te->penalty - now;
			new = remaining + isakmp_cfg_config.auth_throttle;

			if (new > THROTTLE_PENALTY_MAX)
				new = THROTTLE_PENALTY_MAX;

			te->penalty = now + new;
		}
	}
	
	return te->penalty;
}

