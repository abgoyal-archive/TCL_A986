
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include "utils.h"
#include "tc_util.h"
#include "m_pedit.h"

static int
parse_udp(int *argc_p, char ***argv_p,struct tc_pedit_sel *sel,struct tc_pedit_key *tkey)
{
	int res = -1;
	return res;
}

struct m_pedit_util p_pedit_udp = {
	NULL,
	"udp",
	parse_udp,
};

