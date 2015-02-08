
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

static void explain(void)
{
	fprintf(stderr, "Usage: ... multiq [help]\n");
}

#define usage() return(-1)

static int multiq_parse_opt(struct qdisc_util *qu, int argc, char **argv,
			    struct nlmsghdr *n)
{
	struct tc_multiq_qopt opt;

	if (argc > 0) {
		if (strcmp(*argv, "help") == 0) {
			explain();
			return -1;
		} else {
			fprintf(stderr, "What is \"%s\"?\n", *argv);
			explain();
			return -1;
		}
		argc--; argv++;
	}

	addattr_l(n, 1024, TCA_OPTIONS, &opt, sizeof(opt));
	return 0;
}

int multiq_print_opt(struct qdisc_util *qu, FILE *f, struct rtattr *opt)
{
	struct tc_multiq_qopt *qopt;

	if (opt == NULL)
		return 0;
	if (RTA_PAYLOAD(opt) < sizeof(*qopt))
		return 0;

	qopt = RTA_DATA(opt);

	fprintf(f, "bands %u/%u ", qopt->bands, qopt->max_bands);

	return 0;
}

struct qdisc_util multiq_qdisc_util = {
	.id	 	= "multiq",
	.parse_qopt	= multiq_parse_opt,
	.print_qopt	= multiq_print_opt,
};
