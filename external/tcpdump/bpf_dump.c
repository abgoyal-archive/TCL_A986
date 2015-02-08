
#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/bpf_dump.c,v 1.16 2003/11/16 09:36:08 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <pcap.h>
#include <stdio.h>

#include "interface.h"

void
bpf_dump(struct bpf_program *p, int option)
{
	struct bpf_insn *insn;
	int i;
	int n = p->bf_len;

	insn = p->bf_insns;
	if (option > 2) {
		printf("%d\n", n);
		for (i = 0; i < n; ++insn, ++i) {
			printf("%u %u %u %u\n", insn->code,
			       insn->jt, insn->jf, insn->k);
		}
		return ;
	}
	if (option > 1) {
		for (i = 0; i < n; ++insn, ++i)
			printf("{ 0x%x, %d, %d, 0x%08x },\n",
			       insn->code, insn->jt, insn->jf, insn->k);
		return;
	}
	for (i = 0; i < n; ++insn, ++i) {
#ifdef BDEBUG
		extern int bids[];
		printf(bids[i] > 0 ? "[%02d]" : " -- ", bids[i] - 1);
#endif
		puts(bpf_image(insn, i));
	}
}
