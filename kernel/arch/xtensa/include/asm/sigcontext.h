

#ifndef _XTENSA_SIGCONTEXT_H
#define _XTENSA_SIGCONTEXT_H


struct sigcontext {
	unsigned long sc_pc;
	unsigned long sc_ps;
	unsigned long sc_lbeg;
	unsigned long sc_lend;
	unsigned long sc_lcount;
	unsigned long sc_sar;
	unsigned long sc_acclo;
	unsigned long sc_acchi;
	unsigned long sc_a[16];
	void *sc_xtregs;
};

#endif /* _XTENSA_SIGCONTEXT_H */
