

#ifndef _LINUX_KERNEL_DEBUGGER_H_
#define _LINUX_KERNEL_DEBUGGER_H_

struct kdbg_ctxt {
	int (*printf)(void *cookie, const char *fmt, ...);
	void *cookie;
};

int kernel_debugger(struct kdbg_ctxt *ctxt, char *cmd);

#endif
