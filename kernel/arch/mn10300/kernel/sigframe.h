

struct sigframe
{
	void (*pretcode)(void);
	int sig;
	struct sigcontext *psc;
	struct sigcontext sc;
	struct fpucontext fpuctx;
	unsigned long extramask[_NSIG_WORDS-1];
	char retcode[8];
};

struct rt_sigframe
{
	void (*pretcode)(void);
	int sig;
	struct siginfo *pinfo;
	void *puc;
	struct siginfo info;
	struct ucontext uc;
	struct fpucontext fpuctx;
	char retcode[8];
};
