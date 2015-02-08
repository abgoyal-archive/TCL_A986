
#include "chansession.h"

/* Mapping of signal values to ssh signal strings */
const struct SigMap signames[] = {
	{SIGABRT, "ABRT"},
	{SIGALRM, "ALRM"},
	{SIGFPE, "FPE"},
	{SIGHUP, "HUP"},
	{SIGILL, "ILL"},
	{SIGINT, "INT"},
	{SIGKILL, "KILL"},
	{SIGPIPE, "PIPE"},
	{SIGQUIT, "QUIT"},
	{SIGSEGV, "SEGV"},
	{SIGTERM, "TERM"},
	{SIGUSR1, "USR1"},
	{SIGUSR2, "USR2"},
	{0, NULL}
};
