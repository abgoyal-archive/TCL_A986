

#ifndef __PROCESS_H__
#define __PROCESS_H__

#include <signal.h>

/* Copied from linux/compiler-gcc.h since we can't include it directly */
#define barrier() __asm__ __volatile__("": : :"memory")

extern void sig_handler(int sig, struct sigcontext *sc);
extern void alarm_handler(int sig, struct sigcontext *sc);

#endif
