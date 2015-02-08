
#include <linux/errno.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/smp.h>
#include <linux/sem.h>
#include <linux/msg.h>
#include <linux/shm.h>
#include <linux/stat.h>
#include <linux/mman.h>
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/ipc.h>
#include <asm/uaccess.h>
#include <asm/ptrace.h>
#include <asm/unistd.h>

int kernel_execve(const char *filename, char *const argv[], char *const envp[])
{
	register unsigned long __sc0 __asm__ ("r9") = ((0x13 << 16) | __NR_execve);
	register unsigned long __sc2 __asm__ ("r2") = (unsigned long) filename;
	register unsigned long __sc3 __asm__ ("r3") = (unsigned long) argv;
	register unsigned long __sc4 __asm__ ("r4") = (unsigned long) envp;
	__asm__ __volatile__ ("trapa	%1 !\t\t\t execve(%2,%3,%4)"
	: "=r" (__sc0)
	: "r" (__sc0), "r" (__sc2), "r" (__sc3), "r" (__sc4) );
	__asm__ __volatile__ ("!dummy	%0 %1 %2 %3"
	: : "r" (__sc0), "r" (__sc2), "r" (__sc3), "r" (__sc4) : "memory");
	return __sc0;
}
