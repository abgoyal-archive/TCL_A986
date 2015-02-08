

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/smp.h>
#include <linux/sem.h>
#include <linux/msg.h>
#include <linux/shm.h>
#include <linux/stat.h>
#include <linux/syscalls.h>
#include <linux/mman.h>
#include <linux/file.h>
#include <linux/utsname.h>
#include <linux/ipc.h>

#include <linux/uaccess.h>
#include <linux/unistd.h>

#include <asm/syscalls.h>

int kernel_execve(const char *filename, char *const argv[], char *const envp[])
{
	long __res;
	asm volatile ("push %%ebx ; movl %2,%%ebx ; int $0x80 ; pop %%ebx"
	: "=a" (__res)
	: "0" (__NR_execve), "ri" (filename), "c" (argv), "d" (envp) : "memory");
	return __res;
}
