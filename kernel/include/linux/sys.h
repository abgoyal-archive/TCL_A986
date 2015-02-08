
#ifndef _LINUX_SYS_H
#define _LINUX_SYS_H


#ifdef notdef
#define _sys_waitpid	_sys_old_syscall	/* _sys_wait4 */
#define _sys_olduname	_sys_old_syscall	/* _sys_newuname */
#define _sys_uname	_sys_old_syscall	/* _sys_newuname */
#define _sys_stat	_sys_old_syscall	/* _sys_newstat */
#define _sys_fstat	_sys_old_syscall	/* _sys_newfstat */
#define _sys_lstat	_sys_old_syscall	/* _sys_newlstat */
#define _sys_signal	_sys_old_syscall	/* _sys_sigaction */
#define _sys_sgetmask	_sys_old_syscall	/* _sys_sigprocmask */
#define _sys_ssetmask	_sys_old_syscall	/* _sys_sigprocmask */
#endif

#endif
