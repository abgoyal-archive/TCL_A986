
#ifndef _ASM_SGIALIB_H
#define _ASM_SGIALIB_H

#include <asm/sgiarcs.h>

extern struct linux_romvec *romvec;
extern int prom_argc;

extern LONG *_prom_argv, *_prom_envp;

#define prom_argv(index) ((char *) (long) _prom_argv[(index)])
#define prom_argc(index) ((char *) (long) _prom_argc[(index)])

extern int prom_flags;

#define PROM_FLAG_ARCS			1
#define PROM_FLAG_USE_AS_CONSOLE	2
#define PROM_FLAG_DONT_FREE_TEMP	4

/* Simple char-by-char console I/O. */
extern void prom_putchar(char c);
extern char prom_getchar(void);

extern struct linux_mdesc *prom_getmdesc(struct linux_mdesc *curr);
#define PROM_NULL_MDESC   ((struct linux_mdesc *) 0)

extern void prom_meminit(void);

/* PROM device tree library routines. */
#define PROM_NULL_COMPONENT ((pcomponent *) 0)

/* Get sibling component of THIS. */
extern pcomponent *ArcGetPeer(pcomponent *this);

/* Get child component of THIS. */
extern pcomponent *ArcGetChild(pcomponent *this);

extern void prom_identify_arch(void);

/* Environment variable routines. */
extern PCHAR ArcGetEnvironmentVariable(PCHAR name);
extern LONG ArcSetEnvironmentVariable(PCHAR name, PCHAR value);

/* ARCS command line parsing. */
extern void prom_init_cmdline(void);

/* File operations. */
extern LONG ArcRead(ULONG fd, PVOID buf, ULONG num, PULONG cnt);
extern LONG ArcWrite(ULONG fd, PVOID buf, ULONG num, PULONG cnt);

/* Misc. routines. */
extern VOID ArcReboot(VOID) __attribute__((noreturn));
extern VOID ArcEnterInteractiveMode(VOID) __attribute__((noreturn));
extern VOID ArcFlushAllCaches(VOID);
extern DISPLAY_STATUS *ArcGetDisplayStatus(ULONG FileID);

#endif /* _ASM_SGIALIB_H */
