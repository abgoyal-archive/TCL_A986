

#ifndef _ARCH_MIPS_POWERTV_ASIC_PREALLOC_H
#define _ARCH_MIPS_POWERTV_ASIC_PREALLOC_H

#define KIBIBYTE(n) ((n) * 1024)    /* Number of kibibytes */
#define MEBIBYTE(n) ((n) * KIBIBYTE(1024)) /* Number of mebibytes */

/* "struct resource" array element definition */
#define PREALLOC(NAME, START, END, FLAGS) {	\
		.name = (NAME),			\
		.start = (START),		\
		.end = (END),			\
		.flags = (FLAGS)		\
	},

#ifdef CONFIG_PREALLOC_NORMAL
#define PREALLOC_NORMAL(name, start, end, flags) \
   PREALLOC(name, start, end, flags)
#else
#define PREALLOC_NORMAL(name, start, end, flags)
#endif

#ifdef CONFIG_PREALLOC_TFTP
#define PREALLOC_TFTP(name, start, end, flags) \
   PREALLOC(name, start, end, flags)
#else
#define PREALLOC_TFTP(name, start, end, flags)
#endif

#ifdef CONFIG_PREALLOC_DOCSIS
#define PREALLOC_DOCSIS(name, start, end, flags) \
   PREALLOC(name, start, end, flags)
#else
#define PREALLOC_DOCSIS(name, start, end, flags)
#endif

#ifdef CONFIG_PREALLOC_PMEM
#define PREALLOC_PMEM(name, start, end, flags) \
   PREALLOC(name, start, end, flags)
#else
#define PREALLOC_PMEM(name, start, end, flags)
#endif
#endif
