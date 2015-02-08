
#ifndef __ASM_ARCH_PROCESSOR_H
#define __ASM_ARCH_PROCESSOR_H

#include <mach/module.h>

#define processor_is_ns9210()	(0			\
		|| module_is_cc7ucamry()		\
		|| module_is_cc9p9210()			\
		|| module_is_inc20otter()		\
		|| module_is_otter()			\
		)

#define processor_is_ns9215()	(0			\
		|| module_is_cc9p9215()			\
		)

#define processor_is_ns9360()	(0			\
		|| module_is_cc9p9360()			\
		|| module_is_cc9c()			\
		|| module_is_ccw9c()			\
		)

#define processor_is_ns9750()	(0			\
		|| module_is_cc9p9750()			\
		)

#define processor_is_ns921x()	(0			\
		|| processor_is_ns9210()		\
		|| processor_is_ns9215()		\
		)

#endif /* ifndef __ASM_ARCH_PROCESSOR_H */
