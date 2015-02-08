

#ifndef __UM_ARCHPARAM_I386_H
#define __UM_ARCHPARAM_I386_H

#ifdef CONFIG_X86_PAE
#define LAST_PKMAP 512
#else
#define LAST_PKMAP 1024
#endif

#endif

