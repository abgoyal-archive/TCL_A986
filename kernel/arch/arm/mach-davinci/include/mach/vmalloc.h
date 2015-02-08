
#include <mach/hardware.h>

/* Allow vmalloc range until the IO virtual range minus a 2M "hole" */
#define VMALLOC_END	  (IO_VIRT - (2<<20))
