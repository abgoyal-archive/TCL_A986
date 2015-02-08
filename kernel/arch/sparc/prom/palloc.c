

#include <asm/openprom.h>
#include <asm/oplib.h>


char *
prom_alloc(char *virtual_hint, unsigned int num_bytes)
{
	if(prom_vers == PROM_V0) return (char *) 0x0;
	if(num_bytes == 0x0) return (char *) 0x0;
	return (*(romvec->pv_v2devops.v2_dumb_mem_alloc))(virtual_hint, num_bytes);
}

void
prom_free(char *vaddr, unsigned int num_bytes)
{
	if((prom_vers == PROM_V0) || (num_bytes == 0x0)) return;
	(*(romvec->pv_v2devops.v2_dumb_mem_free))(vaddr, num_bytes);
}
