
#ifndef _ASM_X86_IOMAP_H
#define _ASM_X86_IOMAP_H


#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>

void *
iomap_atomic_prot_pfn(unsigned long pfn, enum km_type type, pgprot_t prot);

void
iounmap_atomic(void *kvaddr, enum km_type type);

int
iomap_create_wc(resource_size_t base, unsigned long size, pgprot_t *prot);

void
iomap_free(resource_size_t base, unsigned long size);

#endif /* _ASM_X86_IOMAP_H */
