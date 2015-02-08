
#ifndef _ASM_X86_MRST_H
#define _ASM_X86_MRST_H
extern int pci_mrst_init(void);
int __init sfi_parse_mrtc(struct sfi_table_header *table);

#define SFI_MTMR_MAX_NUM 8
#define SFI_MRTC_MAX	8

#endif /* _ASM_X86_MRST_H */
