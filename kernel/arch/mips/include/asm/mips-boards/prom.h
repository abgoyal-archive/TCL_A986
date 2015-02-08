

#ifndef _MIPS_PROM_H
#define _MIPS_PROM_H

extern char *prom_getcmdline(void);
extern char *prom_getenv(char *name);
extern void prom_init_cmdline(void);
extern void prom_meminit(void);
extern void prom_fixup_mem_map(unsigned long start_mem, unsigned long end_mem);
extern void mips_display_message(const char *str);
extern void mips_display_word(unsigned int num);
extern void mips_scroll_message(void);
extern int get_ethernet_addr(char *ethernet_addr);

/* Memory descriptor management. */
#define PROM_MAX_PMEMBLOCKS    32
struct prom_pmemblock {
        unsigned long base; /* Within KSEG0. */
        unsigned int size;  /* In bytes. */
        unsigned int type;  /* free or prom memory */
};

#endif /* !(_MIPS_PROM_H) */
