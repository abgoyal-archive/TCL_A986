

#ifndef __SPARC64_OPLIB_H
#define __SPARC64_OPLIB_H

#include <asm/openprom.h>

/* OBP version string. */
extern char prom_version[];

extern int prom_root_node;

/* PROM stdin and stdout */
extern int prom_stdin, prom_stdout;

extern int prom_chosen_node;

/* Helper values and strings in arch/sparc64/kernel/head.S */
extern const char prom_peer_name[];
extern const char prom_compatible_name[];
extern const char prom_root_compatible[];
extern const char prom_cpu_compatible[];
extern const char prom_finddev_name[];
extern const char prom_chosen_path[];
extern const char prom_cpu_path[];
extern const char prom_getprop_name[];
extern const char prom_mmu_name[];
extern const char prom_callmethod_name[];
extern const char prom_translate_name[];
extern const char prom_map_name[];
extern const char prom_unmap_name[];
extern int prom_mmu_ihandle_cache;
extern unsigned int prom_boot_mapped_pc;
extern unsigned int prom_boot_mapping_mode;
extern unsigned long prom_boot_mapping_phys_high, prom_boot_mapping_phys_low;

struct linux_mlist_p1275 {
	struct linux_mlist_p1275 *theres_more;
	unsigned long start_adr;
	unsigned long num_bytes;
};

struct linux_mem_p1275 {
	struct linux_mlist_p1275 **p1275_totphys;
	struct linux_mlist_p1275 **p1275_prommap;
	struct linux_mlist_p1275 **p1275_available; /* What we can use */
};

/* The functions... */

extern void prom_init(void *cif_handler, void *cif_stack);

/* Boot argument acquisition, returns the boot command line string. */
extern char *prom_getbootargs(void);

/* Device utilities. */

/* Device operations. */

extern int prom_devopen(const char *device_string);

extern int prom_devclose(int device_handle);

extern void prom_seek(int device_handle, unsigned int seek_hival,
		      unsigned int seek_lowval);

/* Miscellaneous routines, don't really fit in any category per se. */

/* Reboot the machine with the command line passed. */
extern void prom_reboot(const char *boot_command);

/* Evaluate the forth string passed. */
extern void prom_feval(const char *forth_string);

extern void prom_cmdline(void);

extern void prom_halt(void) __attribute__ ((noreturn));

/* Halt and power-off the machine. */
extern void prom_halt_power_off(void) __attribute__ ((noreturn));

typedef int (*callback_func_t)(long *cmd);
extern void prom_setcallback(callback_func_t func_ptr);

extern unsigned char prom_get_idprom(char *idp_buffer, int idpbuf_size);

/* Character operations to/from the console.... */

/* Non-blocking get character from console. */
extern int prom_nbgetchar(void);

/* Non-blocking put character to console. */
extern int prom_nbputchar(char character);

/* Blocking get character from console. */
extern char prom_getchar(void);

/* Blocking put character to console. */
extern void prom_putchar(char character);

/* Prom's internal routines, don't use in kernel/boot code. */
extern void prom_printf(const char *fmt, ...);
extern void prom_write(const char *buf, unsigned int len);

/* Multiprocessor operations... */
#ifdef CONFIG_SMP
extern void prom_startcpu(int cpunode, unsigned long pc, unsigned long arg);

extern void prom_startcpu_cpuid(int cpuid, unsigned long pc, unsigned long arg);

/* Stop the CPU with the given cpu ID.  */
extern void prom_stopcpu_cpuid(int cpuid);

/* Stop the current CPU. */
extern void prom_stopself(void);

/* Idle the current CPU. */
extern void prom_idleself(void);

/* Resume the CPU with the passed device tree node. */
extern void prom_resumecpu(int cpunode);
#endif

/* Power management interfaces. */

/* Put the current CPU to sleep. */
extern void prom_sleepself(void);

/* Put the entire system to sleep. */
extern int prom_sleepsystem(void);

/* Initiate a wakeup event. */
extern int prom_wakeupsystem(void);

/* MMU and memory related OBP interfaces. */

/* Get unique string identifying SIMM at given physical address. */
extern int prom_getunumber(int syndrome_code,
			   unsigned long phys_addr,
			   char *buf, int buflen);

/* Retain physical memory to the caller across soft resets. */
extern int prom_retain(const char *name, unsigned long size,
		       unsigned long align, unsigned long *paddr);

/* Load explicit I/D TLB entries into the calling processor. */
extern long prom_itlb_load(unsigned long index,
			   unsigned long tte_data,
			   unsigned long vaddr);

extern long prom_dtlb_load(unsigned long index,
			   unsigned long tte_data,
			   unsigned long vaddr);

#define PROM_MAP_WRITE	0x0001 /* Writable */
#define PROM_MAP_READ	0x0002 /* Readable - sw */
#define PROM_MAP_EXEC	0x0004 /* Executable - sw */
#define PROM_MAP_LOCKED	0x0010 /* Locked, use i/dtlb load calls for this instead */
#define PROM_MAP_CACHED	0x0020 /* Cacheable in both L1 and L2 caches */
#define PROM_MAP_SE	0x0040 /* Side-Effects */
#define PROM_MAP_GLOB	0x0080 /* Global */
#define PROM_MAP_IE	0x0100 /* Invert-Endianness */
#define PROM_MAP_DEFAULT (PROM_MAP_WRITE | PROM_MAP_READ | PROM_MAP_EXEC | PROM_MAP_CACHED)

extern int prom_map(int mode, unsigned long size,
		    unsigned long vaddr, unsigned long paddr);
extern void prom_unmap(unsigned long size, unsigned long vaddr);


/* PROM device tree traversal functions... */

/* Get the child node of the given node, or zero if no child exists. */
extern int prom_getchild(int parent_node);

extern int prom_getsibling(int node);

extern int prom_getproplen(int thisnode, const char *property);

extern int prom_getproperty(int thisnode, const char *property,
			    char *prop_buffer, int propbuf_size);

/* Acquire an integer property. */
extern int prom_getint(int node, const char *property);

/* Acquire an integer property, with a default value. */
extern int prom_getintdefault(int node, const char *property, int defval);

/* Acquire a boolean property, 0=FALSE 1=TRUE. */
extern int prom_getbool(int node, const char *prop);

/* Acquire a string property, null string on error. */
extern void prom_getstring(int node, const char *prop, char *buf, int bufsize);

/* Does the passed node have the given "name"? YES=1 NO=0 */
extern int prom_nodematch(int thisnode, const char *name);

extern int prom_searchsiblings(int node_start, const char *name);

extern char *prom_firstprop(int node, char *buffer);

extern char *prom_nextprop(int node, const char *prev_property, char *buffer);

/* Returns 1 if the specified node has given property. */
extern int prom_node_has_property(int node, const char *property);

/* Returns phandle of the path specified */
extern int prom_finddevice(const char *name);

extern int prom_setprop(int node, const char *prop_name, char *prop_value,
			int value_size);

extern int prom_pathtoinode(const char *path);
extern int prom_inst2pkg(int);
extern int prom_service_exists(const char *service_name);
extern void prom_sun4v_guest_soft_state(void);

extern int prom_ihandle2path(int handle, char *buffer, int bufsize);

/* Client interface level routines. */
extern void p1275_cmd_direct(unsigned long *);

#endif /* !(__SPARC64_OPLIB_H) */
