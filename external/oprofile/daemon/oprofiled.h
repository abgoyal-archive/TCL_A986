
#ifndef OPROFILED_H

#include <signal.h>

struct oprofiled_ops {
	void (*init)(void);
	void (*start)(void);
	void (*exit)(void);
};


void opd_open_logfile(void);

 
int is_image_ignored(char const * name);

/** return the int in the given oprofilefs file, error is fatal if !is_fatal */
int opd_read_fs_int(char const * path, char const * name, int is_fatal);


/** global variable positioned by signal handler */
extern sig_atomic_t signal_alarm;
extern sig_atomic_t signal_hup;
extern sig_atomic_t signal_term;
extern sig_atomic_t signal_child;
extern sig_atomic_t signal_usr1;
extern sig_atomic_t signal_usr2;

extern unsigned int op_nr_counters;
extern int separate_lib;
extern int separate_kernel;
extern int separate_thread;
extern int separate_cpu;
extern int no_vmlinux;
extern char * vmlinux;
extern char * kernel_range;
extern int no_xen;
extern char * xenimage;
extern char * xen_range;

#endif /* OPROFILED_H */
