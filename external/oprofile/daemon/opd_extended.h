
#ifndef OPD_EXTENDED_H
#define OPD_EXTENDED_H

#include "opd_trans.h"
#include "odb.h"

#include <stdlib.h>
#include <stdint.h>


struct opd_ext_feature {
	// Feature name
	const char* feature;
	// Feature handlers
	struct opd_ext_handlers * handlers;
};

struct opd_ext_handlers {
	// Extended init
	int (*ext_init)(char const *);
	// Extended statistics
	int (*ext_print_stats)();
	// Extended sfile handlers
	struct opd_ext_sfile_handlers * ext_sfile;
};

struct opd_ext_sfile_handlers {
	int (*create)(struct sfile *);
	int (*dup)(struct sfile *, struct sfile *);
	int (*close)(struct sfile *);
	int (*sync)(struct sfile *);
	odb_t * (*get)(struct transient const *, int);
	struct opd_event * (*find_counter_event)(unsigned long);
};

extern int opd_ext_initialize(char const * value);

extern void opd_ext_print_stats();

extern void opd_ext_sfile_create(struct sfile * sf);
extern void opd_ext_sfile_dup (struct sfile * to, struct sfile * from);
extern void opd_ext_sfile_close(struct sfile * sf);
extern void opd_ext_sfile_sync(struct sfile * sf);
extern odb_t * opd_ext_sfile_get(struct transient const * trans, int is_cg);

extern struct opd_event * opd_ext_find_counter_event(unsigned long counter);


#endif
