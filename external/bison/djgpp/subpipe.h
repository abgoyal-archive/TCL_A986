
/* Written by Juan Manuel Guerrero <juan.guerrero@gmx.de>. */

#if HAVE_SYS_TYPES_H
# include <sys/types.h>
#endif

void init_subpipe(void);
pid_t create_subpipe(char const * const *, int[2]);
void end_of_output_subpipe(pid_t, int[2]);
void reap_subpipe(pid_t, char const *);
