
#ifndef CONFLICTS_H_
# define CONFLICTS_H_
# include "state.h"

void conflicts_solve (void);
void conflicts_print (void);
int conflicts_total_count (void);
void conflicts_output (FILE *out);
void conflicts_free (void);

/* Were there conflicts? */
extern int expected_sr_conflicts;
extern int expected_rr_conflicts;
#endif /* !CONFLICTS_H_ */
