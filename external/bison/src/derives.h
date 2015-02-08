
#ifndef DERIVES_H_
# define DERIVES_H_

# include "gram.h"

extern rule ***derives;

/* Compute DERIVES.  */

void derives_compute (void);
void derives_free (void);

#endif /* !DERIVES_H_ */
