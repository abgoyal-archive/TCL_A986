
#ifndef NULLABLE_H_
# define NULLABLE_H_

extern bool *nullable;

/* Set up NULLABLE. */
extern void nullable_compute (void);

/* Free NULLABLE. */
extern void nullable_free (void);
#endif /* !NULLABLE_H_ */
