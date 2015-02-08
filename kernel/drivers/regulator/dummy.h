

#ifndef _DUMMY_H
#define _DUMMY_H

struct regulator_dev;

extern struct regulator_dev *dummy_regulator_rdev;

#ifdef CONFIG_REGULATOR_DUMMY
void __init regulator_dummy_init(void);
#else
static inline void regulator_dummy_init(void) { }
#endif

#endif
