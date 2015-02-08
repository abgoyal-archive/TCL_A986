

#ifndef OP_COUNTER_H
#define OP_COUNTER_H

#define OP_MAX_COUNTER 32

struct op_counter_config {
	unsigned long count;
	unsigned long enabled;
	unsigned long event;
	unsigned long kernel;
	unsigned long user;
	unsigned long unit_mask;
};

extern struct op_counter_config counter_config[];

#endif /* OP_COUNTER_H */
