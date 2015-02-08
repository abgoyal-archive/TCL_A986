

#ifndef _ATMEL_H
#define _ATMEL_H

typedef enum {
	ATMEL_FW_TYPE_NONE = 0,
	ATMEL_FW_TYPE_502,
	ATMEL_FW_TYPE_502D,
	ATMEL_FW_TYPE_502E,
	ATMEL_FW_TYPE_502_3COM,
	ATMEL_FW_TYPE_504,
	ATMEL_FW_TYPE_504_2958,
	ATMEL_FW_TYPE_504A_2958,
	ATMEL_FW_TYPE_506
} AtmelFWType;

struct net_device *init_atmel_card(unsigned short, unsigned long, const AtmelFWType, struct device *, 
				    int (*present_func)(void *), void * );
void stop_atmel_card( struct net_device *);
int atmel_open( struct net_device * );

#endif
