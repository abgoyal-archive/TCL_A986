

#ifndef _AM9513_H_
#define _AM9513_H_

#if 0

#define Am9513_8BITBUS xxx
/* or */
#define Am9513_16BITBUS xxx

#define Am9513_output_control(a)	xxx
#define Am9513_input_status()		xxx
#define Am9513_output_data(a)		xxx
#define Am9513_input_data()		xxx

#endif


#ifdef Am9513_8BITBUS

#define Am9513_write_register(reg, val)				\
	do{							\
		Am9513_output_control(reg);			\
		Am9513_output_data(val>>8);			\
		Am9513_output_data(val&0xff);			\
	}while (0)

#define Am9513_read_register(reg, val)				\
	do{							\
		Am9513_output_control(reg);			\
		val=Am9513_input_data()<<8;			\
		val|=Am9513_input_data();			\
	}while (0)

#else /* Am9513_16BITBUS */

#define Am9513_write_register(reg, val)				\
	do{							\
		Am9513_output_control(reg);			\
		Am9513_output_data(val);			\
	}while (0)

#define Am9513_read_register(reg, val)				\
	do{							\
		Am9513_output_control(reg);			\
		val=Am9513_input_data();			\
	}while (0)

#endif

#endif
