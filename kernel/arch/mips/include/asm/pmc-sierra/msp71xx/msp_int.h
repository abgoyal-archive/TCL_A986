

#ifndef _MSP_INT_H
#define _MSP_INT_H


#if defined(CONFIG_IRQ_MSP_SLP)
	#include "msp_slp_int.h"
#elif defined(CONFIG_IRQ_MSP_CIC)
	#include "msp_cic_int.h"
#else
	#error "What sort of interrupt controller does *your* MSP have?"
#endif

#endif /* !_MSP_INT_H */
