

#ifndef CFSERL_H_
#define CFSERL_H_
#include <net/caif/caif_layer.h>

struct cflayer *cfserl_create(int type, int instance, bool use_stx);
#endif				/* CFSERL_H_ */
