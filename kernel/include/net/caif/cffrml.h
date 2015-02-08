

#ifndef CFFRML_H_
#define CFFRML_H_
#include <net/caif/caif_layer.h>

struct cffrml;
struct cflayer *cffrml_create(u16 phyid, bool DoFCS);
void cffrml_set_uplayer(struct cflayer *this, struct cflayer *up);
void cffrml_set_dnlayer(struct cflayer *this, struct cflayer *dn);

#endif /* CFFRML_H_ */
