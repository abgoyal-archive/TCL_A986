#ifndef __ZTRANS_H
#define __ZTRANS_H


#include "unicode/utypes.h"

#ifndef UCNV_H

struct ZTrans;
typedef struct ZTrans ZTrans;

#endif

U_DRAFT ZTrans* U_EXPORT2
ztrans_open(UDate time, const void* from, const void* to);

U_DRAFT ZTrans* U_EXPORT2
ztrans_openEmpty();

U_DRAFT void U_EXPORT2
ztrans_close(ZTrans *trans);

U_DRAFT ZTrans* U_EXPORT2
ztrans_clone(ZTrans *trans);

U_DRAFT UBool U_EXPORT2
ztrans_equals(const ZTrans* trans1, const ZTrans* trans2);

U_DRAFT UDate U_EXPORT2
ztrans_getTime(ZTrans* trans);

U_DRAFT void U_EXPORT2
ztrans_setTime(ZTrans* trans, UDate time);

U_DRAFT void* U_EXPORT2
ztrans_getFrom(ZTrans* & trans);

U_DRAFT void U_EXPORT2
ztrans_setFrom(ZTrans* trans, const void* from);

U_DRAFT void U_EXPORT2
ztrans_adoptFrom(ZTrans* trans, void* from);

U_DRAFT void* U_EXPORT2
ztrans_getTo(ZTrans* trans);

U_DRAFT void U_EXPORT2
ztrans_setTo(ZTrans* trans, const void* to);

U_DRAFT void U_EXPORT2
ztrans_adoptTo(ZTrans* trans, void* to);

U_DRAFT UClassID U_EXPORT2
ztrans_getStaticClassID(ZTrans* trans);

U_DRAFT UClassID U_EXPORT2
ztrans_getDynamicClassID(ZTrans* trans);

#endif
