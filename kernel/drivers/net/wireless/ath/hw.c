

#include <asm/unaligned.h>

#include "ath.h"
#include "reg.h"

#define REG_READ	(common->ops->read)
#define REG_WRITE	(common->ops->write)

void ath_hw_setbssidmask(struct ath_common *common)
{
	void *ah = common->ah;

	REG_WRITE(ah, get_unaligned_le32(common->bssidmask), AR_BSSMSKL);
	REG_WRITE(ah, get_unaligned_le16(common->bssidmask + 4), AR_BSSMSKU);
}
EXPORT_SYMBOL(ath_hw_setbssidmask);
