

#ifndef _CHANNEL_H_
#define _CHANNEL_H_

#include "ttype.h"

/*---------------------  Export Definitions -------------------------*/

/*---------------------  Export Classes  ----------------------------*/
typedef struct tagSChannelTblElement {
    BYTE    byChannelNumber;
    unsigned int    uFrequency;
    BOOL    bValid;
}SChannelTblElement, *PSChannelTblElement;

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/
BOOL    ChannelValid(unsigned int CountryCode, unsigned int ChannelNum);
void    CHvInitChannelTable(void *pDeviceHandler);
BYTE    CHbyGetChannelMapping(BYTE byChannelNumber);

BOOL
CHvChannelGetList (
      unsigned int       uCountryCodeIdx,
     PBYTE      pbyChannelTable
    );

#endif  /* _REGULATE_H_ */
