

#ifndef DateConversion_h
#define DateConversion_h

#include "UString.h"

namespace JSC {

class ExecState;
struct GregorianDateTime;

static const unsigned DateConversionBufferSize = 100;
typedef char DateConversionBuffer[DateConversionBufferSize];

double parseDate(ExecState* exec, const UString&);
void formatDate(const GregorianDateTime&, DateConversionBuffer&);
void formatDateUTCVariant(const GregorianDateTime&, DateConversionBuffer&);
void formatTime(const GregorianDateTime&, DateConversionBuffer&);
void formatTimeUTC(const GregorianDateTime&, DateConversionBuffer&);

} // namespace JSC

#endif // DateConversion_h
