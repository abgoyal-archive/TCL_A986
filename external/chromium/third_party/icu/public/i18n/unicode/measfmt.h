#ifndef MEASUREFORMAT_H
#define MEASUREFORMAT_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "unicode/format.h"


U_NAMESPACE_BEGIN

class U_I18N_API MeasureFormat : public Format {

 public:

    /**
     * Return a formatter for CurrencyAmount objects in the given
     * locale.
     * @param locale desired locale
     * @param ec input-output error code
     * @return a formatter object, or NULL upon error
     * @stable ICU 3.0
     */
    static MeasureFormat* U_EXPORT2 createCurrencyFormat(const Locale& locale,
                                               UErrorCode& ec);

    /**
     * Return a formatter for CurrencyAmount objects in the default
     * locale.
     * @param ec input-output error code
     * @return a formatter object, or NULL upon error
     * @stable ICU 3.0
     */
    static MeasureFormat* U_EXPORT2 createCurrencyFormat(UErrorCode& ec);

 protected:

    /**
     * Default constructor.
     * @stable ICU 3.0
     */
    MeasureFormat();
};

U_NAMESPACE_END

#endif // #if !UCONFIG_NO_FORMATTING
#endif // #ifndef MEASUREFORMAT_H
