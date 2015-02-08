#ifndef __MEASUREUNIT_H__
#define __MEASUREUNIT_H__

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "unicode/fmtable.h"

 
U_NAMESPACE_BEGIN

class U_I18N_API MeasureUnit: public UObject {
 public:
    /**
     * Return a polymorphic clone of this object.  The result will
     * have the same class as returned by getDynamicClassID().
     * @stable ICU 3.0
     */
    virtual UObject* clone() const = 0;

    /**
     * Destructor
     * @stable ICU 3.0
     */
    virtual ~MeasureUnit();
    
    /**
     * Equality operator.  Return true if this object is equal
     * to the given object.
     * @stable ICU 3.0
     */
    virtual UBool operator==(const UObject& other) const = 0;

 protected:
    /**
     * Default constructor.
     * @stable ICU 3.0
     */
    MeasureUnit();
};

U_NAMESPACE_END

// NOTE: There is no measunit.cpp. For implementation, see measure.cpp. [alan]

#endif // !UCONFIG_NO_FORMATTING
#endif // __MEASUREUNIT_H__
