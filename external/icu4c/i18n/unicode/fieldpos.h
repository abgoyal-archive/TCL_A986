
// *****************************************************************************
// This file was generated from the java source file FieldPosition.java
// *****************************************************************************
 
#ifndef FIELDPOS_H
#define FIELDPOS_H

#include "unicode/utypes.h"


#if !UCONFIG_NO_FORMATTING

#include "unicode/uobject.h"

U_NAMESPACE_BEGIN

class U_I18N_API FieldPosition : public UObject {
public:
    /**
     * DONT_CARE may be specified as the field to indicate that the
     * caller doesn't need to specify a field.  Do not subclass.
     */
    enum { DONT_CARE = -1 };

    /**
     * Creates a FieldPosition object with a non-specified field.
     * @stable ICU 2.0
     */
    FieldPosition() 
        : UObject(), fField(DONT_CARE), fBeginIndex(0), fEndIndex(0) {}

    /**
     * Creates a FieldPosition object for the given field.  Fields are
     * identified by constants, whose names typically end with _FIELD,
     * in the various subclasses of Format.
     *
     * @see NumberFormat#INTEGER_FIELD
     * @see NumberFormat#FRACTION_FIELD
     * @see DateFormat#YEAR_FIELD
     * @see DateFormat#MONTH_FIELD
     * @stable ICU 2.0
     */
    FieldPosition(int32_t field) 
        : UObject(), fField(field), fBeginIndex(0), fEndIndex(0) {}

    /**
     * Copy constructor
     * @param copy the object to be copied from.
     * @stable ICU 2.0
     */
    FieldPosition(const FieldPosition& copy) 
        : UObject(copy), fField(copy.fField), fBeginIndex(copy.fBeginIndex), fEndIndex(copy.fEndIndex) {}

    /**
     * Destructor
     * @stable ICU 2.0
     */
    virtual ~FieldPosition();

    /**
     * Assignment operator
     * @param copy the object to be copied from.
     * @stable ICU 2.0
     */
    FieldPosition&      operator=(const FieldPosition& copy);

    /** 
     * Equality operator.
     * @param that    the object to be compared with.
     * @return        TRUE if the two field positions are equal, FALSE otherwise.
     * @stable ICU 2.0
     */
    UBool              operator==(const FieldPosition& that) const;

    /** 
     * Equality operator.
     * @param that    the object to be compared with.
     * @return        TRUE if the two field positions are not equal, FALSE otherwise.
     * @stable ICU 2.0
     */
    UBool              operator!=(const FieldPosition& that) const;

    /**
     * Clone this object.
     * Clones can be used concurrently in multiple threads.
     * If an error occurs, then NULL is returned.
     * The caller must delete the clone.
     *
     * @return a clone of this object
     *
     * @see getDynamicClassID
     * @stable ICU 2.8
     */
    FieldPosition *clone() const;

    /**
     * Retrieve the field identifier.
     * @return    the field identifier.
     * @stable ICU 2.0
     */
    int32_t getField(void) const { return fField; }

    /**
     * Retrieve the index of the first character in the requested field.
     * @return    the index of the first character in the requested field.
     * @stable ICU 2.0
     */
    int32_t getBeginIndex(void) const { return fBeginIndex; }

    /**
     * Retrieve the index of the character following the last character in the
     * requested field.
     * @return    the index of the character following the last character in the
     *            requested field.
     * @stable ICU 2.0
     */
    int32_t getEndIndex(void) const { return fEndIndex; }
 
    /**
     * Set the field.
     * @param f    the new value of the field.
     * @stable ICU 2.0
     */
    void setField(int32_t f) { fField = f; }

    /**
     * Set the begin index.  For use by subclasses of Format.
     * @param bi    the new value of the begin index
     * @stable ICU 2.0
     */
    void setBeginIndex(int32_t bi) { fBeginIndex = bi; }

    /**
     * Set the end index.  For use by subclasses of Format.
     * @param ei    the new value of the end index
     * @stable ICU 2.0
     */
    void setEndIndex(int32_t ei) { fEndIndex = ei; }
    
    /**
     * ICU "poor man's RTTI", returns a UClassID for the actual class.
     *
     * @stable ICU 2.2
     */
    virtual UClassID getDynamicClassID() const;

    /**
     * ICU "poor man's RTTI", returns a UClassID for this class.
     *
     * @stable ICU 2.2
     */
    static UClassID U_EXPORT2 getStaticClassID();

private:
    /**
     * Input: Desired field to determine start and end offsets for.
     * The meaning depends on the subclass of Format.
     */
    int32_t fField;

    /**
     * Output: Start offset of field in text.
     * If the field does not occur in the text, 0 is returned.
     */
    int32_t fBeginIndex;

    /**
     * Output: End offset of field in text.
     * If the field does not occur in the text, 0 is returned.
     */
    int32_t fEndIndex;
};

inline FieldPosition&
FieldPosition::operator=(const FieldPosition& copy)
{
    fField         = copy.fField;
    fEndIndex     = copy.fEndIndex;
    fBeginIndex = copy.fBeginIndex;
    return *this;
}

inline UBool
FieldPosition::operator==(const FieldPosition& copy) const
{
    return (fField == copy.fField &&
        fEndIndex == copy.fEndIndex &&
        fBeginIndex == copy.fBeginIndex);
}

inline UBool
FieldPosition::operator!=(const FieldPosition& copy) const
{
    return !operator==(copy);
}

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif // _FIELDPOS
//eof
