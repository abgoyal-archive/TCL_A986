
#ifndef __ERRORCODE_H__
#define __ERRORCODE_H__


#include "unicode/utypes.h"
#include "unicode/uobject.h"

U_NAMESPACE_BEGIN

class U_COMMON_API ErrorCode: public UMemory {
public:
    /**
     * Default constructor. Initializes its UErrorCode to U_ZERO_ERROR.
     * @stable ICU 4.2
     */
    ErrorCode() : errorCode(U_ZERO_ERROR) {}
    /** Destructor, does nothing. See class documentation for details. @stable ICU 4.2 */
    virtual ~ErrorCode() {}
    /** Conversion operator, returns a reference. @stable ICU 4.2 */
    operator UErrorCode & () { return errorCode; }
    /** Conversion operator, returns a pointer. @stable ICU 4.2 */
    operator UErrorCode * () { return &errorCode; }
    /** Tests for U_SUCCESS(). @stable ICU 4.2 */
    UBool isSuccess() const { return U_SUCCESS(errorCode); }
    /** Tests for U_FAILURE(). @stable ICU 4.2 */
    UBool isFailure() const { return U_FAILURE(errorCode); }
    /** Returns the UErrorCode value. @stable ICU 4.2 */
    UErrorCode get() const { return errorCode; }
    /** Sets the UErrorCode value. @stable ICU 4.2 */
    void set(UErrorCode value) { errorCode=value; }
    /** Returns the UErrorCode value and resets it to U_ZERO_ERROR. @stable ICU 4.2 */
    UErrorCode reset();
    /**
     * Asserts isSuccess().
     * In other words, this method checks for a failure code,
     * and the base class handles it like this:
     * \code
     *   if(isFailure()) { handleFailure(); }
     * \endcode
     * @draft ICU 4.4
     */
    void assertSuccess() const;
    /**
     * Return a string for the UErrorCode value.
     * The string will be the same as the name of the error code constant
     * in the UErrorCode enum.
     * @draft ICU 4.4
     */
    const char* errorName() const;

protected:
    /**
     * Internal UErrorCode, accessible to subclasses.
     * @stable ICU 4.2
     */
    UErrorCode errorCode;
    /**
     * Called by assertSuccess() if isFailure() is true.
     * A subclass should override this function to deal with a failure code:
     * Throw an exception, log an error, terminate the program, or similar.
     * @stable ICU 4.2
     */
    virtual void handleFailure() const {}
};

U_NAMESPACE_END

#endif  // __ERRORCODE_H__
