
#ifndef __ERRORCODE_H__
#define __ERRORCODE_H__


#include "unicode/utypes.h"
#include "unicode/uobject.h"

U_NAMESPACE_BEGIN

class U_COMMON_API ErrorCode: public UMemory {
public:
    /**
     * Default constructor. Initializes its UErrorCode to U_ZERO_ERROR.
     * @draft ICU 4.2
     */
    ErrorCode() : errorCode(U_ZERO_ERROR) {}
    /** Destructor, does nothing. See class documentation for details. @draft ICU 4.2 */
    virtual ~ErrorCode() {}
    /** Conversion operator, returns a reference. @draft ICU 4.2 */
    operator UErrorCode & () { return errorCode; }
    /** Conversion operator, returns a pointer. @draft ICU 4.2 */
    operator UErrorCode * () { return &errorCode; }
    /** Tests for U_SUCCESS(). @draft ICU 4.2 */
    UBool isSuccess() const { return U_SUCCESS(errorCode); }
    /** Tests for U_FAILURE(). @draft ICU 4.2 */
    UBool isFailure() const { return U_FAILURE(errorCode); }
    /** Returns the UErrorCode value. @draft ICU 4.2 */
    UErrorCode get() const { return errorCode; }
    /** Sets the UErrorCode value. @draft ICU 4.2 */
    void set(UErrorCode value) { errorCode=value; }
    /** Returns the UErrorCode value and resets it to U_ZERO_ERROR. @draft ICU 4.2 */
    UErrorCode reset();
    /**
     * Checks for a failure code:
     * \code
     *   if(isFailure()) { handleFailure(); }
     * \endcode
     * @draft ICU 4.2
     */
    void check() const;

protected:
    /**
     * Internal UErrorCode, accessible to subclasses.
     * @draft ICU 4.2
     */
    UErrorCode errorCode;
    /**
     * Called by check() if isFailure() is true.
     * A subclass should override this function to deal with a failure code:
     * Throw an exception, log an error, terminate the program, or similar.
     * @draft ICU 4.2
     */
    virtual void handleFailure() const {}
};

U_NAMESPACE_END

#endif  // __ERRORCODE_H__
