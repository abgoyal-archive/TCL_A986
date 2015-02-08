


#ifndef UCNV_ERR_H
#define UCNV_ERR_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_CONVERSION

/** Forward declaring the UConverter structure. @stable ICU 2.0 */
struct UConverter;

/** @stable ICU 2.0 */
typedef struct UConverter UConverter;

#define UCNV_SUB_STOP_ON_ILLEGAL "i"

#define UCNV_SKIP_STOP_ON_ILLEGAL "i"

#define UCNV_ESCAPE_ICU       NULL
#define UCNV_ESCAPE_JAVA      "J"
#define UCNV_ESCAPE_C         "C"
#define UCNV_ESCAPE_XML_DEC   "D"
#define UCNV_ESCAPE_XML_HEX   "X"
#define UCNV_ESCAPE_UNICODE   "U"

typedef enum {
    UCNV_UNASSIGNED = 0,  /**< The code point is unassigned.
                             The error code U_INVALID_CHAR_FOUND will be set. */
    UCNV_ILLEGAL = 1,     /**< The code point is illegal. For example, 
                             \\x81\\x2E is illegal in SJIS because \\x2E
                             is not a valid trail byte for the \\x81 
                             lead byte.
                             Also, starting with Unicode 3.0.1, non-shortest byte sequences
                             in UTF-8 (like \\xC1\\xA1 instead of \\x61 for U+0061)
                             are also illegal, not just irregular.
                             The error code U_ILLEGAL_CHAR_FOUND will be set. */
    UCNV_IRREGULAR = 2,   /**< The codepoint is not a regular sequence in 
                             the encoding. For example, \\xED\\xA0\\x80..\\xED\\xBF\\xBF
                             are irregular UTF-8 byte sequences for single surrogate
                             code points.
                             The error code U_INVALID_CHAR_FOUND will be set. */
    UCNV_RESET = 3,       /**< The callback is called with this reason when a
                             'reset' has occured. Callback should reset all
                             state. */
    UCNV_CLOSE = 4,        /**< Called when the converter is closed. The
                             callback should release any allocated memory.*/
    UCNV_CLONE = 5         /**< Called when ucnv_safeClone() is called on the
                              converter. the pointer available as the
                              'context' is an alias to the original converters'
                              context pointer. If the context must be owned
                              by the new converter, the callback must clone 
                              the data and call ucnv_setFromUCallback 
                              (or setToUCallback) with the correct pointer.
                              @stable ICU 2.2
                           */
} UConverterCallbackReason;


typedef struct {
    uint16_t size;              /**< The size of this struct. @stable ICU 2.0 */
    UBool flush;                /**< The internal state of converter will be reset and data flushed if set to TRUE. @stable ICU 2.0    */
    UConverter *converter;      /**< Pointer to the converter that is opened and to which this struct is passed as an argument. @stable ICU 2.0  */
    const UChar *source;        /**< Pointer to the source source buffer. @stable ICU 2.0    */
    const UChar *sourceLimit;   /**< Pointer to the limit (end + 1) of source buffer. @stable ICU 2.0    */
    char *target;               /**< Pointer to the target buffer. @stable ICU 2.0    */
    const char *targetLimit;    /**< Pointer to the limit (end + 1) of target buffer. @stable ICU 2.0     */
    int32_t *offsets;           /**< Pointer to the buffer that recieves the offsets. *offset = blah ; offset++;. @stable ICU 2.0  */
} UConverterFromUnicodeArgs;


typedef struct {
    uint16_t size;              /**< The size of this struct   @stable ICU 2.0 */
    UBool flush;                /**< The internal state of converter will be reset and data flushed if set to TRUE. @stable ICU 2.0   */
    UConverter *converter;      /**< Pointer to the converter that is opened and to which this struct is passed as an argument. @stable ICU 2.0 */
    const char *source;         /**< Pointer to the source source buffer. @stable ICU 2.0    */
    const char *sourceLimit;    /**< Pointer to the limit (end + 1) of source buffer. @stable ICU 2.0    */
    UChar *target;              /**< Pointer to the target buffer. @stable ICU 2.0    */
    const UChar *targetLimit;   /**< Pointer to the limit (end + 1) of target buffer. @stable ICU 2.0     */
    int32_t *offsets;           /**< Pointer to the buffer that recieves the offsets. *offset = blah ; offset++;. @stable ICU 2.0  */
} UConverterToUnicodeArgs;


U_STABLE void U_EXPORT2 UCNV_FROM_U_CALLBACK_STOP (
                  const void *context,
                  UConverterFromUnicodeArgs *fromUArgs,
                  const UChar* codeUnits,
                  int32_t length,
                  UChar32 codePoint,
                  UConverterCallbackReason reason,
                  UErrorCode * err);



U_STABLE void U_EXPORT2 UCNV_TO_U_CALLBACK_STOP (
                  const void *context,
                  UConverterToUnicodeArgs *toUArgs,
                  const char* codeUnits,
                  int32_t length,
                  UConverterCallbackReason reason,
                  UErrorCode * err);

U_STABLE void U_EXPORT2 UCNV_FROM_U_CALLBACK_SKIP (
                  const void *context,
                  UConverterFromUnicodeArgs *fromUArgs,
                  const UChar* codeUnits,
                  int32_t length,
                  UChar32 codePoint,
                  UConverterCallbackReason reason,
                  UErrorCode * err);

U_STABLE void U_EXPORT2 UCNV_FROM_U_CALLBACK_SUBSTITUTE (
                  const void *context,
                  UConverterFromUnicodeArgs *fromUArgs,
                  const UChar* codeUnits,
                  int32_t length,
                  UChar32 codePoint,
                  UConverterCallbackReason reason,
                  UErrorCode * err);

U_STABLE void U_EXPORT2 UCNV_FROM_U_CALLBACK_ESCAPE (
                  const void *context,
                  UConverterFromUnicodeArgs *fromUArgs,
                  const UChar* codeUnits,
                  int32_t length,
                  UChar32 codePoint,
                  UConverterCallbackReason reason,
                  UErrorCode * err);


U_STABLE void U_EXPORT2 UCNV_TO_U_CALLBACK_SKIP (
                  const void *context,
                  UConverterToUnicodeArgs *toUArgs,
                  const char* codeUnits,
                  int32_t length,
                  UConverterCallbackReason reason,
                  UErrorCode * err);

U_STABLE void U_EXPORT2 UCNV_TO_U_CALLBACK_SUBSTITUTE (
                  const void *context,
                  UConverterToUnicodeArgs *toUArgs,
                  const char* codeUnits,
                  int32_t length,
                  UConverterCallbackReason reason,
                  UErrorCode * err);


U_STABLE void U_EXPORT2 UCNV_TO_U_CALLBACK_ESCAPE (
                  const void *context,
                  UConverterToUnicodeArgs *toUArgs,
                  const char* codeUnits,
                  int32_t length,
                  UConverterCallbackReason reason,
                  UErrorCode * err);

#endif

#endif

/*UCNV_ERR_H*/ 
