
#ifndef __UTEXT_H__
#define __UTEXT_H__




#include "unicode/utypes.h"
#ifdef XP_CPLUSPLUS
#include "unicode/rep.h"
#include "unicode/unistr.h"
#include "unicode/chariter.h"
#endif


U_CDECL_BEGIN

struct UText;
typedef struct UText UText; /**< C typedef for struct UText. @stable ICU 3.6 */




U_STABLE UText * U_EXPORT2
utext_close(UText *ut);


U_STABLE UText * U_EXPORT2
utext_openUTF8(UText *ut, const char *s, int64_t length, UErrorCode *status);


U_STABLE UText * U_EXPORT2
utext_openUChars(UText *ut, const UChar *s, int64_t length, UErrorCode *status);


#ifdef XP_CPLUSPLUS
U_STABLE UText * U_EXPORT2
utext_openUnicodeString(UText *ut, U_NAMESPACE_QUALIFIER UnicodeString *s, UErrorCode *status);


U_STABLE UText * U_EXPORT2
utext_openConstUnicodeString(UText *ut, const U_NAMESPACE_QUALIFIER UnicodeString *s, UErrorCode *status);


U_STABLE UText * U_EXPORT2
utext_openReplaceable(UText *ut, U_NAMESPACE_QUALIFIER Replaceable *rep, UErrorCode *status);

U_STABLE UText * U_EXPORT2
utext_openCharacterIterator(UText *ut, U_NAMESPACE_QUALIFIER CharacterIterator *ic, UErrorCode *status);

#endif


U_STABLE UText * U_EXPORT2
utext_clone(UText *dest, const UText *src, UBool deep, UBool readOnly, UErrorCode *status);


U_STABLE UBool U_EXPORT2
utext_equals(const UText *a, const UText *b);



U_STABLE int64_t U_EXPORT2
utext_nativeLength(UText *ut);

U_STABLE UBool U_EXPORT2
utext_isLengthExpensive(const UText *ut);

U_STABLE UChar32 U_EXPORT2
utext_char32At(UText *ut, int64_t nativeIndex);


U_STABLE UChar32 U_EXPORT2
utext_current32(UText *ut);


U_STABLE UChar32 U_EXPORT2
utext_next32(UText *ut);


U_STABLE UChar32 U_EXPORT2
utext_previous32(UText *ut);


U_STABLE UChar32 U_EXPORT2
utext_next32From(UText *ut, int64_t nativeIndex);



U_STABLE UChar32 U_EXPORT2
utext_previous32From(UText *ut, int64_t nativeIndex);

U_STABLE int64_t U_EXPORT2
utext_getNativeIndex(const UText *ut);

U_STABLE void U_EXPORT2
utext_setNativeIndex(UText *ut, int64_t nativeIndex);

U_STABLE UBool U_EXPORT2
utext_moveIndex32(UText *ut, int32_t delta);

U_STABLE int64_t U_EXPORT2
utext_getPreviousNativeIndex(UText *ut); 


U_STABLE int32_t U_EXPORT2
utext_extract(UText *ut,
             int64_t nativeStart, int64_t nativeLimit,
             UChar *dest, int32_t destCapacity,
             UErrorCode *status);



#define UTEXT_NEXT32(ut)  \
    ((ut)->chunkOffset < (ut)->chunkLength && ((ut)->chunkContents)[(ut)->chunkOffset]<0xd800 ? \
    ((ut)->chunkContents)[((ut)->chunkOffset)++] : utext_next32(ut))

#define UTEXT_PREVIOUS32(ut)  \
    ((ut)->chunkOffset > 0 && \
     (ut)->chunkContents[(ut)->chunkOffset-1] < 0xd800 ? \
          (ut)->chunkContents[--((ut)->chunkOffset)]  :  utext_previous32(ut))

#define UTEXT_GETNATIVEINDEX(ut)                       \
    ((ut)->chunkOffset <= (ut)->nativeIndexingLimit?   \
        (ut)->chunkNativeStart+(ut)->chunkOffset :     \
        (ut)->pFuncs->mapOffsetToNative(ut))    

#define UTEXT_SETNATIVEINDEX(ut, ix)                       \
    { int64_t __offset = (ix) - (ut)->chunkNativeStart; \
      if (__offset>=0 && __offset<=(int64_t)(ut)->nativeIndexingLimit) { \
          (ut)->chunkOffset=(int32_t)__offset; \
      } else { \
          utext_setNativeIndex((ut), (ix)); } }





U_STABLE UBool U_EXPORT2
utext_isWritable(const UText *ut);


U_STABLE UBool U_EXPORT2
utext_hasMetaData(const UText *ut);


U_STABLE int32_t U_EXPORT2
utext_replace(UText *ut,
             int64_t nativeStart, int64_t nativeLimit,
             const UChar *replacementText, int32_t replacementLength,
             UErrorCode *status);



U_STABLE void U_EXPORT2
utext_copy(UText *ut,
          int64_t nativeStart, int64_t nativeLimit,
          int64_t destIndex,
          UBool move,
          UErrorCode *status);


U_STABLE void U_EXPORT2
utext_freeze(UText *ut);


enum {
    /**
     * It is potentially time consuming for the provider to determine the length of the text.
     * @stable ICU 3.4
     */
    UTEXT_PROVIDER_LENGTH_IS_EXPENSIVE = 1,
    /**
     * Text chunks remain valid and usable until the text object is modified or
     * deleted, not just until the next time the access() function is called
     * (which is the default).
     * @stable ICU 3.4
     */
    UTEXT_PROVIDER_STABLE_CHUNKS = 2,
    /**
     * The provider supports modifying the text via the replace() and copy()
     * functions.
     * @see Replaceable
     * @stable ICU 3.4
     */
    UTEXT_PROVIDER_WRITABLE = 3,
    /**
     * There is meta data associated with the text.
     * @see Replaceable::hasMetaData()
     * @stable ICU 3.4
     */ 
    UTEXT_PROVIDER_HAS_META_DATA = 4,
    /**
     * Text provider owns the text storage.
     *  Generally occurs as the result of a deep clone of the UText.
     *  When closing the UText, the associated text must
     *  also be closed/deleted/freed/ whatever is appropriate.
     * @stable ICU 3.6
     */
     UTEXT_PROVIDER_OWNS_TEXT = 5
};

typedef UText * U_CALLCONV
UTextClone(UText *dest, const UText *src, UBool deep, UErrorCode *status);


typedef int64_t U_CALLCONV
UTextNativeLength(UText *ut);

typedef UBool U_CALLCONV
UTextAccess(UText *ut, int64_t nativeIndex, UBool forward);

typedef int32_t U_CALLCONV
UTextExtract(UText *ut,
             int64_t nativeStart, int64_t nativeLimit,
             UChar *dest, int32_t destCapacity,
             UErrorCode *status);

typedef int32_t U_CALLCONV
UTextReplace(UText *ut,
             int64_t nativeStart, int64_t nativeLimit,
             const UChar *replacementText, int32_t replacmentLength,
             UErrorCode *status);

typedef void U_CALLCONV
UTextCopy(UText *ut,
          int64_t nativeStart, int64_t nativeLimit,
          int64_t nativeDest,
          UBool move,
          UErrorCode *status);

typedef int64_t U_CALLCONV
UTextMapOffsetToNative(const UText *ut);

typedef int32_t U_CALLCONV
UTextMapNativeIndexToUTF16(const UText *ut, int64_t nativeIndex);


typedef void U_CALLCONV
UTextClose(UText *ut);


struct UTextFuncs {
    /**
     *   (public)  Function table size, sizeof(UTextFuncs)
     *             Intended for use should the table grow to accomodate added
     *             functions in the future, to allow tests for older format
     *             function tables that do not contain the extensions.
     *
     *             Fields are placed for optimal alignment on
     *             32/64/128-bit-pointer machines, by normally grouping together
     *             4 32-bit fields,
     *             4 pointers,
     *             2 64-bit fields
     *             in sequence.
     *   @stable ICU 3.6
     */
    int32_t       tableSize;

    /**
      *   (private)  Alignment padding.
      *              Do not use, reserved for use by the UText framework only.
      *   @internal
      */
    int32_t       reserved1, /** @internal */ reserved2, /** @internal */ reserved3;


    /**
     * (public) Function pointer for UTextClone
     *
     * @see UTextClone
     * @stable ICU 3.6
     */
    UTextClone *clone;

    /**
     * (public) function pointer for UTextLength
     * May be expensive to compute!
     *
     * @see UTextLength
     * @stable ICU 3.6
     */
    UTextNativeLength *nativeLength;

    /**
     * (public) Function pointer for UTextAccess.
     *
     * @see UTextAccess
     * @stable ICU 3.6
     */
    UTextAccess *access;

    /**
     * (public) Function pointer for UTextExtract.
     *
     * @see UTextExtract
     * @stable ICU 3.6
     */
    UTextExtract *extract;

    /**
     * (public) Function pointer for UTextReplace.
     *
     * @see UTextReplace
     * @stable ICU 3.6
     */
    UTextReplace *replace;

    /**
     * (public) Function pointer for UTextCopy.
     *
     * @see UTextCopy
     * @stable ICU 3.6
     */
    UTextCopy *copy;

    /**
     * (public) Function pointer for UTextMapOffsetToNative.
     *
     * @see UTextMapOffsetToNative
     * @stable ICU 3.6
     */
    UTextMapOffsetToNative *mapOffsetToNative;

    /**
     * (public) Function pointer for UTextMapNativeIndexToUTF16.
     *
     * @see UTextMapNativeIndexToUTF16
     * @stable ICU 3.6
     */
    UTextMapNativeIndexToUTF16 *mapNativeIndexToUTF16;

    /**
     * (public) Function pointer for UTextClose.
      *
      * @see UTextClose
      * @stable ICU 3.6
      */
    UTextClose  *close;

    /**
      * (private)  Spare function pointer
      * @internal
      */

    UTextClose  *spare1;
    /**
      * (private)  Spare function pointer
      * @internal
      */
    UTextClose  *spare2;

    /**
      * (private)  Spare function pointer
      * @internal
      */
    UTextClose  *spare3;

};
typedef struct UTextFuncs UTextFuncs;

 /**
  *   UText struct.  Provides the interface between the generic UText access code
  *                  and the UText provider code that works on specific kinds of
  *                  text  (UTF-8, noncontiguous UTF-16, whatever.)
  *
  *                  Applications that are using predefined types of text providers
  *                  to pass text data to ICU services will have no need to view the
  *                  internals of the UText structs that they open.
  *
  * @stable ICU 3.6
  */
struct UText {
    /**
     *     (private)  Magic.  Used to help detect when UText functions are handed
     *                        invalid or unitialized UText structs.
     *                        utext_openXYZ() functions take an initialized,
     *                        but not necessarily open, UText struct as an
     *                        optional fill-in parameter.  This magic field
     *                        is used to check for that initialization.
     *                        Text provider close functions must NOT clear
     *                        the magic field because that would prevent
     *                        reuse of the UText struct.
     * @internal
     */
    uint32_t       magic;


    /**
     *     (private)  Flags for managing the allocation and freeing of
     *                memory associated with this UText.
     * @internal
     */
    int32_t        flags;


    /**
      *  Text provider properties.  This set of flags is maintainted by the
      *                             text provider implementation.
      *  @stable ICU 3.4
      */
    int32_t         providerProperties;

    /**
     * (public) sizeOfStruct=sizeof(UText)
     * Allows possible backward compatible extension.
     *
     * @stable ICU 3.4
     */
    int32_t         sizeOfStruct;
    
    /* ------ 16 byte alignment boundary -----------  */
    

    /**
      *  (protected) Native index of the first character position following
      *              the current chunk.
      *  @stable ICU 3.6
      */
    int64_t         chunkNativeLimit;

    /**
     *   (protected)  Size in bytes of the extra space (pExtra).
     *  @stable ICU 3.4
     */
    int32_t        extraSize;

    /**
      *    (protected) The highest chunk offset where native indexing and
      *    chunk (UTF-16) indexing correspond.  For UTF-16 sources, value
      *    will be equal to chunkLength.
      *
      *    @stable ICU 3.6
      */
    int32_t         nativeIndexingLimit;

    /* ---- 16 byte alignment boundary------ */
    
    /**
     *  (protected) Native index of the first character in the text chunk.
     *  @stable ICU 3.6
     */
    int64_t         chunkNativeStart;

    /**
     *  (protected) Current iteration position within the text chunk (UTF-16 buffer).
     *  This is the index to the character that will be returned by utext_next32().
     *  @stable ICU 3.6
     */
    int32_t         chunkOffset;

    /**
     *  (protected) Length the text chunk (UTF-16 buffer), in UChars.
     *  @stable ICU 3.6
     */
    int32_t         chunkLength;

    /* ---- 16  byte alignment boundary-- */
    

    /**
     *  (protected)  pointer to a chunk of text in UTF-16 format.
     *  May refer either to original storage of the source of the text, or
     *  if conversion was required, to a buffer owned by the UText.
     *  @stable ICU 3.6
     */
    const UChar    *chunkContents;

     /**
      * (public)     Pointer to Dispatch table for accessing functions for this UText.
      * @stable ICU 3.6
      */
    const UTextFuncs     *pFuncs;

    /**
     *  (protected)  Pointer to additional space requested by the
     *               text provider during the utext_open operation.
     * @stable ICU 3.4
     */
    void          *pExtra;

    /**
     * (protected) Pointer to string or text-containin object or similar.
     * This is the source of the text that this UText is wrapping, in a format
     *  that is known to the text provider functions.
     * @stable ICU 3.4
     */
    const void   *context;

    /* --- 16 byte alignment boundary--- */

    /**
     * (protected) Pointer fields available for use by the text provider.
     * Not used by UText common code.
     * @stable ICU 3.6
     */
    const void     *p; 
    /**
     * (protected) Pointer fields available for use by the text provider.
     * Not used by UText common code.
     * @stable ICU 3.6
     */
    const void     *q;
     /**
     * (protected) Pointer fields available for use by the text provider.
     * Not used by UText common code.
     * @stable ICU 3.6
      */
    const void     *r;

    /**
      *  Private field reserved for future use by the UText framework
      *     itself.  This is not to be touched by the text providers.
      * @internal ICU 3.4
      */
    void           *privP;


    /* --- 16 byte alignment boundary--- */
    

    /**
      * (protected) Integer field reserved for use by the text provider.
      * Not used by the UText framework, or by the client (user) of the UText.
      * @stable ICU 3.4
      */
    int64_t         a;

    /**
      * (protected) Integer field reserved for use by the text provider.
      * Not used by the UText framework, or by the client (user) of the UText.
      * @stable ICU 3.4
      */
    int32_t         b;

    /**
      * (protected) Integer field reserved for use by the text provider.
      * Not used by the UText framework, or by the client (user) of the UText.
      * @stable ICU 3.4
      */
    int32_t         c;

    /*  ---- 16 byte alignment boundary---- */


    /**
      *  Private field reserved for future use by the UText framework
      *     itself.  This is not to be touched by the text providers.
      * @internal ICU 3.4
      */
    int64_t         privA;
    /**
      *  Private field reserved for future use by the UText framework
      *     itself.  This is not to be touched by the text providers.
      * @internal ICU 3.4
      */
    int32_t         privB;
    /**
      *  Private field reserved for future use by the UText framework
      *     itself.  This is not to be touched by the text providers.
      * @internal ICU 3.4
      */
    int32_t         privC;
};


U_STABLE UText * U_EXPORT2
utext_setup(UText *ut, int32_t extraSpace, UErrorCode *status);

enum {
    UTEXT_MAGIC = 0x345ad82c
};

#define UTEXT_INITIALIZER {                                        \
                  UTEXT_MAGIC,          /* magic                */ \
                  0,                    /* flags                */ \
                  0,                    /* providerProps        */ \
                  sizeof(UText),        /* sizeOfStruct         */ \
                  0,                    /* chunkNativeLimit     */ \
                  0,                    /* extraSize            */ \
                  0,                    /* nativeIndexingLimit  */ \
                  0,                    /* chunkNativeStart     */ \
                  0,                    /* chunkOffset          */ \
                  0,                    /* chunkLength          */ \
                  NULL,                 /* chunkContents        */ \
                  NULL,                 /* pFuncs               */ \
                  NULL,                 /* pExtra               */ \
                  NULL,                 /* context              */ \
                  NULL, NULL, NULL,     /* p, q, r              */ \
                  NULL,                 /* privP                */ \
                  0, 0, 0,              /* a, b, c              */ \
                  0, 0, 0               /* privA,B,C,           */ \
                  }


U_CDECL_END



#endif
