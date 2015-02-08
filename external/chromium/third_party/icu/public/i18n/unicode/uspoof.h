

#ifndef USPOOF_H
#define USPOOF_H

#include "unicode/utypes.h"
#include "unicode/uset.h"
#include "unicode/parseerr.h"


#if !UCONFIG_NO_NORMALIZATION


#ifdef XP_CPLUSPLUS
#include "unicode/unistr.h"
#include "unicode/uniset.h"

U_NAMESPACE_USE
#endif


 /**
 *
 * \brief C API for Unicode Security and Spoofing Detection.
 *
 * These functions are intended to check strings, typically
 * identifiers of some type, such as URLs, for the presence of
 * characters that are likely to be visually confusing - 
 * for cases where the displayed form of an identifier may
 * not be what it appears to be.
 *
 * Unicode Technical Report #36, http://unicode.org/reports/tr36, and
 * Unicode Technical Standard #39, http://unicode.org/reports/tr39
 * "Unicode security considerations", give more background on 
 * security an spoofing issues with Unicode identifiers.
 * The tests and checks provided by this module implement the recommendations
 * from these Unicode documents.
 *
 * The tests available on identifiers fall into two general categories:
 *   -#  Single identier tests.  Check whether an identifier is
 *       potentially confusable with any other string, or is suspicious
 *       for other reasons.
 *   -#  Two identifier tests.  Check whether two specific identifiers are confusable.
 *       This does not consider whether either of strings is potentially
 *       confusable with any string other than the exact one specified.
 *
 * The steps to perform confusability testing are
 *   -#  Open a USpoofChecker.
 *   -#  Configure the USPoofChecker for the desired set of tests.  The tests that will
 *       be performed are specified by a set of USpoofChecks flags.
 *   -#  Perform the checks using the pre-configured USpoofChecker.  The results indicate
 *       which (if any) of the selected tests have identified possible problems with the identifier.
 *       Results are reported as a set of USpoofChecks flags;  this mirrors the form in which
 *       the set of tests to perform was originally specified tothe USpoofChecker.
 *
 * A USpoofChecker may be used repeatedly to perform checks on any number of identifiers.
 *
 * Thread Safety: The test functions for checking a single identifier, or for testing 
 * whether two identifiers are possible confusable, are thread safe.  
 * They may called concurrently, from multiple threads, using the same USpoofChecker instance.
 *
 * More generally, the standard ICU thread safety rules apply:  functions that take a
 * const USpoofChecker parameter are thread safe.  Those that take a non-const 
 * USpoofChecier are not thread safe.
 *
 *
 * Descriptions of the available checks.
 *
 * When testing whether pairs of identifiers are confusable, with the uspoof_areConfusable()
 * family of functions, the relevant tests are
 *
 *   -# USPOOF_SINGLE_SCRIPT_CONFUSABLE:  All of the characters from the two idenifiers are
 *      from a single script, and the two identifiers are visually confusable.
 *   -# USPOOF_MIXED_SCRIPT_CONFUSABLE:  At least one of the identifiers contains characters
 *      from more than one script, and the two identifiers are visually confusable.
 *   -# USPOOF_WHOLE_SCRIPT_CONFUSABLE: Each of the two idenifiers is of a single script, but
 *      the the two identifiers are from different scripts, and they are visually confusable.
 *
 * The safest approach is to enable all three of these checks as a group.
 *
 * USPOOF_ANY_CASE is a modifier for the above tests.  If the identifiers being checked can
 * be of mixed case and are used in a case-sensitive manner, this option should be specified.
 *
 * If the identiers being checked are used in a case-insensitive manner, and if they are
 * displayed to users in lower-case form only, the USPOOF_ANY_CASE option should not be
 * specified.  Confusabality issues involving upper case letters will not be reported.
 *
 * When performing tests on a single identifier, with the uspoof_check() family of functions,
 * the relevant tests are:
 *
 *    -# USPOOF_MIXED_SCRIPT_CONFUSABLE: the identifier contains characters from multiple
 *       scripts, and there exists an identier of a single script that is visually confusable.
 *    -# USPOOF_WHOLE_SCRIPT_CONFUSABLE: the identifier consists of characters from a single
 *       script, and there exists a visually confusable identifier.
 *       The visally confusable identifier also consists of characters from a single script.
 *       but not the same script as the identifier being checked.
 *    -# USPOOF_ANY_CASE: modifies the mixed script and whole script confusables tests.  If
 *       specified, the checks will confusable characters of any case.  If this flag is not
 *       set, the test is performed assuming case folded identifiers.
 *    -# USPOOF_SINGLE_SCRIPT: check that the identifier contains only characters from a
 *       single script.  (Characters from the 'common' and 'inherited' scripts are ignored.)
 *       This is not a test for confusable identifiers
 *    -# USPOOF_INVISIBLE: check an identifier for the presence of invisible characters,
 *       such as zero-width spaces, or character sequences that are
 *       likely not to display, such as multiple occurences of the same
 *       non-spacing mark.  This check does not test the input string as a whole
 *       for conformance to any particular syntax for identifiers.
 *    -# USPOOF_CHAR_LIMIT: check that an identifier contains only characters from a specified set
 *       of acceptable characters.  See uspoof_setAllowedChars() and
 *       uspoof_setAllowedLocales().
 *
 *  Note on Scripts:
 *     Characters from the Unicode Scripts "Common" and "Inherited" are ignored when consdering
 *     the script of an identifier. Common characters include digits and symbols that
 *     are normally used with text from more than one script.
 *
 */

struct USpoofChecker;
typedef struct USpoofChecker USpoofChecker; /**< typedef for C of USpoofChecker */

typedef enum USpoofChecks {
    /**   Single script confusable test.
      *   When testing whether two identifiers are confusable, report that they are if
      *   both are from the same script and they are visually confusable.
      *   Note: this test is not applicable to a check of a single identifier.
      */
    USPOOF_SINGLE_SCRIPT_CONFUSABLE =   1,

    /** Mixed script confusable test.
     *  When checking a single identifier, report a problem if
     *    the identifier contains multiple scripts, and
     *    is confusable with some other identifer in a single script
     *  When testing whether two identifiers are confusable, report that they are if
     *    the two IDs are visually confusable, and
     *    and at least one contains characters from more than one script.
     */
    USPOOF_MIXED_SCRIPT_CONFUSABLE  =   2,

    /** Whole script confusable test.
     *  When checking a single identifier, report a problem if
     *    The identifier is of a single script, and
     *    there exists a confusable identifier in another script.
     *  When testing whether two identfiers are confusable, report that they are if
     *    each is of a single script, 
     *    the scripts of the two identifiers are different, and
     *    the identifiers are visually confusable.
     */
    USPOOF_WHOLE_SCRIPT_CONFUSABLE  =   4,
    
    /** Any Case Modifier for confusable identifier tests.
        If specified, consider all characters, of any case, when looking for confusables.
        If USPOOF_ANY_CASE is not specified, identifiers being checked are assumed to have been
        case folded.  Upper case conusable characters will not be checked.
        Selects between Lower Case Confusable and
        Any Case Confusable.   */
    USPOOF_ANY_CASE                 =   8,

    /** Check that an identifer contains only characters from a
      * single script (plus chars from the common and inherited scripts.)
      * Applies to checks of a single identifier check only.
      */
    USPOOF_SINGLE_SCRIPT            =  16,
    
    /** Check an identifier for the presence of invisble characters,
      * such as zero-width spaces, or character sequences that are
      * likely not to display, such as multiple occurences of the same
      * non-spacing mark.  This check does not test the input string as a whole
      * for conformance to any particular syntax for identifiers.
      */
    USPOOF_INVISIBLE                =  32,

    /** Check that an identifier contains only characters from a specified set
      * of acceptable characters.  See uspoof_setAllowedChars() and
      * uspoof_setAllowedLocales().
      */
    USPOOF_CHAR_LIMIT               =  64,

    USPOOF_ALL_CHECKS               = 0x7f
    } USpoofChecks;
    
    
U_DRAFT USpoofChecker * U_EXPORT2
uspoof_open(UErrorCode *status);


U_CAPI USpoofChecker * U_EXPORT2
uspoof_openFromSerialized(const void *data, int32_t length, int32_t *pActualLength,
                          UErrorCode *pErrorCode);

U_CAPI USpoofChecker * U_EXPORT2
uspoof_openFromSource(const char *confusables,  int32_t confusablesLen,
                      const char *confusablesWholeScript, int32_t confusablesWholeScriptLen,
                      int32_t *errType, UParseError *pe, UErrorCode *status);


U_DRAFT void U_EXPORT2
uspoof_close(USpoofChecker *sc);

U_DRAFT USpoofChecker * U_EXPORT2
uspoof_clone(const USpoofChecker *sc, UErrorCode *status);


U_DRAFT void U_EXPORT2
uspoof_setChecks(USpoofChecker *sc, int32_t checks, UErrorCode *status);

U_DRAFT int32_t U_EXPORT2
uspoof_getChecks(const USpoofChecker *sc, UErrorCode *status);

U_DRAFT void U_EXPORT2
uspoof_setAllowedLocales(USpoofChecker *sc, const char *localesList, UErrorCode *status);

U_DRAFT const char * U_EXPORT2
uspoof_getAllowedLocales(USpoofChecker *sc, UErrorCode *status);


U_DRAFT void U_EXPORT2
uspoof_setAllowedChars(USpoofChecker *sc, const USet *chars, UErrorCode *status);


U_DRAFT const USet * U_EXPORT2
uspoof_getAllowedChars(const USpoofChecker *sc, UErrorCode *status);


#ifdef XP_CPLUSPLUS
U_DRAFT void U_EXPORT2
uspoof_setAllowedUnicodeSet(USpoofChecker *sc, const UnicodeSet *chars, UErrorCode *status);


U_DRAFT const UnicodeSet * U_EXPORT2
uspoof_getAllowedUnicodeSet(const USpoofChecker *sc, UErrorCode *status);
#endif


U_DRAFT int32_t U_EXPORT2
uspoof_check(const USpoofChecker *sc,
                         const UChar *text, int32_t length, 
                         int32_t *position,
                         UErrorCode *status);


U_DRAFT int32_t U_EXPORT2
uspoof_checkUTF8(const USpoofChecker *sc,
                 const char *text, int32_t length,
                 int32_t *position,
                 UErrorCode *status);


#ifdef XP_CPLUSPLUS
U_DRAFT int32_t U_EXPORT2
uspoof_checkUnicodeString(const USpoofChecker *sc,
                          const U_NAMESPACE_QUALIFIER UnicodeString &text, 
                          int32_t *position,
                          UErrorCode *status);

#endif


U_DRAFT int32_t U_EXPORT2
uspoof_areConfusable(const USpoofChecker *sc,
                     const UChar *s1, int32_t length1,
                     const UChar *s2, int32_t length2,
                     UErrorCode *status);



U_DRAFT int32_t U_EXPORT2
uspoof_areConfusableUTF8(const USpoofChecker *sc,
                         const char *s1, int32_t length1,
                         const char *s2, int32_t length2,
                         UErrorCode *status);




#ifdef XP_CPLUSPLUS
U_DRAFT int32_t U_EXPORT2
uspoof_areConfusableUnicodeString(const USpoofChecker *sc,
                                  const U_NAMESPACE_QUALIFIER UnicodeString &s1,
                                  const U_NAMESPACE_QUALIFIER UnicodeString &s2,
                                  UErrorCode *status);
#endif


U_DRAFT int32_t U_EXPORT2
uspoof_getSkeleton(const USpoofChecker *sc,
                   uint32_t type,
                   const UChar *s,  int32_t length,
                   UChar *dest, int32_t destCapacity,
                   UErrorCode *status);
    
U_DRAFT int32_t U_EXPORT2
uspoof_getSkeletonUTF8(const USpoofChecker *sc,
                       uint32_t type,
                       const char *s,  int32_t length,
                       char *dest, int32_t destCapacity,
                       UErrorCode *status);
    
#ifdef XP_CPLUSPLUS
U_DRAFT UnicodeString & U_EXPORT2
uspoof_getSkeletonUnicodeString(const USpoofChecker *sc,
                                uint32_t type,
                                const UnicodeString &s,
                                UnicodeString &dest,
                                UErrorCode *status);
#endif   /* XP_CPLUSPLUS */


U_CAPI int32_t U_EXPORT2
uspoof_serialize(USpoofChecker *sc,
                 void *data, int32_t capacity,
                 UErrorCode *status);


#endif

#endif   /* USPOOF_H */
