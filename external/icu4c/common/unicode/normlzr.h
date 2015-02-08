
#ifndef NORMLZR_H
#define NORMLZR_H

#include "unicode/utypes.h"

 
#if !UCONFIG_NO_NORMALIZATION

#include "unicode/chariter.h"
#include "unicode/normalizer2.h"
#include "unicode/unistr.h"
#include "unicode/unorm.h"
#include "unicode/uobject.h"

U_NAMESPACE_BEGIN
class U_COMMON_API Normalizer : public UObject {
public:
  /**
   * If DONE is returned from an iteration function that returns a code point,
   * then there are no more normalization results available.
   * @stable ICU 2.0
   */
  enum {
      DONE=0xffff
  };

  // Constructors

  /**
   * Creates a new <code>Normalizer</code> object for iterating over the
   * normalized form of a given string.
   * <p>
   * @param str   The string to be normalized.  The normalization
   *              will start at the beginning of the string.
   *
   * @param mode  The normalization mode.
   * @stable ICU 2.0
   */
  Normalizer(const UnicodeString& str, UNormalizationMode mode);

  /**
   * Creates a new <code>Normalizer</code> object for iterating over the
   * normalized form of a given string.
   * <p>
   * @param str   The string to be normalized.  The normalization
   *              will start at the beginning of the string.
   *
   * @param length Length of the string, or -1 if NUL-terminated.
   * @param mode  The normalization mode.
   * @stable ICU 2.0
   */
  Normalizer(const UChar* str, int32_t length, UNormalizationMode mode);

  /**
   * Creates a new <code>Normalizer</code> object for iterating over the
   * normalized form of the given text.
   * <p>
   * @param iter  The input text to be normalized.  The normalization
   *              will start at the beginning of the string.
   *
   * @param mode  The normalization mode.
   * @stable ICU 2.0
   */
  Normalizer(const CharacterIterator& iter, UNormalizationMode mode);

  /**
   * Copy constructor.
   * @param copy The object to be copied.
   * @stable ICU 2.0
   */
  Normalizer(const Normalizer& copy);

  /**
   * Destructor
   * @stable ICU 2.0
   */
  virtual ~Normalizer();


  //-------------------------------------------------------------------------
  // Static utility methods
  //-------------------------------------------------------------------------

  /**
   * Normalizes a <code>UnicodeString</code> according to the specified normalization mode.
   * This is a wrapper for unorm_normalize(), using UnicodeString's.
   *
   * The <code>options</code> parameter specifies which optional
   * <code>Normalizer</code> features are to be enabled for this operation.
   *
   * @param source    the input string to be normalized.
   * @param mode      the normalization mode
   * @param options   the optional features to be enabled (0 for no options)
   * @param result    The normalized string (on output).
   * @param status    The error code.
   * @stable ICU 2.0
   */
  static void U_EXPORT2 normalize(const UnicodeString& source,
                        UNormalizationMode mode, int32_t options,
                        UnicodeString& result,
                        UErrorCode &status);

  /**
   * Compose a <code>UnicodeString</code>.
   * This is equivalent to normalize() with mode UNORM_NFC or UNORM_NFKC.
   * This is a wrapper for unorm_normalize(), using UnicodeString's.
   *
   * The <code>options</code> parameter specifies which optional
   * <code>Normalizer</code> features are to be enabled for this operation.
   *
   * @param source    the string to be composed.
   * @param compat    Perform compatibility decomposition before composition.
   *                  If this argument is <code>FALSE</code>, only canonical
   *                  decomposition will be performed.
   * @param options   the optional features to be enabled (0 for no options)
   * @param result    The composed string (on output).
   * @param status    The error code.
   * @stable ICU 2.0
   */
  static void U_EXPORT2 compose(const UnicodeString& source,
                      UBool compat, int32_t options,
                      UnicodeString& result,
                      UErrorCode &status);

  /**
   * Static method to decompose a <code>UnicodeString</code>.
   * This is equivalent to normalize() with mode UNORM_NFD or UNORM_NFKD.
   * This is a wrapper for unorm_normalize(), using UnicodeString's.
   *
   * The <code>options</code> parameter specifies which optional
   * <code>Normalizer</code> features are to be enabled for this operation.
   *
   * @param source    the string to be decomposed.
   * @param compat    Perform compatibility decomposition.
   *                  If this argument is <code>FALSE</code>, only canonical
   *                  decomposition will be performed.
   * @param options   the optional features to be enabled (0 for no options)
   * @param result    The decomposed string (on output).
   * @param status    The error code.
   * @stable ICU 2.0
   */
  static void U_EXPORT2 decompose(const UnicodeString& source,
                        UBool compat, int32_t options,
                        UnicodeString& result,
                        UErrorCode &status);

  /**
   * Performing quick check on a string, to quickly determine if the string is
   * in a particular normalization format.
   * This is a wrapper for unorm_quickCheck(), using a UnicodeString.
   *
   * Three types of result can be returned UNORM_YES, UNORM_NO or
   * UNORM_MAYBE. Result UNORM_YES indicates that the argument
   * string is in the desired normalized format, UNORM_NO determines that
   * argument string is not in the desired normalized format. A
   * UNORM_MAYBE result indicates that a more thorough check is required,
   * the user may have to put the string in its normalized form and compare the
   * results.
   * @param source       string for determining if it is in a normalized format
   * @param mode         normalization format
   * @param status A reference to a UErrorCode to receive any errors
   * @return UNORM_YES, UNORM_NO or UNORM_MAYBE
   *
   * @see isNormalized
   * @stable ICU 2.0
   */
  static inline UNormalizationCheckResult
  quickCheck(const UnicodeString &source, UNormalizationMode mode, UErrorCode &status);

  /**
   * Performing quick check on a string; same as the other version of quickCheck
   * but takes an extra options parameter like most normalization functions.
   *
   * @param source       string for determining if it is in a normalized format
   * @param mode         normalization format
   * @param options      the optional features to be enabled (0 for no options)
   * @param status A reference to a UErrorCode to receive any errors
   * @return UNORM_YES, UNORM_NO or UNORM_MAYBE
   *
   * @see isNormalized
   * @stable ICU 2.6
   */
  static UNormalizationCheckResult
  quickCheck(const UnicodeString &source, UNormalizationMode mode, int32_t options, UErrorCode &status);

  /**
   * Test if a string is in a given normalization form.
   * This is semantically equivalent to source.equals(normalize(source, mode)) .
   *
   * Unlike unorm_quickCheck(), this function returns a definitive result,
   * never a "maybe".
   * For NFD, NFKD, and FCD, both functions work exactly the same.
   * For NFC and NFKC where quickCheck may return "maybe", this function will
   * perform further tests to arrive at a TRUE/FALSE result.
   *
   * @param src        String that is to be tested if it is in a normalization format.
   * @param mode       Which normalization form to test for.
   * @param errorCode  ICU error code in/out parameter.
   *                   Must fulfill U_SUCCESS before the function call.
   * @return Boolean value indicating whether the source string is in the
   *         "mode" normalization form.
   *
   * @see quickCheck
   * @stable ICU 2.2
   */
  static inline UBool
  isNormalized(const UnicodeString &src, UNormalizationMode mode, UErrorCode &errorCode);

  /**
   * Test if a string is in a given normalization form; same as the other version of isNormalized
   * but takes an extra options parameter like most normalization functions.
   *
   * @param src        String that is to be tested if it is in a normalization format.
   * @param mode       Which normalization form to test for.
   * @param options      the optional features to be enabled (0 for no options)
   * @param errorCode  ICU error code in/out parameter.
   *                   Must fulfill U_SUCCESS before the function call.
   * @return Boolean value indicating whether the source string is in the
   *         "mode" normalization form.
   *
   * @see quickCheck
   * @stable ICU 2.6
   */
  static UBool
  isNormalized(const UnicodeString &src, UNormalizationMode mode, int32_t options, UErrorCode &errorCode);

  /**
   * Concatenate normalized strings, making sure that the result is normalized as well.
   *
   * If both the left and the right strings are in
   * the normalization form according to "mode/options",
   * then the result will be
   *
   * \code
   *     dest=normalize(left+right, mode, options)
   * \endcode
   *
   * For details see unorm_concatenate in unorm.h.
   *
   * @param left Left source string.
   * @param right Right source string.
   * @param result The output string.
   * @param mode The normalization mode.
   * @param options A bit set of normalization options.
   * @param errorCode ICU error code in/out parameter.
   *                   Must fulfill U_SUCCESS before the function call.
   * @return result
   *
   * @see unorm_concatenate
   * @see normalize
   * @see unorm_next
   * @see unorm_previous
   *
   * @stable ICU 2.1
   */
  static UnicodeString &
  U_EXPORT2 concatenate(UnicodeString &left, UnicodeString &right,
              UnicodeString &result,
              UNormalizationMode mode, int32_t options,
              UErrorCode &errorCode);

  /**
   * Compare two strings for canonical equivalence.
   * Further options include case-insensitive comparison and
   * code point order (as opposed to code unit order).
   *
   * Canonical equivalence between two strings is defined as their normalized
   * forms (NFD or NFC) being identical.
   * This function compares strings incrementally instead of normalizing
   * (and optionally case-folding) both strings entirely,
   * improving performance significantly.
   *
   * Bulk normalization is only necessary if the strings do not fulfill the FCD
   * conditions. Only in this case, and only if the strings are relatively long,
   * is memory allocated temporarily.
   * For FCD strings and short non-FCD strings there is no memory allocation.
   *
   * Semantically, this is equivalent to
   *   strcmp[CodePointOrder](NFD(foldCase(s1)), NFD(foldCase(s2)))
   * where code point order and foldCase are all optional.
   *
   * UAX 21 2.5 Caseless Matching specifies that for a canonical caseless match
   * the case folding must be performed first, then the normalization.
   *
   * @param s1 First source string.
   * @param s2 Second source string.
   *
   * @param options A bit set of options:
   *   - U_FOLD_CASE_DEFAULT or 0 is used for default options:
   *     Case-sensitive comparison in code unit order, and the input strings
   *     are quick-checked for FCD.
   *
   *   - UNORM_INPUT_IS_FCD
   *     Set if the caller knows that both s1 and s2 fulfill the FCD conditions.
   *     If not set, the function will quickCheck for FCD
   *     and normalize if necessary.
   *
   *   - U_COMPARE_CODE_POINT_ORDER
   *     Set to choose code point order instead of code unit order
   *     (see u_strCompare for details).
   *
   *   - U_COMPARE_IGNORE_CASE
   *     Set to compare strings case-insensitively using case folding,
   *     instead of case-sensitively.
   *     If set, then the following case folding options are used.
   *
   *   - Options as used with case-insensitive comparisons, currently:
   *
   *   - U_FOLD_CASE_EXCLUDE_SPECIAL_I
   *    (see u_strCaseCompare for details)
   *
   *   - regular normalization options shifted left by UNORM_COMPARE_NORM_OPTIONS_SHIFT
   *
   * @param errorCode ICU error code in/out parameter.
   *                  Must fulfill U_SUCCESS before the function call.
   * @return <0 or 0 or >0 as usual for string comparisons
   *
   * @see unorm_compare
   * @see normalize
   * @see UNORM_FCD
   * @see u_strCompare
   * @see u_strCaseCompare
   *
   * @stable ICU 2.2
   */
  static inline int32_t
  compare(const UnicodeString &s1, const UnicodeString &s2,
          uint32_t options,
          UErrorCode &errorCode);

  //-------------------------------------------------------------------------
  // Iteration API
  //-------------------------------------------------------------------------

  /**
   * Return the current character in the normalized text.
   * current() may need to normalize some text at getIndex().
   * The getIndex() is not changed.
   *
   * @return the current normalized code point
   * @stable ICU 2.0
   */
  UChar32              current(void);

  /**
   * Return the first character in the normalized text.
   * This is equivalent to setIndexOnly(startIndex()) followed by next().
   * (Post-increment semantics.)
   *
   * @return the first normalized code point
   * @stable ICU 2.0
   */
  UChar32              first(void);

  /**
   * Return the last character in the normalized text.
   * This is equivalent to setIndexOnly(endIndex()) followed by previous().
   * (Pre-decrement semantics.)
   *
   * @return the last normalized code point
   * @stable ICU 2.0
   */
  UChar32              last(void);

  /**
   * Return the next character in the normalized text.
   * (Post-increment semantics.)
   * If the end of the text has already been reached, DONE is returned.
   * The DONE value could be confused with a U+FFFF non-character code point
   * in the text. If this is possible, you can test getIndex()<endIndex()
   * before calling next(), or (getIndex()<endIndex() || last()!=DONE)
   * after calling next(). (Calling last() will change the iterator state!)
   *
   * The C API unorm_next() is more efficient and does not have this ambiguity.
   *
   * @return the next normalized code point
   * @stable ICU 2.0
   */
  UChar32              next(void);

  /**
   * Return the previous character in the normalized text and decrement.
   * (Pre-decrement semantics.)
   * If the beginning of the text has already been reached, DONE is returned.
   * The DONE value could be confused with a U+FFFF non-character code point
   * in the text. If this is possible, you can test
   * (getIndex()>startIndex() || first()!=DONE). (Calling first() will change
   * the iterator state!)
   *
   * The C API unorm_previous() is more efficient and does not have this ambiguity.
   *
   * @return the previous normalized code point
   * @stable ICU 2.0
   */
  UChar32              previous(void);

  /**
   * Set the iteration position in the input text that is being normalized,
   * without any immediate normalization.
   * After setIndexOnly(), getIndex() will return the same index that is
   * specified here.
   *
   * @param index the desired index in the input text.
   * @stable ICU 2.0
   */
  void                 setIndexOnly(int32_t index);

  /**
   * Reset the index to the beginning of the text.
   * This is equivalent to setIndexOnly(startIndex)).
   * @stable ICU 2.0
   */
  void                reset(void);

  /**
   * Retrieve the current iteration position in the input text that is
   * being normalized.
   *
   * A following call to next() will return a normalized code point from
   * the input text at or after this index.
   *
   * After a call to previous(), getIndex() will point at or before the
   * position in the input text where the normalized code point
   * was returned from with previous().
   *
   * @return the current index in the input text
   * @stable ICU 2.0
   */
  int32_t            getIndex(void) const;

  /**
   * Retrieve the index of the start of the input text. This is the begin index
   * of the <code>CharacterIterator</code> or the start (i.e. index 0) of the string
   * over which this <code>Normalizer</code> is iterating.
   *
   * @return the smallest index in the input text where the Normalizer operates
   * @stable ICU 2.0
   */
  int32_t            startIndex(void) const;

  /**
   * Retrieve the index of the end of the input text. This is the end index
   * of the <code>CharacterIterator</code> or the length of the string
   * over which this <code>Normalizer</code> is iterating.
   * This end index is exclusive, i.e., the Normalizer operates only on characters
   * before this index.
   *
   * @return the first index in the input text where the Normalizer does not operate
   * @stable ICU 2.0
   */
  int32_t            endIndex(void) const;

  /**
   * Returns TRUE when both iterators refer to the same character in the same
   * input text.
   *
   * @param that a Normalizer object to compare this one to
   * @return comparison result
   * @stable ICU 2.0
   */
  UBool        operator==(const Normalizer& that) const;

  /**
   * Returns FALSE when both iterators refer to the same character in the same
   * input text.
   *
   * @param that a Normalizer object to compare this one to
   * @return comparison result
   * @stable ICU 2.0
   */
  inline UBool        operator!=(const Normalizer& that) const;

  /**
   * Returns a pointer to a new Normalizer that is a clone of this one.
   * The caller is responsible for deleting the new clone.
   * @return a pointer to a new Normalizer
   * @stable ICU 2.0
   */
  Normalizer*        clone(void) const;

  /**
   * Generates a hash code for this iterator.
   *
   * @return the hash code
   * @stable ICU 2.0
   */
  int32_t                hashCode(void) const;

  //-------------------------------------------------------------------------
  // Property access methods
  //-------------------------------------------------------------------------

  /**
   * Set the normalization mode for this object.
   * <p>
   * <b>Note:</b>If the normalization mode is changed while iterating
   * over a string, calls to {@link #next() } and {@link #previous() } may
   * return previously buffers characters in the old normalization mode
   * until the iteration is able to re-sync at the next base character.
   * It is safest to call {@link #setIndexOnly }, {@link #reset() },
   * {@link #setText }, {@link #first() },
   * {@link #last() }, etc. after calling <code>setMode</code>.
   * <p>
   * @param newMode the new mode for this <code>Normalizer</code>.
   * @see #getUMode
   * @stable ICU 2.0
   */
  void setMode(UNormalizationMode newMode);

  /**
   * Return the normalization mode for this object.
   *
   * This is an unusual name because there used to be a getMode() that
   * returned a different type.
   *
   * @return the mode for this <code>Normalizer</code>
   * @see #setMode
   * @stable ICU 2.0
   */
  UNormalizationMode getUMode(void) const;

  /**
   * Set options that affect this <code>Normalizer</code>'s operation.
   * Options do not change the basic composition or decomposition operation
   * that is being performed, but they control whether
   * certain optional portions of the operation are done.
   * Currently the only available option is obsolete.
   *
   * It is possible to specify multiple options that are all turned on or off.
   *
   * @param   option  the option(s) whose value is/are to be set.
   * @param   value   the new setting for the option.  Use <code>TRUE</code> to
   *                  turn the option(s) on and <code>FALSE</code> to turn it/them off.
   *
   * @see #getOption
   * @stable ICU 2.0
   */
  void setOption(int32_t option,
         UBool value);

  /**
   * Determine whether an option is turned on or off.
   * If multiple options are specified, then the result is TRUE if any
   * of them are set.
   * <p>
   * @param option the option(s) that are to be checked
   * @return TRUE if any of the option(s) are set
   * @see #setOption
   * @stable ICU 2.0
   */
  UBool getOption(int32_t option) const;

  /**
   * Set the input text over which this <code>Normalizer</code> will iterate.
   * The iteration position is set to the beginning.
   *
   * @param newText a string that replaces the current input text
   * @param status a UErrorCode
   * @stable ICU 2.0
   */
  void setText(const UnicodeString& newText,
           UErrorCode &status);

  /**
   * Set the input text over which this <code>Normalizer</code> will iterate.
   * The iteration position is set to the beginning.
   *
   * @param newText a CharacterIterator object that replaces the current input text
   * @param status a UErrorCode
   * @stable ICU 2.0
   */
  void setText(const CharacterIterator& newText,
           UErrorCode &status);

  /**
   * Set the input text over which this <code>Normalizer</code> will iterate.
   * The iteration position is set to the beginning.
   *
   * @param newText a string that replaces the current input text
   * @param length the length of the string, or -1 if NUL-terminated
   * @param status a UErrorCode
   * @stable ICU 2.0
   */
  void setText(const UChar* newText,
                    int32_t length,
            UErrorCode &status);
  /**
   * Copies the input text into the UnicodeString argument.
   *
   * @param result Receives a copy of the text under iteration.
   * @stable ICU 2.0
   */
  void            getText(UnicodeString&  result);

  /**
   * ICU "poor man's RTTI", returns a UClassID for this class.
   * @returns a UClassID for this class.
   * @stable ICU 2.2
   */
  static UClassID U_EXPORT2 getStaticClassID();

  /**
   * ICU "poor man's RTTI", returns a UClassID for the actual class.
   * @return a UClassID for the actual class.
   * @stable ICU 2.2
   */
  virtual UClassID getDynamicClassID() const;

private:
  //-------------------------------------------------------------------------
  // Private functions
  //-------------------------------------------------------------------------

  Normalizer(); // default constructor not implemented
  Normalizer &operator=(const Normalizer &that); // assignment operator not implemented

  // Private utility methods for iteration
  // For documentation, see the source code
  UBool nextNormalize();
  UBool previousNormalize();

  void    init();
  void    clearBuffer(void);

  //-------------------------------------------------------------------------
  // Private data
  //-------------------------------------------------------------------------

  FilteredNormalizer2*fFilteredNorm2;  // owned if not NULL
  const Normalizer2  *fNorm2;  // not owned; may be equal to fFilteredNorm2
  UNormalizationMode  fUMode;
  int32_t             fOptions;

  // The input text and our position in it
  CharacterIterator  *text;

  // The normalization buffer is the result of normalization
  // of the source in [currentIndex..nextIndex[ .
  int32_t         currentIndex, nextIndex;

  // A buffer for holding intermediate results
  UnicodeString       buffer;
  int32_t         bufferPos;
};

//-------------------------------------------------------------------------
// Inline implementations
//-------------------------------------------------------------------------

inline UBool
Normalizer::operator!= (const Normalizer& other) const
{ return ! operator==(other); }

inline UNormalizationCheckResult
Normalizer::quickCheck(const UnicodeString& source,
                       UNormalizationMode mode,
                       UErrorCode &status) {
    return quickCheck(source, mode, 0, status);
}

inline UBool
Normalizer::isNormalized(const UnicodeString& source,
                         UNormalizationMode mode,
                         UErrorCode &status) {
    return isNormalized(source, mode, 0, status);
}

inline int32_t
Normalizer::compare(const UnicodeString &s1, const UnicodeString &s2,
                    uint32_t options,
                    UErrorCode &errorCode) {
  // all argument checking is done in unorm_compare
  return unorm_compare(s1.getBuffer(), s1.length(),
                       s2.getBuffer(), s2.length(),
                       options,
                       &errorCode);
}

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_NORMALIZATION */

#endif // NORMLZR_H
