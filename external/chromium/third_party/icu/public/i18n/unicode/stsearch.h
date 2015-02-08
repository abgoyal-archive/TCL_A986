
#ifndef STSEARCH_H
#define STSEARCH_H

#include "unicode/utypes.h"

 
#if !UCONFIG_NO_COLLATION && !UCONFIG_NO_BREAK_ITERATION

#include "unicode/tblcoll.h"
#include "unicode/coleitr.h"
#include "unicode/search.h"

U_NAMESPACE_BEGIN


class U_I18N_API StringSearch : public SearchIterator
{
public:

    // public constructors and destructors --------------------------------

    /**
     * Creating a <tt>StringSearch</tt> instance using the argument locale 
     * language rule set. A collator will be created in the process, which 
     * will be owned by this instance and will be deleted during 
     * destruction
     * @param pattern The text for which this object will search.
     * @param text    The text in which to search for the pattern.
     * @param locale  A locale which defines the language-sensitive 
     *                comparison rules used to determine whether text in the 
     *                pattern and target matches. 
     * @param breakiter A <tt>BreakIterator</tt> object used to constrain 
     *                the matches that are found. Matches whose start and end 
     *                indices in the target text are not boundaries as 
     *                determined by the <tt>BreakIterator</tt> are 
     *                ignored. If this behavior is not desired, 
     *                <tt>NULL</tt> can be passed in instead.
     * @param status  for errors if any. If pattern or text is NULL, or if
     *               either the length of pattern or text is 0 then an 
     *               U_ILLEGAL_ARGUMENT_ERROR is returned.
     * @stable ICU 2.0
     */
    StringSearch(const UnicodeString &pattern, const UnicodeString &text,
                 const Locale        &locale,       
                       BreakIterator *breakiter,
                       UErrorCode    &status);

    /**
     * Creating a <tt>StringSearch</tt> instance using the argument collator 
     * language rule set. Note, user retains the ownership of this collator, 
     * it does not get destroyed during this instance's destruction.
     * @param pattern The text for which this object will search.
     * @param text    The text in which to search for the pattern.
     * @param coll    A <tt>RuleBasedCollator</tt> object which defines 
     *                the language-sensitive comparison rules used to 
     *                determine whether text in the pattern and target 
     *                matches. User is responsible for the clearing of this
     *                object.
     * @param breakiter A <tt>BreakIterator</tt> object used to constrain 
     *                the matches that are found. Matches whose start and end 
     *                indices in the target text are not boundaries as 
     *                determined by the <tt>BreakIterator</tt> are 
     *                ignored. If this behavior is not desired, 
     *                <tt>NULL</tt> can be passed in instead.
     * @param status for errors if any. If either the length of pattern or 
     *               text is 0 then an U_ILLEGAL_ARGUMENT_ERROR is returned.
     * @stable ICU 2.0
     */
    StringSearch(const UnicodeString     &pattern, 
                 const UnicodeString     &text,
                       RuleBasedCollator *coll,       
                       BreakIterator     *breakiter,
                       UErrorCode        &status);

    /**
     * Creating a <tt>StringSearch</tt> instance using the argument locale 
     * language rule set. A collator will be created in the process, which 
     * will be owned by this instance and will be deleted during 
     * destruction
     * <p>
     * Note: No parsing of the text within the <tt>CharacterIterator</tt> 
     * will be done during searching for this version. The block of text 
     * in <tt>CharacterIterator</tt> will be used as it is.
     * @param pattern The text for which this object will search.
     * @param text    The text iterator in which to search for the pattern.
     * @param locale  A locale which defines the language-sensitive 
     *                comparison rules used to determine whether text in the 
     *                pattern and target matches. User is responsible for 
     *                the clearing of this object.
     * @param breakiter A <tt>BreakIterator</tt> object used to constrain 
     *                the matches that are found. Matches whose start and end 
     *                indices in the target text are not boundaries as 
     *                determined by the <tt>BreakIterator</tt> are 
     *                ignored. If this behavior is not desired, 
     *                <tt>NULL</tt> can be passed in instead.
     * @param status for errors if any. If either the length of pattern or 
     *               text is 0 then an U_ILLEGAL_ARGUMENT_ERROR is returned.
     * @stable ICU 2.0
     */
    StringSearch(const UnicodeString &pattern, CharacterIterator &text,
                 const Locale        &locale, 
                       BreakIterator *breakiter,
                       UErrorCode    &status);

    /**
     * Creating a <tt>StringSearch</tt> instance using the argument collator 
     * language rule set. Note, user retains the ownership of this collator, 
     * it does not get destroyed during this instance's destruction.
     * <p>
     * Note: No parsing of the text within the <tt>CharacterIterator</tt> 
     * will be done during searching for this version. The block of text 
     * in <tt>CharacterIterator</tt> will be used as it is.
     * @param pattern The text for which this object will search.
     * @param text    The text in which to search for the pattern.
     * @param coll    A <tt>RuleBasedCollator</tt> object which defines 
     *                the language-sensitive comparison rules used to 
     *                determine whether text in the pattern and target 
     *                matches. User is responsible for the clearing of this
     *                object.
     * @param breakiter A <tt>BreakIterator</tt> object used to constrain 
     *                the matches that are found. Matches whose start and end 
     *                indices in the target text are not boundaries as 
     *                determined by the <tt>BreakIterator</tt> are 
     *                ignored. If this behavior is not desired, 
     *                <tt>NULL</tt> can be passed in instead.
     * @param status for errors if any. If either the length of pattern or 
     *               text is 0 then an U_ILLEGAL_ARGUMENT_ERROR is returned.
     * @stable ICU 2.0
     */
    StringSearch(const UnicodeString     &pattern, CharacterIterator &text,
                       RuleBasedCollator *coll, 
                       BreakIterator     *breakiter,
                       UErrorCode        &status);

    /**
     * Copy constructor that creates a StringSearch instance with the same 
     * behavior, and iterating over the same text.
     * @param that StringSearch instance to be copied.
     * @stable ICU 2.0
     */
    StringSearch(const StringSearch &that);

    /**
    * Destructor. Cleans up the search iterator data struct.
    * If a collator is created in the constructor, it will be destroyed here.
    * @stable ICU 2.0
    */
    virtual ~StringSearch(void);

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
    StringSearch *clone() const;

    // operator overloading ---------------------------------------------

    /**
     * Assignment operator. Sets this iterator to have the same behavior,
     * and iterate over the same text, as the one passed in.
     * @param that instance to be copied.
     * @stable ICU 2.0
     */
    StringSearch & operator=(const StringSearch &that);

    /**
     * Equality operator. 
     * @param that instance to be compared.
     * @return TRUE if both instances have the same attributes, 
     *         breakiterators, collators and iterate over the same text 
     *         while looking for the same pattern.
     * @stable ICU 2.0
     */
    virtual UBool operator==(const SearchIterator &that) const;

    // public get and set methods ----------------------------------------

    /**
     * Sets the index to point to the given position, and clears any state 
     * that's affected.
     * <p>
     * This method takes the argument index and sets the position in the text 
     * string accordingly without checking if the index is pointing to a 
     * valid starting point to begin searching. 
     * @param position within the text to be set. If position is less
     *          than or greater than the text range for searching, 
     *          an U_INDEX_OUTOFBOUNDS_ERROR will be returned
     * @param status for errors if it occurs
     * @stable ICU 2.0
     */
    virtual void setOffset(int32_t position, UErrorCode &status);

    /**
     * Return the current index in the text being searched.
     * If the iteration has gone past the end of the text
     * (or past the beginning for a backwards search), USEARCH_DONE
     * is returned.
     * @return current index in the text being searched.
     * @stable ICU 2.0
     */
    virtual int32_t getOffset(void) const;

    /**
     * Set the target text to be searched.
     * Text iteration will hence begin at the start of the text string. 
     * This method is 
     * useful if you want to re-use an iterator to search for the same 
     * pattern within a different body of text.
     * @param text text string to be searched
     * @param status for errors if any. If the text length is 0 then an 
     *        U_ILLEGAL_ARGUMENT_ERROR is returned.
     * @stable ICU 2.0
     */
    virtual void setText(const UnicodeString &text, UErrorCode &status);
    
    /**
     * Set the target text to be searched.
     * Text iteration will hence begin at the start of the text string. 
     * This method is 
     * useful if you want to re-use an iterator to search for the same 
     * pattern within a different body of text.
     * Note: No parsing of the text within the <tt>CharacterIterator</tt> 
     * will be done during searching for this version. The block of text 
     * in <tt>CharacterIterator</tt> will be used as it is.
     * @param text text string to be searched
     * @param status for errors if any. If the text length is 0 then an 
     *        U_ILLEGAL_ARGUMENT_ERROR is returned.
     * @stable ICU 2.0
     */
    virtual void setText(CharacterIterator &text, UErrorCode &status);

    /**
     * Gets the collator used for the language rules.
     * <p>
     * Caller may modify but <b>must not</b> delete the <tt>RuleBasedCollator</tt>!
     * Modifications to this collator will affect the original collator passed in to 
     * the <tt>StringSearch></tt> constructor or to setCollator, if any.
     * @return collator used for string search
     * @stable ICU 2.0
     */
    RuleBasedCollator * getCollator() const;
    
    /**
     * Sets the collator used for the language rules. User retains the 
     * ownership of this collator, thus the responsibility of deletion lies 
     * with the user. This method causes internal data such as Boyer-Moore 
     * shift tables to be recalculated, but the iterator's position is 
     * unchanged.
     * @param coll    collator 
     * @param status  for errors if any
     * @stable ICU 2.0
     */
    void setCollator(RuleBasedCollator *coll, UErrorCode &status);
    
    /**
     * Sets the pattern used for matching.
     * Internal data like the Boyer Moore table will be recalculated, but 
     * the iterator's position is unchanged.
     * @param pattern search pattern to be found
     * @param status for errors if any. If the pattern length is 0 then an 
     *               U_ILLEGAL_ARGUMENT_ERROR is returned.
     * @stable ICU 2.0
     */
    void setPattern(const UnicodeString &pattern, UErrorCode &status);
    
    /**
     * Gets the search pattern.
     * @return pattern used for matching
     * @stable ICU 2.0
     */
    const UnicodeString & getPattern() const;

    // public methods ----------------------------------------------------

    /** 
     * Reset the iteration.
     * Search will begin at the start of the text string if a forward 
     * iteration is initiated before a backwards iteration. Otherwise if 
     * a backwards iteration is initiated before a forwards iteration, the 
     * search will begin at the end of the text string.
     * @stable ICU 2.0
     */
    virtual void reset();

    /**
     * Returns a copy of StringSearch with the same behavior, and 
     * iterating over the same text, as this one. Note that all data will be
     * replicated, except for the user-specified collator and the
     * breakiterator.
     * @return cloned object
     * @stable ICU 2.0
     */
    virtual SearchIterator * safeClone(void) const;
    
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

protected:

    // protected method -------------------------------------------------

    /**
     * Search forward for matching text, starting at a given location.
     * Clients should not call this method directly; instead they should 
     * call {@link SearchIterator#next }.
     * <p>
     * If a match is found, this method returns the index at which the match
     * starts and calls {@link SearchIterator#setMatchLength } with the number 
     * of characters in the target text that make up the match. If no match 
     * is found, the method returns <tt>USEARCH_DONE</tt>.
     * <p>
     * The <tt>StringSearch</tt> is adjusted so that its current index 
     * (as returned by {@link #getOffset }) is the match position if one was 
     * found.
     * If a match is not found, <tt>USEARCH_DONE</tt> will be returned and
     * the <tt>StringSearch</tt> will be adjusted to the index USEARCH_DONE.
     * @param position The index in the target text at which the search 
     *                 starts
     * @param status for errors if any occurs
     * @return The index at which the matched text in the target starts, or 
     *         USEARCH_DONE if no match was found.
     * @stable ICU 2.0
     */
    virtual int32_t handleNext(int32_t position, UErrorCode &status);

    /**
     * Search backward for matching text, starting at a given location.
     * Clients should not call this method directly; instead they should call
     * <tt>SearchIterator.previous()</tt>, which this method overrides.
     * <p>
     * If a match is found, this method returns the index at which the match
     * starts and calls {@link SearchIterator#setMatchLength } with the number 
     * of characters in the target text that make up the match. If no match 
     * is found, the method returns <tt>USEARCH_DONE</tt>.
     * <p>
     * The <tt>StringSearch</tt> is adjusted so that its current index 
     * (as returned by {@link #getOffset }) is the match position if one was 
     * found.
     * If a match is not found, <tt>USEARCH_DONE</tt> will be returned and
     * the <tt>StringSearch</tt> will be adjusted to the index USEARCH_DONE.
     * @param position The index in the target text at which the search 
     *                 starts.
     * @param status for errors if any occurs
     * @return The index at which the matched text in the target starts, or 
     *         USEARCH_DONE if no match was found.
     * @stable ICU 2.0
     */
    virtual int32_t handlePrev(int32_t position, UErrorCode &status);
    
private :
    StringSearch(); // default constructor not implemented

    // private data members ----------------------------------------------

    /**
    * RuleBasedCollator, contains exactly the same UCollator * in m_strsrch_
    * @stable ICU 2.0
    */
    RuleBasedCollator  m_collator_;
    /**
    * Pattern text
    * @stable ICU 2.0
    */
    UnicodeString      m_pattern_;
    /**
    * String search struct data
    * @stable ICU 2.0
    */
    UStringSearch     *m_strsrch_;

};

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_COLLATION */

#endif

