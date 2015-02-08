
#ifndef CHARITER_H
#define CHARITER_H

#include "unicode/utypes.h"
#include "unicode/uobject.h"
#include "unicode/unistr.h"
 
U_NAMESPACE_BEGIN
class U_COMMON_API ForwardCharacterIterator : public UObject {
public:
    /**
     * Value returned by most of ForwardCharacterIterator's functions
     * when the iterator has reached the limits of its iteration.
     * @stable ICU 2.0
     */
    enum { DONE = 0xffff };
    
    /**
     * Destructor.  
     * @stable ICU 2.0
     */
    virtual ~ForwardCharacterIterator();
    
    /**
     * Returns true when both iterators refer to the same
     * character in the same character-storage object.  
     * @param that The ForwardCharacterIterator to be compared for equality
     * @return true when both iterators refer to the same
     * character in the same character-storage object
     * @stable ICU 2.0
     */
    virtual UBool operator==(const ForwardCharacterIterator& that) const = 0;
    
    /**
     * Returns true when the iterators refer to different
     * text-storage objects, or to different characters in the
     * same text-storage object.  
     * @param that The ForwardCharacterIterator to be compared for inequality
     * @return true when the iterators refer to different
     * text-storage objects, or to different characters in the
     * same text-storage object
     * @stable ICU 2.0
     */
    inline UBool operator!=(const ForwardCharacterIterator& that) const;
    
    /**
     * Generates a hash code for this iterator.  
     * @return the hash code.
     * @stable ICU 2.0
     */
    virtual int32_t hashCode(void) const = 0;
    
    /**
     * Returns a UClassID for this ForwardCharacterIterator ("poor man's
     * RTTI").<P> Despite the fact that this function is public,
     * DO NOT CONSIDER IT PART OF CHARACTERITERATOR'S API! 
     * @return a UClassID for this ForwardCharacterIterator 
     * @stable ICU 2.0
     */
    virtual UClassID getDynamicClassID(void) const = 0;
    
    /**
     * Gets the current code unit for returning and advances to the next code unit
     * in the iteration range
     * (toward endIndex()).  If there are
     * no more code units to return, returns DONE.
     * @return the current code unit.
     * @stable ICU 2.0
     */
    virtual UChar         nextPostInc(void) = 0;
    
    /**
     * Gets the current code point for returning and advances to the next code point
     * in the iteration range
     * (toward endIndex()).  If there are
     * no more code points to return, returns DONE.
     * @return the current code point.
     * @stable ICU 2.0
     */
    virtual UChar32       next32PostInc(void) = 0;
    
    /**
     * Returns FALSE if there are no more code units or code points
     * at or after the current position in the iteration range.
     * This is used with nextPostInc() or next32PostInc() in forward
     * iteration.
     * @returns FALSE if there are no more code units or code points
     * at or after the current position in the iteration range.
     * @stable ICU 2.0
     */
    virtual UBool        hasNext() = 0;
    
protected:
    /** Default constructor to be overridden in the implementing class. @stable ICU 2.0*/
    ForwardCharacterIterator();
    
    /** Copy constructor to be overridden in the implementing class. @stable ICU 2.0*/
    ForwardCharacterIterator(const ForwardCharacterIterator &other);
    
    /**
     * Assignment operator to be overridden in the implementing class.
     * @stable ICU 2.0
     */
    ForwardCharacterIterator &operator=(const ForwardCharacterIterator&) { return *this; }
};

class U_COMMON_API CharacterIterator : public ForwardCharacterIterator {
public:
    /**
     * Origin enumeration for the move() and move32() functions.
     * @stable ICU 2.0
     */
    enum EOrigin { kStart, kCurrent, kEnd };

    /**
     * Returns a pointer to a new CharacterIterator of the same
     * concrete class as this one, and referring to the same
     * character in the same text-storage object as this one.  The
     * caller is responsible for deleting the new clone.  
     * @return a pointer to a new CharacterIterator
     * @stable ICU 2.0
     */
    virtual CharacterIterator* clone(void) const = 0;

    /**
     * Sets the iterator to refer to the first code unit in its
     * iteration range, and returns that code unit.
     * This can be used to begin an iteration with next().
     * @return the first code unit in its iteration range.
     * @stable ICU 2.0
     */
    virtual UChar         first(void) = 0;

    /**
     * Sets the iterator to refer to the first code unit in its
     * iteration range, returns that code unit, and moves the position
     * to the second code unit. This is an alternative to setToStart()
     * for forward iteration with nextPostInc().
     * @return the first code unit in its iteration range.
     * @stable ICU 2.0
     */
    virtual UChar         firstPostInc(void);

    /**
     * Sets the iterator to refer to the first code point in its
     * iteration range, and returns that code unit,
     * This can be used to begin an iteration with next32().
     * Note that an iteration with next32PostInc(), beginning with,
     * e.g., setToStart() or firstPostInc(), is more efficient.
     * @return the first code point in its iteration range.
     * @stable ICU 2.0
     */
    virtual UChar32       first32(void) = 0;

    /**
     * Sets the iterator to refer to the first code point in its
     * iteration range, returns that code point, and moves the position
     * to the second code point. This is an alternative to setToStart()
     * for forward iteration with next32PostInc().
     * @return the first code point in its iteration range.
     * @stable ICU 2.0
     */
    virtual UChar32       first32PostInc(void);

    /**
     * Sets the iterator to refer to the first code unit or code point in its
     * iteration range. This can be used to begin a forward
     * iteration with nextPostInc() or next32PostInc().
     * @return the start position of the iteration range
     * @stable ICU 2.0
     */
    inline int32_t    setToStart();

    /**
     * Sets the iterator to refer to the last code unit in its
     * iteration range, and returns that code unit.
     * This can be used to begin an iteration with previous().
     * @return the last code unit.
     * @stable ICU 2.0
     */
    virtual UChar         last(void) = 0;
        
    /**
     * Sets the iterator to refer to the last code point in its
     * iteration range, and returns that code unit.
     * This can be used to begin an iteration with previous32().
     * @return the last code point.
     * @stable ICU 2.0
     */
    virtual UChar32       last32(void) = 0;

    /**
     * Sets the iterator to the end of its iteration range, just behind
     * the last code unit or code point. This can be used to begin a backward
     * iteration with previous() or previous32().
     * @return the end position of the iteration range
     * @stable ICU 2.0
     */
    inline int32_t    setToEnd();

    /**
     * Sets the iterator to refer to the "position"-th code unit
     * in the text-storage object the iterator refers to, and
     * returns that code unit.  
     * @param position the "position"-th code unit in the text-storage object
     * @return the "position"-th code unit.
     * @stable ICU 2.0
     */
    virtual UChar         setIndex(int32_t position) = 0;

    /**
     * Sets the iterator to refer to the beginning of the code point
     * that contains the "position"-th code unit
     * in the text-storage object the iterator refers to, and
     * returns that code point.
     * The current position is adjusted to the beginning of the code point
     * (its first code unit).
     * @param position the "position"-th code unit in the text-storage object
     * @return the "position"-th code point.
     * @stable ICU 2.0
     */
    virtual UChar32       setIndex32(int32_t position) = 0;

    /**
     * Returns the code unit the iterator currently refers to. 
     * @return the current code unit. 
     * @stable ICU 2.0
     */
    virtual UChar         current(void) const = 0;
        
    /**
     * Returns the code point the iterator currently refers to.  
     * @return the current code point.
     * @stable ICU 2.0
     */
    virtual UChar32       current32(void) const = 0;
        
    /**
     * Advances to the next code unit in the iteration range
     * (toward endIndex()), and returns that code unit.  If there are
     * no more code units to return, returns DONE.
     * @return the next code unit.
     * @stable ICU 2.0
     */
    virtual UChar         next(void) = 0;
        
    /**
     * Advances to the next code point in the iteration range
     * (toward endIndex()), and returns that code point.  If there are
     * no more code points to return, returns DONE.
     * Note that iteration with "pre-increment" semantics is less
     * efficient than iteration with "post-increment" semantics
     * that is provided by next32PostInc().
     * @return the next code point.
     * @stable ICU 2.0
     */
    virtual UChar32       next32(void) = 0;
        
    /**
     * Advances to the previous code unit in the iteration range
     * (toward startIndex()), and returns that code unit.  If there are
     * no more code units to return, returns DONE.  
     * @return the previous code unit.
     * @stable ICU 2.0
     */
    virtual UChar         previous(void) = 0;

    /**
     * Advances to the previous code point in the iteration range
     * (toward startIndex()), and returns that code point.  If there are
     * no more code points to return, returns DONE. 
     * @return the previous code point. 
     * @stable ICU 2.0
     */
    virtual UChar32       previous32(void) = 0;

    /**
     * Returns FALSE if there are no more code units or code points
     * before the current position in the iteration range.
     * This is used with previous() or previous32() in backward
     * iteration.
     * @return FALSE if there are no more code units or code points
     * before the current position in the iteration range, return TRUE otherwise.
     * @stable ICU 2.0
     */
    virtual UBool        hasPrevious() = 0;

    /**
     * Returns the numeric index in the underlying text-storage
     * object of the character returned by first().  Since it's
     * possible to create an iterator that iterates across only
     * part of a text-storage object, this number isn't
     * necessarily 0.  
     * @returns the numeric index in the underlying text-storage
     * object of the character returned by first().
     * @stable ICU 2.0
     */
    inline int32_t       startIndex(void) const;
        
    /**
     * Returns the numeric index in the underlying text-storage
     * object of the position immediately BEYOND the character
     * returned by last().  
     * @return the numeric index in the underlying text-storage
     * object of the position immediately BEYOND the character
     * returned by last().
     * @stable ICU 2.0
     */
    inline int32_t       endIndex(void) const;
        
    /**
     * Returns the numeric index in the underlying text-storage
     * object of the character the iterator currently refers to
     * (i.e., the character returned by current()).  
     * @return the numberic index in the text-storage object of 
     * the character the iterator currently refers to
     * @stable ICU 2.0
     */
    inline int32_t       getIndex(void) const;

    /**
     * Returns the length of the entire text in the underlying
     * text-storage object.
     * @return the length of the entire text in the text-storage object
     * @stable ICU 2.0
     */
    inline int32_t           getLength() const;

    /**
     * Moves the current position relative to the start or end of the
     * iteration range, or relative to the current position itself.
     * The movement is expressed in numbers of code units forward
     * or backward by specifying a positive or negative delta.
     * @param delta the position relative to origin. A positive delta means forward;
     * a negative delta means backward.
     * @param origin Origin enumeration {kStart, kCurrent, kEnd}
     * @return the new position
     * @stable ICU 2.0
     */
    virtual int32_t      move(int32_t delta, EOrigin origin) = 0;

    /**
     * Moves the current position relative to the start or end of the
     * iteration range, or relative to the current position itself.
     * The movement is expressed in numbers of code points forward
     * or backward by specifying a positive or negative delta.
     * @param delta the position relative to origin. A positive delta means forward;
     * a negative delta means backward.
     * @param origin Origin enumeration {kStart, kCurrent, kEnd}
     * @return the new position
     * @stable ICU 2.0
     */
    virtual int32_t      move32(int32_t delta, EOrigin origin) = 0;

    /**
     * Copies the text under iteration into the UnicodeString
     * referred to by "result".  
     * @param result Receives a copy of the text under iteration.  
     * @stable ICU 2.0
     */
    virtual void            getText(UnicodeString&  result) = 0;

protected:
    /**
     * Empty constructor.
     * @stable ICU 2.0
     */
    CharacterIterator();

    /**
     * Constructor, just setting the length field in this base class.
     * @stable ICU 2.0
     */
    CharacterIterator(int32_t length);

    /**
     * Constructor, just setting the length and position fields in this base class.
     * @stable ICU 2.0
     */
    CharacterIterator(int32_t length, int32_t position);

    /**
     * Constructor, just setting the length, start, end, and position fields in this base class.
     * @stable ICU 2.0
     */
    CharacterIterator(int32_t length, int32_t textBegin, int32_t textEnd, int32_t position);
  
    /**
     * Copy constructor.
     *
     * @param that The CharacterIterator to be copied
     * @stable ICU 2.0
     */
    CharacterIterator(const CharacterIterator &that);

    /**
     * Assignment operator.  Sets this CharacterIterator to have the same behavior,
     * as the one passed in.
     * @param that The CharacterIterator passed in.
     * @return the newly set CharacterIterator.
     * @stable ICU 2.0
     */
    CharacterIterator &operator=(const CharacterIterator &that);

    /**
     * Base class text length field.
     * Necessary this for correct getText() and hashCode().
     * @stable ICU 2.0
     */
    int32_t textLength;

    /**
     * Base class field for the current position.
     * @stable ICU 2.0
     */
    int32_t  pos;

    /**
     * Base class field for the start of the iteration range.
     * @stable ICU 2.0
     */
    int32_t  begin;

    /**
     * Base class field for the end of the iteration range.
     * @stable ICU 2.0
     */
    int32_t  end;
};

inline UBool
ForwardCharacterIterator::operator!=(const ForwardCharacterIterator& that) const {
    return !operator==(that);
}

inline int32_t
CharacterIterator::setToStart() {
    return move(0, kStart);
}

inline int32_t
CharacterIterator::setToEnd() {
    return move(0, kEnd);
}

inline int32_t
CharacterIterator::startIndex(void) const {
    return begin;
}

inline int32_t
CharacterIterator::endIndex(void) const {
    return end;
}

inline int32_t
CharacterIterator::getIndex(void) const {
    return pos;
}

inline int32_t
CharacterIterator::getLength(void) const {
    return textLength;
}

U_NAMESPACE_END
#endif
