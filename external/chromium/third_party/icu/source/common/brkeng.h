
#ifndef BRKENG_H
#define BRKENG_H

#include "unicode/utypes.h"
#include "unicode/uobject.h"
#include "unicode/utext.h"
#include "unicode/uscript.h"

U_NAMESPACE_BEGIN

class UnicodeSet;
class UStack;
class CompactTrieDictionary;


class LanguageBreakEngine : public UMemory {
 public:

  /**
   * <p>Default constructor.</p>
   *
   */
  LanguageBreakEngine();

  /**
   * <p>Virtual destructor.</p>
   */
  virtual ~LanguageBreakEngine();

 /**
  * <p>Indicate whether this engine handles a particular character for
  * a particular kind of break.</p>
  *
  * @param c A character which begins a run that the engine might handle
  * @param breakType The type of text break which the caller wants to determine
  * @return TRUE if this engine handles the particular character and break
  * type.
  */
  virtual UBool handles(UChar32 c, int32_t breakType) const = 0;

 /**
  * <p>Find any breaks within a run in the supplied text.</p>
  *
  * @param text A UText representing the text. The
  * iterator is left at the end of the run of characters which the engine
  * is capable of handling.
  * @param startPos The start of the run within the supplied text.
  * @param endPos The end of the run within the supplied text.
  * @param reverse Whether the caller is looking for breaks in a reverse
  * direction.
  * @param breakType The type of break desired, or -1.
  * @param foundBreaks An allocated C array of the breaks found, if any
  * @return The number of breaks found.
  */
  virtual int32_t findBreaks( UText *text,
                              int32_t startPos,
                              int32_t endPos,
                              UBool reverse,
                              int32_t breakType,
                              UStack &foundBreaks ) const = 0;

};


class LanguageBreakFactory : public UMemory {
 public:

  /**
   * <p>Default constructor.</p>
   *
   */
  LanguageBreakFactory();

  /**
   * <p>Virtual destructor.</p>
   */
  virtual ~LanguageBreakFactory();

 /**
  * <p>Find and return a LanguageBreakEngine that can find the desired
  * kind of break for the set of characters to which the supplied
  * character belongs. It is up to the set of available engines to
  * determine what the sets of characters are.</p>
  *
  * @param c A character that begins a run for which a LanguageBreakEngine is
  * sought.
  * @param breakType The kind of text break for which a LanguageBreakEngine is
  * sought.
  * @return A LanguageBreakEngine with the desired characteristics, or 0.
  */
  virtual const LanguageBreakEngine *getEngineFor(UChar32 c, int32_t breakType) = 0;

};



class UnhandledEngine : public LanguageBreakEngine {
 private:

    /**
     * The sets of characters handled, for each break type
     * @internal
     */

  UnicodeSet    *fHandled[4];

 public:

  /**
   * <p>Default constructor.</p>
   *
   */
  UnhandledEngine(UErrorCode &status);

  /**
   * <p>Virtual destructor.</p>
   */
  virtual ~UnhandledEngine();

 /**
  * <p>Indicate whether this engine handles a particular character for
  * a particular kind of break.</p>
  *
  * @param c A character which begins a run that the engine might handle
  * @param breakType The type of text break which the caller wants to determine
  * @return TRUE if this engine handles the particular character and break
  * type.
  */
  virtual UBool handles(UChar32 c, int32_t breakType) const;

 /**
  * <p>Find any breaks within a run in the supplied text.</p>
  *
  * @param text A UText representing the text (TODO: UText). The
  * iterator is left at the end of the run of characters which the engine
  * is capable of handling.
  * @param startPos The start of the run within the supplied text.
  * @param endPos The end of the run within the supplied text.
  * @param reverse Whether the caller is looking for breaks in a reverse
  * direction.
  * @param breakType The type of break desired, or -1.
  * @param foundBreaks An allocated C array of the breaks found, if any
  * @return The number of breaks found.
  */
  virtual int32_t findBreaks( UText *text,
                              int32_t startPos,
                              int32_t endPos,
                              UBool reverse,
                              int32_t breakType,
                              UStack &foundBreaks ) const;

 /**
  * <p>Tell the engine to handle a particular character and break type.</p>
  *
  * @param c A character which the engine should handle
  * @param breakType The type of text break for which the engine should handle c
  */
  virtual void handleCharacter(UChar32 c, int32_t breakType);

};


class ICULanguageBreakFactory : public LanguageBreakFactory {
 private:

    /**
     * The stack of break engines created by this factory
     * @internal
     */

  UStack    *fEngines;

 public:

  /**
   * <p>Standard constructor.</p>
   *
   */
  ICULanguageBreakFactory(UErrorCode &status);

  /**
   * <p>Virtual destructor.</p>
   */
  virtual ~ICULanguageBreakFactory();

 /**
  * <p>Find and return a LanguageBreakEngine that can find the desired
  * kind of break for the set of characters to which the supplied
  * character belongs. It is up to the set of available engines to
  * determine what the sets of characters are.</p>
  *
  * @param c A character that begins a run for which a LanguageBreakEngine is
  * sought.
  * @param breakType The kind of text break for which a LanguageBreakEngine is
  * sought.
  * @return A LanguageBreakEngine with the desired characteristics, or 0.
  */
  virtual const LanguageBreakEngine *getEngineFor(UChar32 c, int32_t breakType);

 protected:

 /**
  * <p>Create a LanguageBreakEngine for the set of characters to which
  * the supplied character belongs, for the specified break type.</p>
  *
  * @param c A character that begins a run for which a LanguageBreakEngine is
  * sought.
  * @param breakType The kind of text break for which a LanguageBreakEngine is
  * sought.
  * @return A LanguageBreakEngine with the desired characteristics, or 0.
  */
  virtual const LanguageBreakEngine *loadEngineFor(UChar32 c, int32_t breakType);

 /**
  * <p>Create a CompactTrieDictionary for the specified script and break type.</p>
  *
  * @param script An ISO 15924 script code that identifies the dictionary to be
  * created.
  * @param breakType The kind of text break for which a dictionary is
  * sought.
  * @return A CompactTrieDictionary with the desired characteristics, or 0.
  */
  virtual const CompactTrieDictionary *loadDictionaryFor(UScriptCode script, int32_t breakType);

};

U_NAMESPACE_END

    /* BRKENG_H */
#endif
