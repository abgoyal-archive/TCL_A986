

// A collection of utilities for font handling.

// FIXME: Move all methods to the files that have their callsites and remove this file.
// *Utils files are not very WebKit-ty.

#ifndef FontUtilsWin_h
#define FontUtilsWin_h

#include <usp10.h>
#include <wchar.h>
#include <windows.h>

#include "FontDescription.h"
#include <unicode/uscript.h>

namespace WebCore {

// Return a font family that supports a script and belongs to |generic| font
// family.  It can return NULL and a caller has to implement its own fallback.
const UChar* getFontFamilyForScript(UScriptCode, FontDescription::GenericFamilyType);

// Return a font family that can render |characters| based on
// what script characters belong to. When char_checked is non-NULL,
// it's filled with the character used to determine the script.
// When script_checked is non-NULL, the script used to determine
// the family is returned.
// FIXME: This function needs a total overhaul.
const UChar* getFallbackFamily(const UChar* characters, int length,
                               FontDescription::GenericFamilyType,
                               UChar32* charChecked,
                               UScriptCode* scriptChecked);

// Derive a new HFONT by replacing lfFaceName of LOGFONT with |family|,
// calculate the ascent for the derived HFONT, and initialize SCRIPT_CACHE
// in FontData.
// |style| is only used for cache key generation. |style| is
// bit-wise OR of BOLD(1), UNDERLINED(2) and ITALIC(4) and
// should match what's contained in LOGFONT. It should be calculated
// by calling GetStyleFromLogFont.
// Returns false if the font is not accessible, in which case |ascent| field
// of |fontdata| is set to kUndefinedAscent.
// Be aware that this is not thread-safe.
// FIXME: Instead of having three out params, we'd better have one
// (|*FontData|), but somehow it mysteriously messes up the layout for
// certain complex script pages (e.g. hi.wikipedia.org) and also crashes
// at the start-up if recently visited page list includes pages with complex
// scripts in their title. Moreover, somehow the very first-pass of
// intl2 page-cycler test is noticeably slower with one out param than
// the current version although the subsequent 9 passes take about the
// same time.
bool getDerivedFontData(const UChar* family, int style, LOGFONT*, int* ascent, HFONT*, SCRIPT_CACHE**);

enum {
    FontStyleNormal = 0,
    FontStyleBold = 1,
    FontStyleItalic = 2,
    FontStyleUnderlined = 4
};

// Derive style (bit-wise OR of FONT_STYLE_BOLD, FONT_STYLE_UNDERLINED, and
// FONT_STYLE_ITALIC) from LOGFONT. Returns 0 if |*logfont| is NULL.
int getStyleFromLogfont(const LOGFONT*);

}  // namespace WebCore

#endif  // FontUtilsWin_h
