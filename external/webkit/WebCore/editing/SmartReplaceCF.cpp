

#include "config.h"
#include "SmartReplace.h"

#include <CoreFoundation/CFCharacterSet.h>
#include <CoreFoundation/CFString.h>

namespace WebCore {

static CFMutableCharacterSetRef getSmartSet(bool isPreviousCharacter)
{
    static CFMutableCharacterSetRef preSmartSet = NULL; 
    static CFMutableCharacterSetRef postSmartSet = NULL;
    CFMutableCharacterSetRef smartSet = isPreviousCharacter ? preSmartSet : postSmartSet;
    if (!smartSet) {
        smartSet = CFCharacterSetCreateMutable(kCFAllocatorDefault);
        CFCharacterSetAddCharactersInString(smartSet, isPreviousCharacter ? CFSTR("([\"\'#$/-`{") : CFSTR(")].,;:?\'!\"%*-/}"));
        CFCharacterSetUnion(smartSet, CFCharacterSetGetPredefined(kCFCharacterSetWhitespaceAndNewline));        
        // Adding CJK ranges
        CFCharacterSetAddCharactersInRange(smartSet, CFRangeMake(0x1100, 256)); // Hangul Jamo (0x1100 - 0x11FF)
        CFCharacterSetAddCharactersInRange(smartSet, CFRangeMake(0x2E80, 352)); // CJK & Kangxi Radicals (0x2E80 - 0x2FDF)
        CFCharacterSetAddCharactersInRange(smartSet, CFRangeMake(0x2FF0, 464)); // Ideograph Descriptions, CJK Symbols, Hiragana, Katakana, Bopomofo, Hangul Compatibility Jamo, Kanbun, & Bopomofo Ext (0x2FF0 - 0x31BF)
        CFCharacterSetAddCharactersInRange(smartSet, CFRangeMake(0x3200, 29392)); // Enclosed CJK, CJK Ideographs (Uni Han & Ext A), & Yi (0x3200 - 0xA4CF)
        CFCharacterSetAddCharactersInRange(smartSet, CFRangeMake(0xAC00, 11183)); // Hangul Syllables (0xAC00 - 0xD7AF)
        CFCharacterSetAddCharactersInRange(smartSet, CFRangeMake(0xF900, 352)); // CJK Compatibility Ideographs (0xF900 - 0xFA5F)
        CFCharacterSetAddCharactersInRange(smartSet, CFRangeMake(0xFE30, 32)); // CJK Compatibility From (0xFE30 - 0xFE4F)
        CFCharacterSetAddCharactersInRange(smartSet, CFRangeMake(0xFF00, 240)); // Half/Full Width Form (0xFF00 - 0xFFEF)
        CFCharacterSetAddCharactersInRange(smartSet, CFRangeMake(0x20000, 0xA6D7)); // CJK Ideograph Exntension B
        CFCharacterSetAddCharactersInRange(smartSet, CFRangeMake(0x2F800, 0x021E)); // CJK Compatibility Ideographs (0x2F800 - 0x2FA1D)

        if (isPreviousCharacter)
            preSmartSet = smartSet;
        else {
            CFCharacterSetUnion(smartSet, CFCharacterSetGetPredefined(kCFCharacterSetPunctuation));       
            postSmartSet = smartSet;
        }
    }
    return smartSet;
}

bool isCharacterSmartReplaceExempt(UChar32 c, bool isPreviousCharacter)
{
    return CFCharacterSetIsLongCharacterMember(getSmartSet(isPreviousCharacter), c);
}

}
