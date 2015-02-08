

namespace WebCore {
    class Range;
}

@interface NSAttributedString (WebKitExtras)

+ (NSAttributedString *)_web_attributedStringFromRange:(WebCore::Range*)range;
- (NSAttributedString *)_web_attributedStringByStrippingAttachmentCharacters;

@end
