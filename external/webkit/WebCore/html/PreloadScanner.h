

#ifndef PreloadScanner_h
#define PreloadScanner_h

#include "AtomicString.h"
#include "SegmentedString.h"
#include <wtf/Noncopyable.h>
#include <wtf/Vector.h>

namespace WebCore {
    
    class CachedResource;
    class CachedResourceClient;
    class Document;
    
    class PreloadScanner : public Noncopyable {
    public:
        PreloadScanner(Document*);
        ~PreloadScanner();
        void begin();
        void write(const SegmentedString&);
        void end();
        bool inProgress() const { return m_inProgress; }
        
        bool scanningBody() const;
        
        static unsigned consumeEntity(SegmentedString&, bool& notEnoughCharacters);
        
    private:
        void tokenize(const SegmentedString&);
        void reset();
        
        void emitTag();
        void emitCharacter(UChar);
        
        void tokenizeCSS(UChar);
        void emitCSSRule();
        
        void processAttribute();

        
        void clearLastCharacters();
        void rememberCharacter(UChar);
        bool lastCharactersMatch(const char*, unsigned count) const;
        
        bool m_inProgress;
        SegmentedString m_source;
        
        enum State {
            Data,
            EntityData,
            TagOpen,
            CloseTagOpen,
            TagName,
            BeforeAttributeName,
            AttributeName,
            AfterAttributeName,
            BeforeAttributeValue,
            AttributeValueDoubleQuoted,
            AttributeValueSingleQuoted,
            AttributeValueUnquoted,
            EntityInAttributeValue,
            BogusComment,
            MarkupDeclarationOpen,
            CommentStart,
            CommentStartDash,
            Comment,
            CommentEndDash,
            CommentEnd
        };
        State m_state;
        bool m_escape;
        enum ContentModel {
            PCDATA,
            RCDATA,
            CDATA,
            PLAINTEXT
        };
        ContentModel m_contentModel;
        unsigned m_commentPos;
        State m_stateBeforeEntityInAttributeValue;
        
        static const unsigned lastCharactersBufferSize = 8;
        UChar m_lastCharacters[lastCharactersBufferSize];
        unsigned m_lastCharacterIndex;
        
        bool m_closeTag;
        Vector<UChar, 32> m_tagName;
        Vector<UChar, 32> m_attributeName;
        Vector<UChar> m_attributeValue;
        AtomicString m_lastStartTag;
        
        String m_urlToLoad;
        String m_charset;
        bool m_linkIsStyleSheet;
        
        enum CSSState {
            CSSInitial,
            CSSMaybeComment,
            CSSComment,
            CSSMaybeCommentEnd,
            CSSRuleStart,
            CSSRule,
            CSSAfterRule,
            CSSRuleValue,
            CSSAfterRuleValue
        };
        CSSState m_cssState;
        Vector<UChar, 16> m_cssRule;
        Vector<UChar> m_cssRuleValue;
        
        double m_timeUsed;
        
        bool m_bodySeen;
        Document* m_document;
    };

}

#endif
