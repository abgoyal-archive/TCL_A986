

#ifndef StyleBase_h
#define StyleBase_h

#include <wtf/RefCounted.h>

namespace WebCore {

    class String;
    class StyleSheet;
    class KURL;

    // Base class for most CSS DOM objects.

    // FIXME: We don't need these to all share one base class.
    // Refactor so they don't any more.

    class StyleBase : public RefCounted<StyleBase> {
    public:
        virtual ~StyleBase() { }

        StyleBase* parent() const { return m_parent; }
        void setParent(StyleBase* parent) { m_parent = parent; }

        // returns the url of the style sheet this object belongs to
        KURL baseURL() const;

        virtual bool isCSSStyleSheet() const { return false; }
        virtual bool isCharsetRule() { return false; }
        virtual bool isFontFaceRule() { return false; }
        virtual bool isImportRule() { return false; }
        virtual bool isKeyframeRule() { return false; }
        virtual bool isKeyframesRule() { return false; }
        virtual bool isMediaRule() { return false; }
        virtual bool isVariablesRule() { return false; }
        
        virtual bool isRule() { return false; }
        virtual bool isStyleRule() { return false; }
        virtual bool isStyleSheet() const { return false; }
        virtual bool isXSLStyleSheet() const { return false; }

        virtual bool isMutableStyleDeclaration() const { return false; }

        virtual String cssText() const;

        virtual void checkLoaded();

        bool useStrictParsing() const { return !m_parent || m_parent->useStrictParsing(); }

        virtual void insertedIntoParent() { }

        StyleSheet* stylesheet();

#ifdef ANDROID_INSTRUMENT
        // Overridden to prevent the normal new from being called.
        void* operator new(size_t size);
        void* operator new[](size_t size);

        // Overridden to prevent the normal delete from being called.
        void operator delete(void* p, size_t size);
        void operator delete[](void* p, size_t size);

        static size_t reportStyleSize();
#endif

    protected:
        StyleBase(StyleBase* parent)
            : m_parent(parent)
        {
        }

    private:
        StyleBase* m_parent;
    };
}

#endif
