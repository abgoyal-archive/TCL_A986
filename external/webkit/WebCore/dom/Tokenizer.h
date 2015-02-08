

#ifndef Tokenizer_h
#define Tokenizer_h

namespace WebCore {

    class SegmentedString;
    class XSSAuditor;

    class Tokenizer : public Noncopyable {
    public:
        virtual ~Tokenizer() { }

        // Script output must be prepended, while new data
        // received during executing a script must be appended, hence the
        // extra bool to be able to distinguish between both cases.
        // document.write() always uses false, while the loader uses true.
        virtual void write(const SegmentedString&, bool appendData) = 0;
        virtual void finish() = 0;
        virtual bool isWaitingForScripts() const = 0;
        virtual void stopParsing() { m_parserStopped = true; }
        virtual bool processingData() const { return false; }
        virtual int executingScript() const { return 0; }

        virtual bool wantsRawData() const { return false; }
        virtual bool writeRawData(const char* /*data*/, int /*length*/) { return false; }

        bool inViewSourceMode() const { return m_inViewSourceMode; }
        void setInViewSourceMode(bool mode) { m_inViewSourceMode = mode; }

        virtual bool wellFormed() const { return true; }

        virtual int lineNumber() const { return -1; }
        virtual int columnNumber() const { return -1; }
        
        virtual void executeScriptsWaitingForStylesheets() {}

        virtual bool isHTMLTokenizer() const { return false; }
        
        XSSAuditor* xssAuditor() const { return m_XSSAuditor; }
        void setXSSAuditor(XSSAuditor* auditor) { m_XSSAuditor = auditor; }

    protected:
        Tokenizer(bool viewSourceMode = false) 
            : m_parserStopped(false)
            , m_inViewSourceMode(viewSourceMode)
            , m_XSSAuditor(0)
        {
        }

        // The tokenizer has buffers, so parsing may continue even after
        // it stops receiving data. We use m_parserStopped to stop the tokenizer
        // even when it has buffered data.
        bool m_parserStopped;
        bool m_inViewSourceMode;
        
        // The XSSAuditor associated with this tokenizer.
        XSSAuditor* m_XSSAuditor;
    };

} // namespace WebCore

#endif // Tokenizer_h
