

#include "config.h"
#include "PluginDocument.h"

#include "DocumentLoader.h"
#include "Element.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "FrameLoaderClient.h"
#include "HTMLEmbedElement.h"
#include "HTMLNames.h"
#include "MainResourceLoader.h"
#include "Page.h"
#include "RenderWidget.h"
#include "SegmentedString.h"
#include "Settings.h"
#include "Text.h"
#include "XMLTokenizer.h"

namespace WebCore {
    
using namespace HTMLNames;
    
class PluginTokenizer : public Tokenizer {
public:
    PluginTokenizer(Document* doc) : m_doc(doc), m_embedElement(0) {}
        
private:
    virtual void write(const SegmentedString&, bool appendData);
    virtual void stopParsing();
    virtual void finish();
    virtual bool isWaitingForScripts() const;
        
    virtual bool wantsRawData() const { return true; }
    virtual bool writeRawData(const char* data, int len);
        
    void createDocumentStructure();

    Document* m_doc;
    HTMLEmbedElement* m_embedElement;
};
    
void PluginTokenizer::write(const SegmentedString&, bool)
{
    ASSERT_NOT_REACHED();
}
    
void PluginTokenizer::createDocumentStructure()
{
    ExceptionCode ec;
    RefPtr<Element> rootElement = m_doc->createElement(htmlTag, false);
    m_doc->appendChild(rootElement, ec);

    RefPtr<Element> body = m_doc->createElement(bodyTag, false);
    body->setAttribute(marginwidthAttr, "0");
    body->setAttribute(marginheightAttr, "0");
    body->setAttribute(bgcolorAttr, "rgb(38,38,38)");

    rootElement->appendChild(body, ec);
        
    RefPtr<Element> embedElement = m_doc->createElement(embedTag, false);
        
    m_embedElement = static_cast<HTMLEmbedElement*>(embedElement.get());
    m_embedElement->setAttribute(widthAttr, "100%");
    m_embedElement->setAttribute(heightAttr, "100%");
    
    m_embedElement->setAttribute(nameAttr, "plugin");
    m_embedElement->setAttribute(srcAttr, m_doc->url().string());
    m_embedElement->setAttribute(typeAttr, m_doc->frame()->loader()->responseMIMEType());
    
    body->appendChild(embedElement, ec);    
}
    
bool PluginTokenizer::writeRawData(const char*, int)
{
    ASSERT(!m_embedElement);
    if (m_embedElement)
        return false;
    
    createDocumentStructure();

    if (Frame* frame = m_doc->frame()) {
        Settings* settings = frame->settings();
        if (settings && settings->arePluginsEnabled()) {
            m_doc->updateLayout();

            if (RenderWidget* renderer = toRenderWidget(m_embedElement->renderer())) {
                frame->loader()->client()->redirectDataToPlugin(renderer->widget());
                frame->loader()->activeDocumentLoader()->mainResourceLoader()->setShouldBufferData(false);
            }

            finish();
        }
    }

    return false;
}
    
void PluginTokenizer::stopParsing()
{
    Tokenizer::stopParsing();        
}
    
void PluginTokenizer::finish()
{
    if (!m_parserStopped) 
        m_doc->finishedParsing();            
}
    
bool PluginTokenizer::isWaitingForScripts() const
{
    // A plugin document is never waiting for scripts
    return false;
}
    
PluginDocument::PluginDocument(Frame* frame)
    : HTMLDocument(frame)
{
    setParseMode(Compat);
}
    
Tokenizer* PluginDocument::createTokenizer()
{
    return new PluginTokenizer(this);
}
    
}
