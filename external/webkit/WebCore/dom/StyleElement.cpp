

#include "config.h"
#include "StyleElement.h"

#include "Document.h"
#include "Element.h"
#include "MappedAttribute.h"
#include "MediaList.h"
#include "MediaQueryEvaluator.h"

namespace WebCore {

StyleElement::StyleElement()
{
}

StyleSheet* StyleElement::sheet(Element* e)
{
    if (!m_sheet)
        createSheet(e);
    return m_sheet.get();
}

void StyleElement::insertedIntoDocument(Document*, Element* element)
{
    process(element);
}

void StyleElement::removedFromDocument(Document* document)
{
    // If we're in document teardown, then we don't need to do any notification of our sheet's removal.
    if (!document->renderer())
        return;

    // FIXME: It's terrible to do a synchronous update of the style selector just because a <style> or <link> element got removed.
    if (m_sheet)
        document->updateStyleSelector();
}

void StyleElement::process(Element* e)
{
    if (!e || !e->inDocument())
        return;

    unsigned resultLength = 0;
    for (Node* c = e->firstChild(); c; c = c->nextSibling()) {
        Node::NodeType nodeType = c->nodeType();
        if (nodeType == Node::TEXT_NODE || nodeType == Node::CDATA_SECTION_NODE || nodeType == Node::COMMENT_NODE) {
            unsigned length = c->nodeValue().length();
            if (length > std::numeric_limits<unsigned>::max() - resultLength)
                CRASH();
            resultLength += length;
        }
    }
    UChar* text;
    String sheetText = String::createUninitialized(resultLength, text);

    UChar* p = text;
    for (Node* c = e->firstChild(); c; c = c->nextSibling()) {
        Node::NodeType nodeType = c->nodeType();
        if (nodeType == Node::TEXT_NODE || nodeType == Node::CDATA_SECTION_NODE || nodeType == Node::COMMENT_NODE) {
            String nodeValue = c->nodeValue();
            unsigned nodeLength = nodeValue.length();
            memcpy(p, nodeValue.characters(), nodeLength * sizeof(UChar));
            p += nodeLength;
        }
    }
    ASSERT(p == text + resultLength);

    createSheet(e, sheetText);
}

void StyleElement::createSheet(Element* e, const String& text)
{
    Document* document = e->document();
    if (m_sheet) {
        if (static_cast<CSSStyleSheet*>(m_sheet.get())->isLoading())
            document->removePendingSheet();
        m_sheet = 0;
    }

    // If type is empty or CSS, this is a CSS style sheet.
    const AtomicString& type = this->type();
    if (type.isEmpty() || (e->isHTMLElement() ? equalIgnoringCase(type, "text/css") : (type == "text/css"))) {
        RefPtr<MediaList> mediaList = MediaList::create(media(), e->isHTMLElement());
        MediaQueryEvaluator screenEval("screen", true);
        MediaQueryEvaluator printEval("print", true);
        if (screenEval.eval(mediaList.get()) || printEval.eval(mediaList.get())) {
            document->addPendingSheet();
            setLoading(true);
            m_sheet = CSSStyleSheet::create(e, String(), KURL(), document->inputEncoding());
            m_sheet->parseString(text, !document->inCompatMode());
            m_sheet->setMedia(mediaList.get());
            m_sheet->setTitle(e->title());
            setLoading(false);
        }
    }

    if (m_sheet)
        m_sheet->checkLoaded();
}

}
