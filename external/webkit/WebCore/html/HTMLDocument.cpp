

#include "config.h"
#include "HTMLDocument.h"

#include "CSSPropertyNames.h"
#include "CSSStyleSelector.h"
#include "CString.h"
#include "CookieJar.h"
#include "DocumentLoader.h"
#include "DocumentType.h"
#include "ExceptionCode.h"
#include "FocusController.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "FrameTree.h"
#include "FrameView.h"
#include "HTMLBodyElement.h"
#include "HTMLElementFactory.h"
#include "HTMLNames.h"
#include "HTMLTokenizer.h"
#include "InspectorController.h"
#include "KURL.h"
#include "Page.h"

#include "DocTypeStrings.cpp"

namespace WebCore {

using namespace HTMLNames;

HTMLDocument::HTMLDocument(Frame* frame)
    : Document(frame, false, true)
{
    clearXMLVersion();
    setParseMode(Compat);
}

HTMLDocument::~HTMLDocument()
{
}

int HTMLDocument::width()
{
    updateLayoutIgnorePendingStylesheets();
    FrameView* frameView = view();
    return frameView ? frameView->contentsWidth() : 0;
}

int HTMLDocument::height()
{
    updateLayoutIgnorePendingStylesheets();
    FrameView* frameView = view();
    return frameView ? frameView->contentsHeight() : 0;
}

String HTMLDocument::dir()
{
    HTMLElement* b = body();
    if (!b)
        return String();
    return b->getAttribute(dirAttr);
}

void HTMLDocument::setDir(const String& value)
{
    HTMLElement* b = body();
    if (b)
        b->setAttribute(dirAttr, value);
}

String HTMLDocument::designMode() const
{
    return inDesignMode() ? "on" : "off";
}

void HTMLDocument::setDesignMode(const String& value)
{
    InheritedBool mode;
    if (equalIgnoringCase(value, "on"))
        mode = on;
    else if (equalIgnoringCase(value, "off"))
        mode = off;
    else
        mode = inherit;
    Document::setDesignMode(mode);
}

String HTMLDocument::compatMode() const
{
    return inCompatMode() ? "BackCompat" : "CSS1Compat";
}

Element* HTMLDocument::activeElement()
{
    if (Node* node = focusedNode())
        if (node->isElementNode())
            return static_cast<Element*>(node);
    return body();
}

bool HTMLDocument::hasFocus()
{
    Page* page = this->page();
    if (!page)
        return false;
    if (!page->focusController()->isActive())
        return false;
    if (Frame* focusedFrame = page->focusController()->focusedFrame()) {
        if (focusedFrame->tree()->isDescendantOf(frame()))
            return true;
    }
    return false;
}

String HTMLDocument::bgColor()
{
    HTMLElement* b = body();
    HTMLBodyElement* bodyElement = (b && b->hasTagName(bodyTag)) ? static_cast<HTMLBodyElement*>(b) : 0;

    if (!bodyElement)
        return String();
    return bodyElement->bgColor();
}

void HTMLDocument::setBgColor(const String& value)
{
    HTMLElement* b = body();
    HTMLBodyElement* bodyElement = (b && b->hasTagName(bodyTag)) ? static_cast<HTMLBodyElement*>(b) : 0;

    if (bodyElement)
        bodyElement->setBgColor(value);
}

String HTMLDocument::fgColor()
{
    HTMLElement* b = body();
    HTMLBodyElement* bodyElement = (b && b->hasTagName(bodyTag)) ? static_cast<HTMLBodyElement*>(b) : 0;

    if (!bodyElement)
        return String();
    return bodyElement->text();
}

void HTMLDocument::setFgColor(const String& value)
{
    HTMLElement* b = body();
    HTMLBodyElement* bodyElement = (b && b->hasTagName(bodyTag)) ? static_cast<HTMLBodyElement*>(b) : 0;

    if (bodyElement)
        bodyElement->setText(value);
}

String HTMLDocument::alinkColor()
{
    HTMLElement* b = body();
    HTMLBodyElement* bodyElement = (b && b->hasTagName(bodyTag)) ? static_cast<HTMLBodyElement*>(b) : 0;

    if (!bodyElement)
        return String();
    return bodyElement->aLink();
}

void HTMLDocument::setAlinkColor(const String& value)
{
    HTMLElement* b = body();
    HTMLBodyElement* bodyElement = (b && b->hasTagName(bodyTag)) ? static_cast<HTMLBodyElement*>(b) : 0;

    if (bodyElement) {
        // This check is a bit silly, but some benchmarks like to set the
        // document's link colors over and over to the same value and we
        // don't want to incur a style update each time.
        if (bodyElement->aLink() != value)
            bodyElement->setALink(value);
    }
}

String HTMLDocument::linkColor()
{
    HTMLElement* b = body();
    HTMLBodyElement* bodyElement = (b && b->hasTagName(bodyTag)) ? static_cast<HTMLBodyElement*>(b) : 0;

    if (!bodyElement)
        return String();
    return bodyElement->link();
}

void HTMLDocument::setLinkColor(const String& value)
{
    HTMLElement* b = body();
    HTMLBodyElement* bodyElement = (b && b->hasTagName(bodyTag)) ? static_cast<HTMLBodyElement*>(b) : 0;

    if (bodyElement) {
        // This check is a bit silly, but some benchmarks like to set the
        // document's link colors over and over to the same value and we
        // don't want to incur a style update each time.
        if (bodyElement->link() != value)
            bodyElement->setLink(value);
    }
}

String HTMLDocument::vlinkColor()
{
    HTMLElement* b = body();
    HTMLBodyElement* bodyElement = (b && b->hasTagName(bodyTag)) ? static_cast<HTMLBodyElement*>(b) : 0;

    if (!bodyElement)
        return String();
    return bodyElement->vLink();
}

void HTMLDocument::setVlinkColor(const String& value)
{
    HTMLElement* b = body();
    HTMLBodyElement* bodyElement = (b && b->hasTagName(bodyTag)) ? static_cast<HTMLBodyElement*>(b) : 0;

    if (bodyElement) {
        // This check is a bit silly, but some benchmarks like to set the
        // document's link colors over and over to the same value and we
        // don't want to incur a style update each time.
        if (bodyElement->vLink() != value)
            bodyElement->setVLink(value);
    }
}

void HTMLDocument::captureEvents()
{
}

void HTMLDocument::releaseEvents()
{
}

Tokenizer *HTMLDocument::createTokenizer()
{
    bool reportErrors = false;
#if ENABLE(INSPECTOR)
    if (Page* page = this->page())
        reportErrors = page->inspectorController()->windowVisible();
#endif

    return new HTMLTokenizer(this, reportErrors);
}

// --------------------------------------------------------------------------
// not part of the DOM
// --------------------------------------------------------------------------

bool HTMLDocument::childAllowed(Node *newChild)
{
    return newChild->hasTagName(htmlTag) || newChild->isCommentNode() || (newChild->nodeType() == DOCUMENT_TYPE_NODE && !doctype());
}

PassRefPtr<Element> HTMLDocument::createElement(const AtomicString& name, ExceptionCode& ec)
{
    if (!isValidName(name)) {
        ec = INVALID_CHARACTER_ERR;
        return 0;
    }
    return HTMLElementFactory::createHTMLElement(QualifiedName(nullAtom, name.lower(), xhtmlNamespaceURI), this, 0, false);
}

static void addItemToMap(HashCountedSet<AtomicStringImpl*>& map, const AtomicString& name)
{
    if (name.isEmpty())
        return;
    map.add(name.impl());
}

static void removeItemFromMap(HashCountedSet<AtomicStringImpl*>& map, const AtomicString& name)
{
    if (name.isEmpty())
        return;
    map.remove(name.impl());
}

void HTMLDocument::addNamedItem(const AtomicString& name)
{
    addItemToMap(m_namedItemCounts, name);
}

void HTMLDocument::removeNamedItem(const AtomicString& name)
{ 
    removeItemFromMap(m_namedItemCounts, name);
}

void HTMLDocument::addExtraNamedItem(const AtomicString& name)
{
    addItemToMap(m_extraNamedItemCounts, name);
}

void HTMLDocument::removeExtraNamedItem(const AtomicString& name)
{ 
    removeItemFromMap(m_extraNamedItemCounts, name);
}

void HTMLDocument::determineParseMode()
{
    // FIXME: It's terrible that this code runs separately and isn't just built in to the
    // HTML tokenizer/parser.

    // This code more or less mimics Mozilla's implementation (specifically the
    // doctype parsing implemented by David Baron in Mozilla's nsParser.cpp).
    //
    // There are three possible parse modes:
    // COMPAT - quirks mode emulates WinIE and NS4.  CSS parsing is also relaxed in this mode, e.g., unit types can
    // be omitted from numbers.
    // ALMOST STRICT - This mode is identical to strict mode except for its treatment of line-height in the inline box model.  For
    // now (until the inline box model is re-written), this mode is identical to STANDARDS mode.
    // STRICT - no quirks apply.  Web pages will obey the specifications to the letter.
    bool wasInCompatMode = inCompatMode();
    DocumentType* docType = doctype();
    if (!docType || !equalIgnoringCase(docType->name(), "html"))
        // No doctype found at all or the doctype is not HTML.  Default to quirks mode and Html4.
        setParseMode(Compat);
    else if (!doctype()->systemId().isEmpty() && equalIgnoringCase(docType->systemId(), "http://www.ibm.com/data/dtd/v11/ibmxhtml1-transitional.dtd"))
        // Assume quirks mode for this particular system ID.  In the HTML5 spec, this is the only
        // system identifier that is examined.
        setParseMode(Compat);
    else if (docType->publicId().isEmpty())
        // A doctype without a public ID means use strict mode.
        setParseMode(Strict);
    else {
        // We have to check a list of public IDs to see what we
        // should do.
        String lowerPubID = docType->publicId().lower();
        CString pubIDStr = lowerPubID.latin1();
       
        // Look up the entry in our gperf-generated table.
        const PubIDInfo* doctypeEntry = findDoctypeEntry(pubIDStr.data(), pubIDStr.length());
        if (!doctypeEntry)
            // The DOCTYPE is not in the list.  Assume strict mode.
            setParseMode(Strict);
        else {
            switch (docType->systemId().isEmpty() ?
                    doctypeEntry->mode_if_no_sysid :
                    doctypeEntry->mode_if_sysid) {
                case PubIDInfo::eQuirks3:
                case PubIDInfo::eQuirks:
                    setParseMode(Compat);
                    break;
                case PubIDInfo::eAlmostStandards:
                    setParseMode(AlmostStrict);
                    break;
                 default:
                    ASSERT(false);
            }
        }
    }
    
    if (inCompatMode() != wasInCompatMode) {
        clearPageUserSheet();
        clearPageGroupUserSheets();
        updateStyleSelector();
    }
}

void HTMLDocument::clear()
{
    // FIXME: This does nothing, and that seems unlikely to be correct.
    // We've long had a comment saying that IE doesn't support this.
    // But I do see it in the documentation for Mozilla.
}

bool HTMLDocument::isFrameSet() const
{
    HTMLElement* bodyElement = body();
    return bodyElement && bodyElement->renderer() && bodyElement->hasTagName(framesetTag);
}

#ifdef ANDROID_INSTRUMENT
void* HTMLDocument::operator new(size_t size)
{
    return Node::operator new(size);
}

void* HTMLDocument::operator new[](size_t size)
{
    return Node::operator new[](size);
}

void HTMLDocument::operator delete(void* p, size_t size)
{
    Node::operator delete(p, size);
}

void HTMLDocument::operator delete[](void* p, size_t size)
{
    Node::operator delete[](p, size);
}
#endif

}
