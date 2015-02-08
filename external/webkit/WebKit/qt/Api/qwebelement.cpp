

#include "config.h"
#include "qwebelement.h"

#include "CSSComputedStyleDeclaration.h"
#include "CSSMutableStyleDeclaration.h"
#include "CSSParser.h"
#include "CSSRule.h"
#include "CSSRuleList.h"
#include "CSSStyleRule.h"
#include "CString.h"
#include "Document.h"
#include "DocumentFragment.h"
#include "FrameView.h"
#include "GraphicsContext.h"
#include "HTMLElement.h"
#include "JSGlobalObject.h"
#include "JSHTMLElement.h"
#include "JSObject.h"
#include "NodeList.h"
#include "PropertyNameArray.h"
#include "RenderImage.h"
#include "StaticNodeList.h"
#include "qt_runtime.h"
#include "qwebframe.h"
#include "qwebframe_p.h"
#include "runtime_root.h"
#include <parser/SourceCode.h>
#include <wtf/Vector.h>

#include <QPainter>

using namespace WebCore;

class QWebElementPrivate {
public:
};


QWebElement::QWebElement()
    : d(0)
    , m_element(0)
{
}

QWebElement::QWebElement(WebCore::Element* domElement)
    : d(0)
    , m_element(domElement)
{
    if (m_element)
        m_element->ref();
}

QWebElement::QWebElement(WebCore::Node* node)
    : d(0)
    , m_element(0)
{
    if (node && node->isHTMLElement()) {
        m_element = static_cast<HTMLElement*>(node);
        m_element->ref();
    }
}

QWebElement::QWebElement(const QWebElement &other)
    : d(0)
    , m_element(other.m_element)
{
    if (m_element)
        m_element->ref();
}

QWebElement &QWebElement::operator=(const QWebElement &other)
{
    // ### handle "d" assignment
    if (this != &other) {
        Element *otherElement = other.m_element;
        if (otherElement)
            otherElement->ref();
        if (m_element)
            m_element->deref();
        m_element = otherElement;
    }
    return *this;
}

QWebElement::~QWebElement()
{
    delete d;
    if (m_element)
        m_element->deref();
}

bool QWebElement::operator==(const QWebElement& o) const
{
    return m_element == o.m_element;
}

bool QWebElement::operator!=(const QWebElement& o) const
{
    return m_element != o.m_element;
}

bool QWebElement::isNull() const
{
    return !m_element;
}

QWebElementCollection QWebElement::findAll(const QString &selectorQuery) const
{
    return QWebElementCollection(*this, selectorQuery);
}

QWebElement QWebElement::findFirst(const QString &selectorQuery) const
{
    if (!m_element)
        return QWebElement();
    ExceptionCode exception = 0; // ###
    return QWebElement(m_element->querySelector(selectorQuery, exception).get());
}

void QWebElement::setPlainText(const QString &text)
{
    if (!m_element || !m_element->isHTMLElement())
        return;
    ExceptionCode exception = 0;
    static_cast<HTMLElement*>(m_element)->setInnerText(text, exception);
}

QString QWebElement::toPlainText() const
{
    if (!m_element || !m_element->isHTMLElement())
        return QString();
    return static_cast<HTMLElement*>(m_element)->innerText();
}

void QWebElement::setOuterXml(const QString &markup)
{
    if (!m_element || !m_element->isHTMLElement())
        return;

    ExceptionCode exception = 0;

    static_cast<HTMLElement*>(m_element)->setOuterHTML(markup, exception);
}

QString QWebElement::toOuterXml() const
{
    if (!m_element || !m_element->isHTMLElement())
        return QString();

    return static_cast<HTMLElement*>(m_element)->outerHTML();
}

void QWebElement::setInnerXml(const QString &markup)
{
    if (!m_element || !m_element->isHTMLElement())
        return;

    ExceptionCode exception = 0;

    static_cast<HTMLElement*>(m_element)->setInnerHTML(markup, exception);
}

QString QWebElement::toInnerXml() const
{
    if (!m_element || !m_element->isHTMLElement())
        return QString();

    return static_cast<HTMLElement*>(m_element)->innerHTML();
}

void QWebElement::setAttribute(const QString &name, const QString &value)
{
    if (!m_element)
        return;
    ExceptionCode exception = 0;
    m_element->setAttribute(name, value, exception);
}

void QWebElement::setAttributeNS(const QString &namespaceUri, const QString &name, const QString &value)
{
    if (!m_element)
        return;
    WebCore::ExceptionCode exception = 0;
    m_element->setAttributeNS(namespaceUri, name, value, exception);
}

QString QWebElement::attribute(const QString &name, const QString &defaultValue) const
{
    if (!m_element)
        return QString();
    if (m_element->hasAttribute(name))
        return m_element->getAttribute(name);
    else
        return defaultValue;
}

QString QWebElement::attributeNS(const QString &namespaceUri, const QString &name, const QString &defaultValue) const
{
    if (!m_element)
        return QString();
    if (m_element->hasAttributeNS(namespaceUri, name))
        return m_element->getAttributeNS(namespaceUri, name);
    else
        return defaultValue;
}

bool QWebElement::hasAttribute(const QString &name) const
{
    if (!m_element)
        return false;
    return m_element->hasAttribute(name);
}

bool QWebElement::hasAttributeNS(const QString &namespaceUri, const QString &name) const
{
    if (!m_element)
        return false;
    return m_element->hasAttributeNS(namespaceUri, name);
}

void QWebElement::removeAttribute(const QString &name)
{
    if (!m_element)
        return;
    ExceptionCode exception = 0;
    m_element->removeAttribute(name, exception);
}

void QWebElement::removeAttributeNS(const QString &namespaceUri, const QString &name)
{
    if (!m_element)
        return;
    WebCore::ExceptionCode exception = 0;
    m_element->removeAttributeNS(namespaceUri, name, exception);
}

bool QWebElement::hasAttributes() const
{
    if (!m_element)
        return false;
    return m_element->hasAttributes();
}

QStringList QWebElement::attributeNames(const QString& namespaceUri) const
{
    if (!m_element)
        return QStringList();

    QStringList attributeNameList;
    const NamedNodeMap* const attrs = m_element->attributes(/* read only = */ true);
    if (attrs) {
        const String namespaceUriString(namespaceUri); // convert QString -> String once
        const unsigned attrsCount = attrs->length();
        for (unsigned i = 0; i < attrsCount; ++i) {
            const Attribute* const attribute = attrs->attributeItem(i);
            if (namespaceUriString == attribute->namespaceURI())
                attributeNameList.append(attribute->localName());
        }
    }
    return attributeNameList;
}

bool QWebElement::hasFocus() const
{
    if (!m_element)
        return false;
    if (m_element->document())
        return m_element == m_element->document()->focusedNode();
    return false;
}

void QWebElement::setFocus()
{
    if (!m_element)
        return;
    if (m_element->document() && m_element->isFocusable())
        m_element->document()->setFocusedNode(m_element);
}

QRect QWebElement::geometry() const
{
    if (!m_element)
        return QRect();
    return m_element->getRect();
}

QString QWebElement::tagName() const
{
    if (!m_element)
        return QString();
    return m_element->tagName();
}

QString QWebElement::prefix() const
{
    if (!m_element)
        return QString();
    return m_element->prefix();
}

QString QWebElement::localName() const
{
    if (!m_element)
        return QString();
    return m_element->localName();
}

QString QWebElement::namespaceUri() const
{
    if (!m_element)
        return QString();
    return m_element->namespaceURI();
}

QWebElement QWebElement::parent() const
{
    if (m_element)
        return QWebElement(m_element->parentElement());
    return QWebElement();
}

QWebElement QWebElement::firstChild() const
{
    if (!m_element)
        return QWebElement();
    for (Node* child = m_element->firstChild(); child; child = child->nextSibling()) {
        if (!child->isElementNode())
            continue;
        Element* e = static_cast<Element*>(child);
        return QWebElement(e);
    }
    return QWebElement();
}

QWebElement QWebElement::lastChild() const
{
    if (!m_element)
        return QWebElement();
    for (Node* child = m_element->lastChild(); child; child = child->previousSibling()) {
        if (!child->isElementNode())
            continue;
        Element* e = static_cast<Element*>(child);
        return QWebElement(e);
    }
    return QWebElement();
}

QWebElement QWebElement::nextSibling() const
{
    if (!m_element)
        return QWebElement();
    for (Node* sib = m_element->nextSibling(); sib; sib = sib->nextSibling()) {
        if (!sib->isElementNode())
            continue;
        Element* e = static_cast<Element*>(sib);
        return QWebElement(e);
    }
    return QWebElement();
}

QWebElement QWebElement::previousSibling() const
{
    if (!m_element)
        return QWebElement();
    for (Node* sib = m_element->previousSibling(); sib; sib = sib->previousSibling()) {
        if (!sib->isElementNode())
            continue;
        Element* e = static_cast<Element*>(sib);
        return QWebElement(e);
    }
    return QWebElement();
}

QWebElement QWebElement::document() const
{
    if (!m_element)
        return QWebElement();
    Document* document = m_element->document();
    if (!document)
        return QWebElement();
    return QWebElement(document->documentElement());
}

QWebFrame *QWebElement::webFrame() const
{
    if (!m_element)
        return 0;

    Document* document = m_element->document();
    if (!document)
        return 0;

    Frame* frame = document->frame();
    if (!frame)
        return 0;
    return QWebFramePrivate::kit(frame);
}

static bool setupScriptContext(WebCore::Element* element, JSC::JSValue& thisValue, ScriptState*& state, ScriptController*& scriptController)
{
    if (!element)
        return false;

    Document* document = element->document();
    if (!document)
        return false;

    Frame* frame = document->frame();
    if (!frame)
        return false;

    scriptController = frame->script();
    if (!scriptController)
        return false;

    state = scriptController->globalObject(mainThreadNormalWorld())->globalExec();
    if (!state)
        return false;

    thisValue = toJS(state, element);
    if (!thisValue)
        return false;

    return true;
}


QVariant QWebElement::evaluateJavaScript(const QString& scriptSource)
{
    if (scriptSource.isEmpty())
        return QVariant();

    ScriptState* state = 0;
    JSC::JSValue thisValue;
    ScriptController* scriptController = 0;

    if (!setupScriptContext(m_element, thisValue, state, scriptController))
        return QVariant();

    JSC::ScopeChain& scopeChain = state->dynamicGlobalObject()->globalScopeChain();
    JSC::UString script((const UChar*)scriptSource.data(), scriptSource.length());
    JSC::Completion completion = JSC::evaluate(state, scopeChain, JSC::makeSource(script), thisValue);
    if ((completion.complType() != JSC::ReturnValue) && (completion.complType() != JSC::Normal))
        return QVariant();

    JSC::JSValue result = completion.value();
    if (!result)
        return QVariant();

    int distance = 0;
    return JSC::Bindings::convertValueToQVariant(state, result, QMetaType::Void, &distance);
}



QString QWebElement::styleProperty(const QString &name, StyleResolveStrategy strategy) const
{
    if (!m_element || !m_element->isStyledElement())
        return QString();

    int propID = cssPropertyID(name);

    if (!propID)
        return QString();

    CSSStyleDeclaration* style = static_cast<StyledElement*>(m_element)->style();

    if (strategy == InlineStyle)
        return style->getPropertyValue(propID);

    if (strategy == CascadedStyle) {
        if (style->getPropertyPriority(propID))
            return style->getPropertyValue(propID);

        // We are going to resolve the style property by walking through the
        // list of non-inline matched CSS rules for the element, looking for
        // the highest priority definition.

        // Get an array of matched CSS rules for the given element sorted
        // by importance and inheritance order. This include external CSS
        // declarations, as well as embedded and inline style declarations.

        DOMWindow* domWindow = m_element->document()->frame()->domWindow();
        if (RefPtr<CSSRuleList> rules = domWindow->getMatchedCSSRules(m_element, "")) {
            for (int i = rules->length(); i > 0; --i) {
                CSSStyleRule* rule = static_cast<CSSStyleRule*>(rules->item(i - 1));

                if (rule->style()->getPropertyPriority(propID))
                    return rule->style()->getPropertyValue(propID);

                if (style->getPropertyValue(propID).isEmpty())
                    style = rule->style();
            }
        }

        return style->getPropertyValue(propID);
    }

    if (strategy == ComputedStyle) {
        if (!m_element || !m_element->isStyledElement())
            return QString();

        int propID = cssPropertyID(name);

        RefPtr<CSSComputedStyleDeclaration> style = computedStyle(m_element);
        if (!propID || !style)
            return QString();

        return style->getPropertyValue(propID);
    }

    return QString();
}

void QWebElement::setStyleProperty(const QString &name, const QString &value)
{
    if (!m_element || !m_element->isStyledElement())
        return;

    int propID = cssPropertyID(name);
    CSSStyleDeclaration* style = static_cast<StyledElement*>(m_element)->style();
    if (!propID || !style)
        return;

    ExceptionCode exception = 0;
    style->setProperty(name, value, exception);
}

QStringList QWebElement::classes() const
{
    if (!hasAttribute(QLatin1String("class")))
        return QStringList();

    QStringList classes =  attribute(QLatin1String("class")).simplified().split(QLatin1Char(' '), QString::SkipEmptyParts);
#if QT_VERSION >= 0x040500
    classes.removeDuplicates();
#else
    int n = classes.size();
    int j = 0;
    QSet<QString> seen;
    seen.reserve(n);
    for (int i = 0; i < n; ++i) {
        const QString& s = classes.at(i);
        if (seen.contains(s))
            continue;
        seen.insert(s);
        if (j != i)
            classes[j] = s;
        ++j;
    }
    if (n != j)
        classes.erase(classes.begin() + j, classes.end());
#endif
    return classes;
}

bool QWebElement::hasClass(const QString &name) const
{
    QStringList list = classes();
    return list.contains(name);
}

void QWebElement::addClass(const QString &name)
{
    QStringList list = classes();
    if (!list.contains(name)) {
        list.append(name);
        QString value = list.join(QLatin1String(" "));
        setAttribute(QLatin1String("class"), value);
    }
}

void QWebElement::removeClass(const QString &name)
{
    QStringList list = classes();
    if (list.contains(name)) {
        list.removeAll(name);
        QString value = list.join(QLatin1String(" "));
        setAttribute(QLatin1String("class"), value);
    }
}

void QWebElement::toggleClass(const QString &name)
{
    QStringList list = classes();
    if (list.contains(name))
        list.removeAll(name);
    else
        list.append(name);

    QString value = list.join(QLatin1String(" "));
    setAttribute(QLatin1String("class"), value);
}

void QWebElement::appendInside(const QWebElement &element)
{
    if (!m_element || element.isNull())
        return;

    ExceptionCode exception = 0;
    m_element->appendChild(element.m_element, exception);
}

void QWebElement::appendInside(const QString &markup)
{
    if (!m_element)
        return;

    if (!m_element->isHTMLElement())
        return;

    HTMLElement* htmlElement = static_cast<HTMLElement*>(m_element);
    RefPtr<DocumentFragment> fragment = htmlElement->createContextualFragment(markup);

    ExceptionCode exception = 0;
    m_element->appendChild(fragment, exception);
}

void QWebElement::prependInside(const QWebElement &element)
{
    if (!m_element || element.isNull())
        return;

    ExceptionCode exception = 0;

    if (m_element->hasChildNodes())
        m_element->insertBefore(element.m_element, m_element->firstChild(), exception);
    else
        m_element->appendChild(element.m_element, exception);
}

void QWebElement::prependInside(const QString &markup)
{
    if (!m_element)
        return;

    if (!m_element->isHTMLElement())
        return;

    HTMLElement* htmlElement = static_cast<HTMLElement*>(m_element);
    RefPtr<DocumentFragment> fragment = htmlElement->createContextualFragment(markup);

    ExceptionCode exception = 0;

    if (m_element->hasChildNodes())
        m_element->insertBefore(fragment, m_element->firstChild(), exception);
    else
        m_element->appendChild(fragment, exception);
}


void QWebElement::prependOutside(const QWebElement &element)
{
    if (!m_element || element.isNull())
        return;

    if (!m_element->parent())
        return;

    ExceptionCode exception = 0;
    m_element->parent()->insertBefore(element.m_element, m_element, exception);
}

void QWebElement::prependOutside(const QString &markup)
{
    if (!m_element)
        return;

    if (!m_element->parent())
        return;

    if (!m_element->isHTMLElement())
        return;

    HTMLElement* htmlElement = static_cast<HTMLElement*>(m_element);
    RefPtr<DocumentFragment> fragment = htmlElement->createContextualFragment(markup);

    ExceptionCode exception = 0;
    m_element->parent()->insertBefore(fragment, m_element, exception);
}

void QWebElement::appendOutside(const QWebElement &element)
{
    if (!m_element || element.isNull())
        return;

    if (!m_element->parent())
        return;

    ExceptionCode exception = 0;
    if (!m_element->nextSibling())
        m_element->parent()->appendChild(element.m_element, exception);
    else
        m_element->parent()->insertBefore(element.m_element, m_element->nextSibling(), exception);
}

void QWebElement::appendOutside(const QString &markup)
{
    if (!m_element)
        return;

    if (!m_element->parent())
        return;

    if (!m_element->isHTMLElement())
        return;

    HTMLElement* htmlElement = static_cast<HTMLElement*>(m_element);
    RefPtr<DocumentFragment> fragment = htmlElement->createContextualFragment(markup);

    ExceptionCode exception = 0;
    if (!m_element->nextSibling())
        m_element->parent()->appendChild(fragment, exception);
    else
        m_element->parent()->insertBefore(fragment, m_element->nextSibling(), exception);
}

QWebElement QWebElement::clone() const
{
    if (!m_element)
        return QWebElement();

    return QWebElement(m_element->cloneElementWithChildren().get());
}

QWebElement &QWebElement::takeFromDocument()
{
    if (!m_element)
        return *this;

    ExceptionCode exception = 0;
    m_element->remove(exception);

    return *this;
}

void QWebElement::removeFromDocument()
{
    if (!m_element)
        return;

    ExceptionCode exception = 0;
    m_element->remove(exception);
    m_element->deref();
    m_element = 0;
}

void QWebElement::removeAllChildren()
{
    if (!m_element)
        return;

    m_element->removeAllChildren();
}

static RefPtr<Node> findInsertionPoint(PassRefPtr<Node> root)
{
    RefPtr<Node> node = root;

    // Go as far down the tree as possible.
    while (node->hasChildNodes() && node->firstChild()->isElementNode())
        node = node->firstChild();

    // TODO: Implement SVG support
    if (node->isHTMLElement()) {
        HTMLElement* element = static_cast<HTMLElement*>(node.get());

        // The insert point could be a non-enclosable tag and it can thus
        // never have children, so go one up. Get the parent element, and not
        // note as a root note will always exist.
        if (element->endTagRequirement() == TagStatusForbidden)
            node = node->parentElement();
    }

    return node;
}

void QWebElement::encloseContentsWith(const QWebElement &element)
{
    if (!m_element || element.isNull())
        return;

    RefPtr<Node> insertionPoint = findInsertionPoint(element.m_element);

    if (!insertionPoint)
        return;

    ExceptionCode exception = 0;

    // reparent children
    for (RefPtr<Node> child = m_element->firstChild(); child;) {
        RefPtr<Node> next = child->nextSibling();
        insertionPoint->appendChild(child, exception);
        child = next;
    }

    if (m_element->hasChildNodes())
        m_element->insertBefore(element.m_element, m_element->firstChild(), exception);
    else
        m_element->appendChild(element.m_element, exception);
}

void QWebElement::encloseContentsWith(const QString &markup)
{
    if (!m_element)
        return;

    if (!m_element->parent())
        return;

    if (!m_element->isHTMLElement())
        return;

    HTMLElement* htmlElement = static_cast<HTMLElement*>(m_element);
    RefPtr<DocumentFragment> fragment = htmlElement->createContextualFragment(markup);

    if (!fragment || !fragment->firstChild())
        return;

    RefPtr<Node> insertionPoint = findInsertionPoint(fragment->firstChild());

    if (!insertionPoint)
        return;

    ExceptionCode exception = 0;

    // reparent children
    for (RefPtr<Node> child = m_element->firstChild(); child;) {
        RefPtr<Node> next = child->nextSibling();
        insertionPoint->appendChild(child, exception);
        child = next;
    }

    if (m_element->hasChildNodes())
        m_element->insertBefore(fragment, m_element->firstChild(), exception);
    else
        m_element->appendChild(fragment, exception);
}

void QWebElement::encloseWith(const QWebElement &element)
{
    if (!m_element || element.isNull())
        return;

    RefPtr<Node> insertionPoint = findInsertionPoint(element.m_element);

    if (!insertionPoint)
        return;

    // Keep reference to these two nodes before pulling out this element and
    // wrapping it in the fragment. The reason for doing it in this order is
    // that once the fragment has been added to the document it is empty, so
    // we no longer have access to the nodes it contained.
    Node* parentNode = m_element->parent();
    Node* siblingNode = m_element->nextSibling();

    ExceptionCode exception = 0;
    insertionPoint->appendChild(m_element, exception);

    if (!siblingNode)
        parentNode->appendChild(element.m_element, exception);
    else
        parentNode->insertBefore(element.m_element, siblingNode, exception);
}

void QWebElement::encloseWith(const QString &markup)
{
    if (!m_element)
        return;

    if (!m_element->parent())
        return;

    if (!m_element->isHTMLElement())
        return;

    HTMLElement* htmlElement = static_cast<HTMLElement*>(m_element);
    RefPtr<DocumentFragment> fragment = htmlElement->createContextualFragment(markup);

    if (!fragment || !fragment->firstChild())
        return;

    RefPtr<Node> insertionPoint = findInsertionPoint(fragment->firstChild());

    if (!insertionPoint)
        return;

    // Keep reference to these two nodes before pulling out this element and
    // wrapping it in the fragment. The reason for doing it in this order is
    // that once the fragment has been added to the document it is empty, so
    // we no longer have access to the nodes it contained.
    Node* parentNode = m_element->parent();
    Node* siblingNode = m_element->nextSibling();

    ExceptionCode exception = 0;
    insertionPoint->appendChild(m_element, exception);

    if (!siblingNode)
        parentNode->appendChild(fragment, exception);
    else
        parentNode->insertBefore(fragment, siblingNode, exception);
}

void QWebElement::replace(const QWebElement &element)
{
    if (!m_element || element.isNull())
        return;

    appendOutside(element);
    takeFromDocument();
}

void QWebElement::replace(const QString &markup)
{
    if (!m_element)
        return;

    appendOutside(markup);
    takeFromDocument();
}

QWebElement QWebElement::enclosingElement(WebCore::Node* node)
{
    QWebElement element(node);

    while (element.isNull() && node) {
        node = node->parentNode();
        element = QWebElement(node);
    }
    return element;
}




void QWebElement::render(QPainter* painter)
{
    WebCore::Element* e = m_element;
    Document* doc = e ? e->document() : 0;
    if (!doc)
        return;

    Frame* frame = doc->frame();
    if (!frame || !frame->view() || !frame->contentRenderer())
        return;

    FrameView* view = frame->view();

    view->layoutIfNeededRecursive();

    IntRect rect = e->getRect();

    if (rect.size().isEmpty())
        return;

    GraphicsContext context(painter);

    context.save();
    context.translate(-rect.x(), -rect.y());
    view->setNodeToDraw(e);
    view->paintContents(&context, rect);
    view->setNodeToDraw(0);
    context.restore();
}

class QWebElementCollectionPrivate : public QSharedData
{
public:
    static QWebElementCollectionPrivate* create(const PassRefPtr<Node> &context, const QString &query);

    RefPtr<NodeList> m_result;

private:
    inline QWebElementCollectionPrivate() {}
};

QWebElementCollectionPrivate* QWebElementCollectionPrivate::create(const PassRefPtr<Node> &context, const QString &query)
{
    if (!context)
        return 0;

    // Let WebKit do the hard work hehehe
    ExceptionCode exception = 0; // ###
    RefPtr<NodeList> nodes = context->querySelectorAll(query, exception);
    if (!nodes)
        return 0;

    QWebElementCollectionPrivate* priv = new QWebElementCollectionPrivate;
    priv->m_result = nodes;
    return priv;
}


QWebElementCollection::QWebElementCollection()
{
}

QWebElementCollection::QWebElementCollection(const QWebElementCollection &other)
    : d(other.d)
{
}

QWebElementCollection::QWebElementCollection(const QWebElement &contextElement, const QString &query)
{
    d = QExplicitlySharedDataPointer<QWebElementCollectionPrivate>(QWebElementCollectionPrivate::create(contextElement.m_element, query));
}

QWebElementCollection &QWebElementCollection::operator=(const QWebElementCollection &other)
{
    d = other.d;
    return *this;
}

QWebElementCollection::~QWebElementCollection()
{
}


QWebElementCollection QWebElementCollection::operator+(const QWebElementCollection &other) const
{
    QWebElementCollection n = *this; n.d.detach(); n += other; return n;
}

void QWebElementCollection::append(const QWebElementCollection &other)
{
    if (!d) {
        *this = other;
        return;
    }
    if (!other.d)
        return;
    Vector<RefPtr<Node> > nodes;
    RefPtr<NodeList> results[] = { d->m_result, other.d->m_result };
    nodes.reserveInitialCapacity(results[0]->length() + results[1]->length());

    for (int i = 0; i < 2; ++i) {
        int j = 0;
        Node* n = results[i]->item(j);
        while (n) {
            nodes.append(n);
            n = results[i]->item(++j);
        }
    }

    d->m_result = StaticNodeList::adopt(nodes);
}

int QWebElementCollection::count() const
{
    if (!d)
        return 0;
    return d->m_result->length();
}

QWebElement QWebElementCollection::at(int i) const
{
    if (!d)
        return QWebElement();
    Node* n = d->m_result->item(i);
    return QWebElement(static_cast<Element*>(n));
}




QList<QWebElement> QWebElementCollection::toList() const
{
    if (!d)
        return QList<QWebElement>();
    QList<QWebElement> elements;
    int i = 0;
    Node* n = d->m_result->item(i);
    while (n) {
        if (n->isElementNode())
            elements.append(QWebElement(static_cast<Element*>(n)));
        n = d->m_result->item(++i);
    }
    return elements;
}












































