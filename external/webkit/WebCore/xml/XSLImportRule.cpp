

#include "config.h"
#include "XSLImportRule.h"

#if ENABLE(XSLT)

#include "CachedXSLStyleSheet.h"
#include "DocLoader.h"
#include "XSLStyleSheet.h"

namespace WebCore {

XSLImportRule::XSLImportRule(XSLStyleSheet* parent, const String& href)
    : StyleBase(parent)
    , m_strHref(href)
    , m_cachedSheet(0)
    , m_loading(false)
{
}

XSLImportRule::~XSLImportRule()
{
    if (m_styleSheet)
        m_styleSheet->setParent(0);
    
    if (m_cachedSheet)
        m_cachedSheet->removeClient(this);
}

XSLStyleSheet* XSLImportRule::parentStyleSheet() const
{
    return (parent() && parent()->isXSLStyleSheet()) ? static_cast<XSLStyleSheet*>(parent()) : 0;
}

void XSLImportRule::setXSLStyleSheet(const String& href, const KURL& baseURL, const String& sheet)
{
    if (m_styleSheet)
        m_styleSheet->setParent(0);

    m_styleSheet = XSLStyleSheet::create(this, href, baseURL);

    XSLStyleSheet* parent = parentStyleSheet();
    if (parent)
        m_styleSheet->setParentStyleSheet(parent);

    m_styleSheet->parseString(sheet);
    m_loading = false;
    
    if (parent)
        parent->checkLoaded();
}

bool XSLImportRule::isLoading()
{
    return (m_loading || (m_styleSheet && m_styleSheet->isLoading()));
}

void XSLImportRule::loadSheet()
{
    DocLoader* docLoader = 0;
    StyleBase* root = this;
    StyleBase* parent;
    while ((parent = root->parent()))
        root = parent;
    if (root->isXSLStyleSheet())
        docLoader = static_cast<XSLStyleSheet*>(root)->docLoader();
    
    String absHref = m_strHref;
    XSLStyleSheet* parentSheet = parentStyleSheet();
    if (!parentSheet->finalURL().isNull())
        // use parent styleheet's URL as the base URL
        absHref = KURL(parentSheet->finalURL(), m_strHref).string();
    
    // Check for a cycle in our import chain.  If we encounter a stylesheet
    // in our parent chain with the same URL, then just bail.
    for (parent = this->parent(); parent; parent = parent->parent()) {
        if (parent->isXSLStyleSheet() && absHref == static_cast<XSLStyleSheet*>(parent)->finalURL().string())
            return;
    }
    
    m_cachedSheet = docLoader->requestXSLStyleSheet(absHref);
    
    if (m_cachedSheet) {
        m_cachedSheet->addClient(this);
        
        // If the imported sheet is in the cache, then setXSLStyleSheet gets called,
        // and the sheet even gets parsed (via parseString).  In this case we have
        // loaded (even if our subresources haven't), so if we have a stylesheet after
        // checking the cache, then we've clearly loaded.
        if (!m_styleSheet)
            m_loading = true;
    }
}

} // namespace WebCore

#endif // ENABLE(XSLT)
