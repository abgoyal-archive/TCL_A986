

#include "config.h"

#if ENABLE(DATAGRID)

#include "HTMLDataGridElement.h"

#include "DOMDataGridDataSource.h"
#include "HTMLNames.h"
#include "RenderDataGrid.h"
#include "Text.h"

namespace WebCore {

using namespace HTMLNames;

HTMLDataGridElement::HTMLDataGridElement(const QualifiedName& tagName, Document* document)
    : HTMLElement(tagName, document)
    , m_columns(DataGridColumnList::create(this))
{
    setDataSource(DOMDataGridDataSource::create());
}

HTMLDataGridElement::~HTMLDataGridElement()
{
    m_columns->clearDataGrid();
}

bool HTMLDataGridElement::checkDTD(const Node* newChild)
{
    if (newChild->isTextNode())
        return static_cast<const Text*>(newChild)->containsOnlyWhitespace();
    return newChild->hasTagName(dcolTag) || newChild->hasTagName(drowTag);
}

RenderObject* HTMLDataGridElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderDataGrid(this);
}

bool HTMLDataGridElement::autofocus() const
{
    return hasAttribute(autofocusAttr);
}

void HTMLDataGridElement::setAutofocus(bool autofocus)
{
    setAttribute(autofocusAttr, autofocus ? "" : 0);
}

bool HTMLDataGridElement::disabled() const
{
    return hasAttribute(disabledAttr);
}

void HTMLDataGridElement::setDisabled(bool disabled)
{
    setAttribute(disabledAttr, disabled ? "" : 0);
}

bool HTMLDataGridElement::multiple() const
{
    return hasAttribute(multipleAttr);
}

void HTMLDataGridElement::setMultiple(bool multiple)
{
    setAttribute(multipleAttr, multiple ? "" : 0);
}

void HTMLDataGridElement::setDataSource(PassRefPtr<DataGridDataSource> ds)
{
    if (m_dataSource == ds)
        return;
    
    RefPtr<DataGridDataSource> dataSource = ds;
    if (!dataSource)
        dataSource = DOMDataGridDataSource::create();
    m_dataSource = dataSource;
    
    // Always clear our columns when a data source changes.
    // The register callback will rebuild the columns.
    m_columns->clear();
}

DataGridDataSource* HTMLDataGridElement::dataSource() const
{
    ASSERT(m_dataSource);
    return m_dataSource.get();
}

} // namespace WebCore

#endif // ENABLE(DATAGRID)
