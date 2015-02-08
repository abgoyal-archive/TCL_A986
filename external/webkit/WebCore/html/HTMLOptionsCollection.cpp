

#include "config.h"
#include "HTMLOptionsCollection.h"

#include "ExceptionCode.h"
#include "HTMLOptionElement.h"
#include "HTMLSelectElement.h"

namespace WebCore {

HTMLOptionsCollection::HTMLOptionsCollection(PassRefPtr<HTMLSelectElement> select)
    : HTMLCollection(select.get(), SelectOptions, select->collectionInfo())
{
}

PassRefPtr<HTMLOptionsCollection> HTMLOptionsCollection::create(PassRefPtr<HTMLSelectElement> select)
{
    return adoptRef(new HTMLOptionsCollection(select));
}

void HTMLOptionsCollection::add(PassRefPtr<HTMLOptionElement> element, ExceptionCode &ec)
{
    add(element, length(), ec);
}

void HTMLOptionsCollection::add(PassRefPtr<HTMLOptionElement> element, int index, ExceptionCode &ec)
{
    HTMLOptionElement* newOption = element.get();

    if (!newOption) {
        ec = TYPE_MISMATCH_ERR;
        return;
    }

    if (index < -1) {
        ec = INDEX_SIZE_ERR;
        return;
    }

    ec = 0;
    HTMLSelectElement* select = static_cast<HTMLSelectElement*>(base());

    if (index == -1 || unsigned(index) >= length())
        select->add(newOption, 0, ec);
    else
        select->add(newOption, static_cast<HTMLOptionElement*>(item(index)), ec);

    ASSERT(ec == 0);
}

void HTMLOptionsCollection::remove(int index)
{
    static_cast<HTMLSelectElement*>(base())->remove(index);
}

int HTMLOptionsCollection::selectedIndex() const
{
    return static_cast<HTMLSelectElement*>(base())->selectedIndex();
}

void HTMLOptionsCollection::setSelectedIndex(int index)
{
    static_cast<HTMLSelectElement*>(base())->setSelectedIndex(index);
}

void HTMLOptionsCollection::setLength(unsigned length, ExceptionCode& ec)
{
    static_cast<HTMLSelectElement*>(base())->setLength(length, ec);
}

} //namespace
