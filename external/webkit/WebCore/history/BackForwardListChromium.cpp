

#include "config.h"
#include "BackForwardList.h"

#include "HistoryItem.h"
#include "Logging.h"

namespace WebCore {

static const unsigned DefaultCapacity = 100;
static const unsigned NoCurrentItemIndex = UINT_MAX;

BackForwardList::BackForwardList(Page* page)
    : m_page(page)
    , m_client(0)
    , m_capacity(DefaultCapacity)
    , m_closed(true)
    , m_enabled(true)
{
}

BackForwardList::~BackForwardList()
{
    ASSERT(m_closed);
}

void BackForwardList::addItem(PassRefPtr<HistoryItem> prpItem)
{
    ASSERT(prpItem);
    if (m_capacity == 0 || !m_enabled)
        return;
 
    m_client->addItem(prpItem);
}

void BackForwardList::goToItem(HistoryItem* item)
{
    m_client->goToItem(item);
}

HistoryItem* BackForwardList::backItem()
{
    ASSERT_NOT_REACHED();
    return 0;
}

HistoryItem* BackForwardList::forwardItem()
{
    ASSERT_NOT_REACHED();
    return 0;
}

HistoryItem* BackForwardList::currentItem()
{
    return m_client->currentItem();
}

int BackForwardList::capacity()
{
    return m_capacity;
}

void BackForwardList::setCapacity(int size)
{
    m_capacity = size;
}

bool BackForwardList::enabled()
{
    return m_enabled;
}

void BackForwardList::setEnabled(bool enabled)
{
    m_enabled = enabled;
    if (!enabled) {
        int capacity = m_capacity;
        setCapacity(0);
        setCapacity(capacity);
    }
}

int BackForwardList::backListCount()
{
    return m_client->backListCount();
}

int BackForwardList::forwardListCount()
{
    return m_client->forwardListCount();
}

HistoryItem* BackForwardList::itemAtIndex(int index)
{
    return m_client->itemAtIndex(index);
}

void BackForwardList::pushStateItem(PassRefPtr<HistoryItem> newItem)
{
    RefPtr<HistoryItem> current = m_client->currentItem();

    addItem(newItem);

    if (!current->stateObject())
        current->setStateObject(SerializedScriptValue::create());
}

HistoryItemVector& BackForwardList::entries()
{
    static HistoryItemVector noEntries;
    return noEntries;
}

void BackForwardList::close()
{
    if (m_client)
        m_client->close();
    m_page = 0;
    m_closed = true;
}

bool BackForwardList::closed()
{
    return m_closed;
}

} // namespace WebCore
