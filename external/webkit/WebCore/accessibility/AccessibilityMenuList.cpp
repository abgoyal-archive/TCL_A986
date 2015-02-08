

#include "config.h"
#include "AccessibilityMenuList.h"

#include "AXObjectCache.h"
#include "AccessibilityMenuListPopup.h"
#include "RenderMenuList.h"

namespace WebCore {

AccessibilityMenuList::AccessibilityMenuList(RenderObject* renderer)
    : AccessibilityRenderObject(renderer)
{
    ASSERT_ARG(renderer, renderer->isMenuList());
}

bool AccessibilityMenuList::press() const
{
    RenderMenuList* menuList = static_cast<RenderMenuList*>(m_renderer);
    if (menuList->popupIsVisible())
        menuList->hidePopup();
    else
        menuList->showPopup();
    return true;
}

void AccessibilityMenuList::addChildren()
{
    m_haveChildren = true;

    AXObjectCache* cache = m_renderer->document()->axObjectCache();

    AccessibilityObject* list = cache->getOrCreate(MenuListPopupRole);
    if (!list)
        return;

    if (list->accessibilityPlatformIncludesObject() == IgnoreObject) {
        cache->remove(list->axObjectID());
        return;
    }

    static_cast<AccessibilityMenuListPopup*>(list)->setMenuList(this);
    m_children.append(list);

    list->addChildren();
}

void AccessibilityMenuList::childrenChanged()
{
    if (m_children.isEmpty())
        return;

    ASSERT(m_children.size() == 1);
    m_children[0]->childrenChanged();
}

bool AccessibilityMenuList::isCollapsed() const
{
    return !static_cast<RenderMenuList*>(m_renderer)->popupIsVisible();
}

} // namespace WebCore
