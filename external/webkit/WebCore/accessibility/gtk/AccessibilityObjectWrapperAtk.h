

#ifndef AccessibilityObjectWrapperAtk_h
#define AccessibilityObjectWrapperAtk_h

#include <atk/atk.h>

namespace WebCore {
    class AccessibilityObject;
}

G_BEGIN_DECLS

#define WEBKIT_TYPE_ACCESSIBLE                  (webkit_accessible_get_type ())
#define WEBKIT_ACCESSIBLE(obj)                  (G_TYPE_CHECK_INSTANCE_CAST ((obj), WEBKIT_TYPE_ACCESSIBLE, WebKitAccessible))
#define WEBKIT_ACCESSIBLE_CLASS(klass)          (G_TYPE_CHECK_CLASS_CAST ((klass), WEBKIT_TYPE_ACCESSIBLE, WebKitAccessibleClass))
#define WEBKIT_IS_ACCESSIBLE(obj)               (G_TYPE_CHECK_INSTANCE_TYPE ((obj), WEBKIT_TYPE_ACCESSIBLE))
#define WEBKIT_IS_ACCESSIBLE_CLASS(klass)       (G_TYPE_CHECK_CLASS_TYPE ((klass), WEBKIT_TYPE_ACCESSIBLE))
#define WEBKIT_ACCESSIBLE_GET_CLASS(obj)        (G_TYPE_INSTANCE_GET_CLASS ((obj), WEBKIT_TYPE_ACCESSIBLE, WebKitAccessibleClass))

typedef struct _WebKitAccessible                WebKitAccessible;
typedef struct _WebKitAccessibleClass           WebKitAccessibleClass;

struct _WebKitAccessible
{
    AtkObject parent;
    WebCore::AccessibilityObject* m_object;
};

struct _WebKitAccessibleClass
{
    AtkObjectClass parent_class;
};

GType webkit_accessible_get_type (void) G_GNUC_CONST;

WebKitAccessible* webkit_accessible_new (WebCore::AccessibilityObject* core_object);

WebCore::AccessibilityObject* webkit_accessible_get_accessibility_object (WebKitAccessible* accessible);

void webkit_accessible_detach (WebKitAccessible* accessible);

AtkObject* webkit_accessible_get_focused_element(WebKitAccessible* accessible);

WebCore::AccessibilityObject* objectAndOffsetUnignored(WebCore::AccessibilityObject* coreObject, int& offset, bool ignoreLinks);

G_END_DECLS

#endif // AccessibilityObjectWrapperAtk_h
