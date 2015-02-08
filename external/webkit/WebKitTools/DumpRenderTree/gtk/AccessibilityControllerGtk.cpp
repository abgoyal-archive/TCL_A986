

#include "config.h"
#include "AccessibilityController.h"

#include "AccessibilityUIElement.h"
#include "DumpRenderTree.h"

#include <atk/atk.h>
#include <gtk/gtk.h>
#include <webkit/webkit.h>

extern "C" {
extern AtkObject* webkit_web_frame_get_focused_accessible_element(WebKitWebFrame*);
}

AccessibilityController::AccessibilityController()
{
}

AccessibilityController::~AccessibilityController()
{
}

AccessibilityUIElement AccessibilityController::focusedElement()
{
    AtkObject* accessible =  webkit_web_frame_get_focused_accessible_element(mainFrame);
    if (!accessible)
        return 0;

    return AccessibilityUIElement(accessible);
}

AccessibilityUIElement AccessibilityController::rootElement()
{
    WebKitWebView* view = webkit_web_frame_get_web_view(mainFrame);

    // The presumed, desired rootElement is the parent of the web view.
    GtkWidget* webViewParent = gtk_widget_get_parent(GTK_WIDGET(view));
    AtkObject* axObject = gtk_widget_get_accessible(webViewParent);
 
    return AccessibilityUIElement(axObject);
}

void AccessibilityController::setLogFocusEvents(bool)
{
}

void AccessibilityController::setLogScrollingStartEvents(bool)
{
}

void AccessibilityController::setLogValueChangeEvents(bool)
{
}

void AccessibilityController::addNotificationListener(PlatformUIElement, JSObjectRef)
{
}

void AccessibilityController::notificationReceived(PlatformUIElement, const std::string&)
{
}
