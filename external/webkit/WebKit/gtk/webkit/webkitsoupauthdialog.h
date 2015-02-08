

#include <gtk/gtk.h>
#include <libsoup/soup.h>

#include <webkit/webkitdefines.h>

#ifndef webkitsoupauthdialog_h
#define webkitsoupauthdialog_h

G_BEGIN_DECLS

#define WEBKIT_TYPE_SOUP_AUTH_DIALOG            (webkit_soup_auth_dialog_get_type ())
#define WEBKIT_SOUP_AUTH_DIALOG(object)         (G_TYPE_CHECK_INSTANCE_CAST ((object), WEBKIT_TYPE_SOUP_AUTH_DIALOG, WebKitSoupAuthDialog))
#define WEBKIT_SOUP_AUTH_DIALOG_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST ((klass), WEBKIT_TYPE_SOUP_AUTH_DIALOG, WebKitSoupAuthDialog))
#define WEBKIT_IS_SOUP_AUTH_DIALOG(object)      (G_TYPE_CHECK_INSTANCE_TYPE ((object), WEBKIT_TYPE_SOUP_AUTH_DIALOG))
#define WEBKIT_IS_SOUP_AUTH_DIALOG_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE ((klass), WEBKIT_TYPE_SOUP_AUTH_DIALOG))
#define WEBKIT_SOUP_AUTH_DIALOG_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS ((obj), WEBKIT_TYPE_SOUP_AUTH_DIALOG, WebKitSoupAuthDialog))

typedef struct {
    GObject parent_instance;
} WebKitSoupAuthDialog;

typedef struct {
    GObjectClass parent_class;

    GtkWidget* (*current_toplevel) (WebKitSoupAuthDialog* feature, SoupMessage* message);
} WebKitSoupAuthDialogClass;

WEBKIT_API GType
webkit_soup_auth_dialog_get_type (void);

G_END_DECLS

#endif /* webkitsoupauthdialog_h */
