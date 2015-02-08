

#ifndef webkithittestresult_h
#define webkithittestresult_h

#include <glib-object.h>

#include <webkit/webkitdefines.h>

G_BEGIN_DECLS

#define WEBKIT_TYPE_HIT_TEST_RESULT            (webkit_hit_test_result_get_type())
#define WEBKIT_HIT_TEST_RESULT(obj)            (G_TYPE_CHECK_INSTANCE_CAST((obj), WEBKIT_TYPE_HIT_TEST_RESULT, WebKitHitTestResult))
#define WEBKIT_HIT_TEST_RESULT_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST((klass),  WEBKIT_TYPE_HIT_TEST_RESULT, WebKitHitTestResultClass))
#define WEBKIT_IS_HIT_TEST_RESULT(obj)         (G_TYPE_CHECK_INSTANCE_TYPE((obj), WEBKIT_TYPE_HIT_TEST_RESULT))
#define WEBKIT_IS_HIT_TEST_RESULT_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass),  WEBKIT_TYPE_HIT_TEST_RESULT))
#define WEBKIT_HIT_TEST_RESULT_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS((obj),  WEBKIT_TYPE_WEB_HIT_TEST_RESULT, WebKitHitTestResultClass))

typedef struct _WebKitHitTestResultPrivate WebKitHitTestResultPrivate;

struct _WebKitHitTestResult {
    GObject parent_instance;

    /*< private >*/
    WebKitHitTestResultPrivate *priv;
};

struct _WebKitHitTestResultClass {
    GObjectClass parent_class;

    /* Padding for future expansion */
    void (*_webkit_reserved0) (void);
    void (*_webkit_reserved1) (void);
    void (*_webkit_reserved2) (void);
    void (*_webkit_reserved3) (void);
};

typedef enum
{
    WEBKIT_HIT_TEST_RESULT_CONTEXT_DOCUMENT   = 1 << 1,
    WEBKIT_HIT_TEST_RESULT_CONTEXT_LINK       = 1 << 2,
    WEBKIT_HIT_TEST_RESULT_CONTEXT_IMAGE      = 1 << 3,
    WEBKIT_HIT_TEST_RESULT_CONTEXT_MEDIA      = 1 << 4,
    WEBKIT_HIT_TEST_RESULT_CONTEXT_SELECTION  = 1 << 5,
    WEBKIT_HIT_TEST_RESULT_CONTEXT_EDITABLE   = 1 << 6,
} WebKitHitTestResultContext;

WEBKIT_API GType
webkit_hit_test_result_get_type (void);

G_END_DECLS

#endif

