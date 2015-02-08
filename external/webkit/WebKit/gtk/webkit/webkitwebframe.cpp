

#include "config.h"

#include "webkitenumtypes.h"
#include "webkitwebframe.h"
#include "webkitwebview.h"
#include "webkitmarshal.h"
#include "webkitprivate.h"

#include "AccessibilityObjectWrapperAtk.h"
#include "AnimationController.h"
#include "AXObjectCache.h"
#include "CString.h"
#include "DocumentLoader.h"
#include "DocumentLoaderGtk.h"
#include "FrameLoader.h"
#include "FrameLoaderClientGtk.h"
#include "FrameTree.h"
#include "FrameView.h"
#include <glib/gi18n-lib.h>
#include "GCController.h"
#include "GraphicsContext.h"
#include "HTMLFrameOwnerElement.h"
#include "JSDOMWindow.h"
#include "JSLock.h"
#include "PrintContext.h"
#include "RenderView.h"
#include "RenderTreeAsText.h"
#include "JSDOMBinding.h"
#include "ScriptController.h"
#include "SubstituteData.h"
#if ENABLE(SVG)
#include "SVGSMILElement.h"
#endif

#include <atk/atk.h>
#include <JavaScriptCore/APICast.h>


using namespace WebKit;
using namespace WebCore;
using namespace std;

enum {
    CLEARED,
    LOAD_COMMITTED,
    LOAD_DONE,
    TITLE_CHANGED,
    HOVERING_OVER_LINK,
    SCROLLBARS_POLICY_CHANGED,
    LAST_SIGNAL
};

enum {
    PROP_0,

    PROP_NAME,
    PROP_TITLE,
    PROP_URI,
    PROP_LOAD_STATUS,
    PROP_HORIZONTAL_SCROLLBAR_POLICY,
    PROP_VERTICAL_SCROLLBAR_POLICY
};

static guint webkit_web_frame_signals[LAST_SIGNAL] = { 0, };

G_DEFINE_TYPE(WebKitWebFrame, webkit_web_frame, G_TYPE_OBJECT)

static void webkit_web_frame_get_property(GObject* object, guint prop_id, GValue* value, GParamSpec* pspec)
{
    WebKitWebFrame* frame = WEBKIT_WEB_FRAME(object);

    switch(prop_id) {
    case PROP_NAME:
        g_value_set_string(value, webkit_web_frame_get_name(frame));
        break;
    case PROP_TITLE:
        g_value_set_string(value, webkit_web_frame_get_title(frame));
        break;
    case PROP_URI:
        g_value_set_string(value, webkit_web_frame_get_uri(frame));
        break;
    case PROP_LOAD_STATUS:
        g_value_set_enum(value, webkit_web_frame_get_load_status(frame));
        break;
    case PROP_HORIZONTAL_SCROLLBAR_POLICY:
        g_value_set_enum(value, webkit_web_frame_get_horizontal_scrollbar_policy(frame));
        break;
    case PROP_VERTICAL_SCROLLBAR_POLICY:
        g_value_set_enum(value, webkit_web_frame_get_vertical_scrollbar_policy(frame));
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }
}

// Called from the FrameLoaderClient when it is destroyed. Normally
// the unref in the FrameLoaderClient is destroying this object as
// well but due reference counting a user might have added a reference...
void webkit_web_frame_core_frame_gone(WebKitWebFrame* frame)
{
    ASSERT(WEBKIT_IS_WEB_FRAME(frame));
    frame->priv->coreFrame = 0;
}

static WebKitWebDataSource* webkit_web_frame_get_data_source_from_core_loader(WebCore::DocumentLoader* loader)
{
    return loader ? static_cast<WebKit::DocumentLoader*>(loader)->dataSource() : NULL;
}

static void webkit_web_frame_finalize(GObject* object)
{
    WebKitWebFrame* frame = WEBKIT_WEB_FRAME(object);
    WebKitWebFramePrivate* priv = frame->priv;

    if (priv->coreFrame) {
        priv->coreFrame->loader()->cancelAndClear();
        priv->coreFrame = 0;
    }

    g_free(priv->name);
    g_free(priv->title);
    g_free(priv->uri);

    G_OBJECT_CLASS(webkit_web_frame_parent_class)->finalize(object);
}

static void webkit_web_frame_class_init(WebKitWebFrameClass* frameClass)
{
    webkit_init();

    /*
     * signals
     */
    webkit_web_frame_signals[CLEARED] = g_signal_new("cleared",
            G_TYPE_FROM_CLASS(frameClass),
            (GSignalFlags)G_SIGNAL_RUN_LAST,
            0,
            NULL,
            NULL,
            g_cclosure_marshal_VOID__VOID,
            G_TYPE_NONE, 0);

    /**
     * WebKitWebFrame::load-done
     * @web_frame: the object on which the signal is emitted
     *
     * Emitted when frame loading is done.
     *
     * Deprecated: Use the "load-status" property instead.
     */
    webkit_web_frame_signals[LOAD_COMMITTED] = g_signal_new("load-committed",
            G_TYPE_FROM_CLASS(frameClass),
            (GSignalFlags)G_SIGNAL_RUN_LAST,
            0,
            NULL,
            NULL,
            g_cclosure_marshal_VOID__VOID,
            G_TYPE_NONE, 0);

    /**
     * WebKitWebFrame::load-done
     * @web_frame: the object on which the signal is emitted
     *
     * Emitted when frame loading is done.
     *
     * Deprecated: Use the "load-status" property instead, and/or
     * WebKitWebView::load-error to be notified of load errors
     */
    webkit_web_frame_signals[LOAD_DONE] = g_signal_new("load-done",
            G_TYPE_FROM_CLASS(frameClass),
            (GSignalFlags)G_SIGNAL_RUN_LAST,
            0,
            NULL,
            NULL,
            g_cclosure_marshal_VOID__BOOLEAN,
            G_TYPE_NONE, 1,
            G_TYPE_BOOLEAN);

    /**
     * WebKitWebFrame::title-changed:
     * @frame: the object on which the signal is emitted
     * @title: the new title
     *
     * When a #WebKitWebFrame changes the document title this signal is emitted.
     *
     * Deprecated: 1.1.18: Use "notify::title" instead.
     */
    webkit_web_frame_signals[TITLE_CHANGED] = g_signal_new("title-changed",
            G_TYPE_FROM_CLASS(frameClass),
            (GSignalFlags)G_SIGNAL_RUN_LAST,
            0,
            NULL,
            NULL,
            webkit_marshal_VOID__STRING,
            G_TYPE_NONE, 1,
            G_TYPE_STRING);

    webkit_web_frame_signals[HOVERING_OVER_LINK] = g_signal_new("hovering-over-link",
            G_TYPE_FROM_CLASS(frameClass),
            (GSignalFlags)G_SIGNAL_RUN_LAST,
            0,
            NULL,
            NULL,
            webkit_marshal_VOID__STRING_STRING,
            G_TYPE_NONE, 2,
            G_TYPE_STRING, G_TYPE_STRING);

    /**
     * WebKitWebFrame::scrollbars-policy-changed:
     * @web_view: the object which received the signal
     *
     * Signal emitted when policy for one or both of the scrollbars of
     * the view has changed. The default handler will apply the new
     * policy to the container that holds the #WebKitWebFrame if it is
     * a #GtkScrolledWindow and the frame is the main frame. If you do
     * not want this to be handled automatically, you need to handle
     * this signal.
     *
     * The exception to this rule is that policies to disable the
     * scrollbars are applied as %GTK_POLICY_AUTOMATIC instead, since
     * the size request of the widget would force browser windows to
     * not be resizable.
     *
     * You can obtain the new policies from the
     * WebKitWebFrame:horizontal-scrollbar-policy and
     * WebKitWebFrame:vertical-scrollbar-policy properties.
     *
     * Return value: %TRUE to stop other handlers from being invoked for the
     * event. %FALSE to propagate the event further.
     *
     * Since: 1.1.14
     */
    webkit_web_frame_signals[SCROLLBARS_POLICY_CHANGED] = g_signal_new("scrollbars-policy-changed",
            G_TYPE_FROM_CLASS(frameClass),
            (GSignalFlags)(G_SIGNAL_RUN_LAST | G_SIGNAL_ACTION),
            0,
            g_signal_accumulator_true_handled,
            NULL,
            webkit_marshal_BOOLEAN__VOID,
            G_TYPE_BOOLEAN, 0);

    /*
     * implementations of virtual methods
     */
    GObjectClass* objectClass = G_OBJECT_CLASS(frameClass);
    objectClass->finalize = webkit_web_frame_finalize;
    objectClass->get_property = webkit_web_frame_get_property;

    /*
     * properties
     */
    g_object_class_install_property(objectClass, PROP_NAME,
                                    g_param_spec_string("name",
                                                        _("Name"),
                                                        _("The name of the frame"),
                                                        NULL,
                                                        WEBKIT_PARAM_READABLE));

    g_object_class_install_property(objectClass, PROP_TITLE,
                                    g_param_spec_string("title",
                                                        _("Title"),
                                                        _("The document title of the frame"),
                                                        NULL,
                                                        WEBKIT_PARAM_READABLE));

    g_object_class_install_property(objectClass, PROP_URI,
                                    g_param_spec_string("uri",
                                                        _("URI"),
                                                        _("The current URI of the contents displayed by the frame"),
                                                        NULL,
                                                        WEBKIT_PARAM_READABLE));

    /**
    * WebKitWebFrame:load-status:
    *
    * Determines the current status of the load.
    *
    * Since: 1.1.7
    */
    g_object_class_install_property(objectClass, PROP_LOAD_STATUS,
                                    g_param_spec_enum("load-status",
                                                      "Load Status",
                                                      "Determines the current status of the load",
                                                      WEBKIT_TYPE_LOAD_STATUS,
                                                      WEBKIT_LOAD_FINISHED,
                                                      WEBKIT_PARAM_READABLE));

    /**
     * WebKitWebFrame:horizontal-scrollbar-policy:
     *
     * Determines the current policy for the horizontal scrollbar of
     * the frame. For the main frame, make sure to set the same policy
     * on the scrollable widget containing the #WebKitWebView, unless
     * you know what you are doing.
     *
     * Since: 1.1.14
     */
    g_object_class_install_property(objectClass, PROP_HORIZONTAL_SCROLLBAR_POLICY,
                                    g_param_spec_enum("horizontal-scrollbar-policy",
                                                      _("Horizontal Scrollbar Policy"),
                                                      _("Determines the current policy for the horizontal scrollbar of the frame."),
                                                      GTK_TYPE_POLICY_TYPE,
                                                      GTK_POLICY_AUTOMATIC,
                                                      WEBKIT_PARAM_READABLE));

    /**
     * WebKitWebFrame:vertical-scrollbar-policy:
     *
     * Determines the current policy for the vertical scrollbar of
     * the frame. For the main frame, make sure to set the same policy
     * on the scrollable widget containing the #WebKitWebView, unless
     * you know what you are doing.
     *
     * Since: 1.1.14
     */
    g_object_class_install_property(objectClass, PROP_VERTICAL_SCROLLBAR_POLICY,
                                    g_param_spec_enum("vertical-scrollbar-policy",
                                                      _("Vertical Scrollbar Policy"),
                                                      _("Determines the current policy for the vertical scrollbar of the frame."),
                                                      GTK_TYPE_POLICY_TYPE,
                                                      GTK_POLICY_AUTOMATIC,
                                                      WEBKIT_PARAM_READABLE));

    g_type_class_add_private(frameClass, sizeof(WebKitWebFramePrivate));
}

static void webkit_web_frame_init(WebKitWebFrame* frame)
{
    WebKitWebFramePrivate* priv = WEBKIT_WEB_FRAME_GET_PRIVATE(frame);

    // TODO: Move constructor code here.
    frame->priv = priv;
}

WebKitWebFrame* webkit_web_frame_new(WebKitWebView* webView)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_VIEW(webView), NULL);

    WebKitWebFrame* frame = WEBKIT_WEB_FRAME(g_object_new(WEBKIT_TYPE_WEB_FRAME, NULL));
    WebKitWebFramePrivate* priv = frame->priv;
    WebKitWebViewPrivate* viewPriv = WEBKIT_WEB_VIEW_GET_PRIVATE(webView);

    priv->webView = webView;
    WebKit::FrameLoaderClient* client = new WebKit::FrameLoaderClient(frame);
    priv->coreFrame = Frame::create(viewPriv->corePage, 0, client).get();
    priv->coreFrame->init();

    priv->origin = NULL;

    return frame;
}

PassRefPtr<Frame> webkit_web_frame_init_with_web_view(WebKitWebView* webView, HTMLFrameOwnerElement* element)
{
    WebKitWebFrame* frame = WEBKIT_WEB_FRAME(g_object_new(WEBKIT_TYPE_WEB_FRAME, NULL));
    WebKitWebFramePrivate* priv = frame->priv;
    WebKitWebViewPrivate* viewPriv = WEBKIT_WEB_VIEW_GET_PRIVATE(webView);

    priv->webView = webView;
    WebKit::FrameLoaderClient* client = new WebKit::FrameLoaderClient(frame);

    RefPtr<Frame> coreFrame = Frame::create(viewPriv->corePage, element, client);
    priv->coreFrame = coreFrame.get();

    return coreFrame.release();
}

G_CONST_RETURN gchar* webkit_web_frame_get_title(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

    WebKitWebFramePrivate* priv = frame->priv;
    return priv->title;
}

G_CONST_RETURN gchar* webkit_web_frame_get_uri(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

    WebKitWebFramePrivate* priv = frame->priv;
    return priv->uri;
}

WebKitWebView* webkit_web_frame_get_web_view(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

    WebKitWebFramePrivate* priv = frame->priv;
    return priv->webView;
}

G_CONST_RETURN gchar* webkit_web_frame_get_name(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

    WebKitWebFramePrivate* priv = frame->priv;

    if (priv->name)
        return priv->name;

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return "";

    String string = coreFrame->tree()->name();
    priv->name = g_strdup(string.utf8().data());
    return priv->name;
}

WebKitWebFrame* webkit_web_frame_get_parent(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return NULL;

    return kit(coreFrame->tree()->parent());
}

void webkit_web_frame_load_uri(WebKitWebFrame* frame, const gchar* uri)
{
    g_return_if_fail(WEBKIT_IS_WEB_FRAME(frame));
    g_return_if_fail(uri);

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return;

    coreFrame->loader()->load(ResourceRequest(KURL(KURL(), String::fromUTF8(uri))), false);
}

static void webkit_web_frame_load_data(WebKitWebFrame* frame, const gchar* content, const gchar* mimeType, const gchar* encoding, const gchar* baseURL, const gchar* unreachableURL)
{
    Frame* coreFrame = core(frame);
    ASSERT(coreFrame);

    KURL baseKURL = baseURL ? KURL(KURL(), String::fromUTF8(baseURL)) : blankURL();

    ResourceRequest request(baseKURL);

    RefPtr<SharedBuffer> sharedBuffer = SharedBuffer::create(content, strlen(content));
    SubstituteData substituteData(sharedBuffer.release(),
                                  mimeType ? String::fromUTF8(mimeType) : String::fromUTF8("text/html"),
                                  encoding ? String::fromUTF8(encoding) : String::fromUTF8("UTF-8"),
                                  KURL(KURL(), String::fromUTF8(unreachableURL)),
                                  KURL(KURL(), String::fromUTF8(unreachableURL)));

    coreFrame->loader()->load(request, substituteData, false);
}

void webkit_web_frame_load_string(WebKitWebFrame* frame, const gchar* content, const gchar* contentMimeType, const gchar* contentEncoding, const gchar* baseUri)
{
    g_return_if_fail(WEBKIT_IS_WEB_FRAME(frame));
    g_return_if_fail(content);

    webkit_web_frame_load_data(frame, content, contentMimeType, contentEncoding, baseUri, NULL);
}

void webkit_web_frame_load_alternate_string(WebKitWebFrame* frame, const gchar* content, const gchar* baseURL, const gchar* unreachableURL)
{
    g_return_if_fail(WEBKIT_IS_WEB_FRAME(frame));
    g_return_if_fail(content);

    webkit_web_frame_load_data(frame, content, NULL, NULL, baseURL, unreachableURL);
}

void webkit_web_frame_load_request(WebKitWebFrame* frame, WebKitNetworkRequest* request)
{
    g_return_if_fail(WEBKIT_IS_WEB_FRAME(frame));
    g_return_if_fail(WEBKIT_IS_NETWORK_REQUEST(request));

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return;

    coreFrame->loader()->load(core(request), false);
}

void webkit_web_frame_stop_loading(WebKitWebFrame* frame)
{
    g_return_if_fail(WEBKIT_IS_WEB_FRAME(frame));

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return;

    coreFrame->loader()->stopAllLoaders();
}

void webkit_web_frame_reload(WebKitWebFrame* frame)
{
    g_return_if_fail(WEBKIT_IS_WEB_FRAME(frame));

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return;

    coreFrame->loader()->reload();
}

WebKitWebFrame* webkit_web_frame_find_frame(WebKitWebFrame* frame, const gchar* name)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);
    g_return_val_if_fail(name, NULL);

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return NULL;

    String nameString = String::fromUTF8(name);
    return kit(coreFrame->tree()->find(AtomicString(nameString)));
}

JSGlobalContextRef webkit_web_frame_get_global_context(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return NULL;

    return toGlobalRef(coreFrame->script()->globalObject(mainThreadNormalWorld())->globalExec());
}

WebKitWebDataSource* webkit_web_frame_get_data_source(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

    Frame* coreFrame = core(frame);
    return webkit_web_frame_get_data_source_from_core_loader(coreFrame->loader()->documentLoader());
}

WebKitWebDataSource* webkit_web_frame_get_provisional_data_source(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

    Frame* coreFrame = core(frame);
    return webkit_web_frame_get_data_source_from_core_loader(coreFrame->loader()->provisionalDocumentLoader());
}

GSList* webkit_web_frame_get_children(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return NULL;

    GSList* children = NULL;
    for (Frame* child = coreFrame->tree()->firstChild(); child; child = child->tree()->nextSibling()) {
        FrameLoader* loader = child->loader();
        WebKit::FrameLoaderClient* client = static_cast<WebKit::FrameLoaderClient*>(loader->client());
        if (client)
          children = g_slist_append(children, client->webFrame());
    }

    return children;
}

gchar* webkit_web_frame_get_inner_text(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return g_strdup("");

    FrameView* view = coreFrame->view();

    if (view && view->layoutPending())
        view->layout();

    Element* documentElement = coreFrame->document()->documentElement();
    String string =  documentElement->innerText();
    return g_strdup(string.utf8().data());
}

gchar* webkit_web_frame_dump_render_tree(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return g_strdup("");

    FrameView* view = coreFrame->view();

    if (view && view->layoutPending())
        view->layout();

    String string = externalRepresentation(coreFrame);
    return g_strdup(string.utf8().data());
}

gchar* webkit_web_frame_counter_value_for_element_by_id(WebKitWebFrame* frame, const gchar* id)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return 0;

    Element* coreElement = coreFrame->document()->getElementById(AtomicString(id));
    if (!coreElement)
        return 0;
    String counterValue = counterValueForElement(coreElement);
    return g_strdup(counterValue.utf8().data());
}

int webkit_web_frame_page_number_for_element_by_id(WebKitWebFrame* frame, const gchar* id, float pageWidth, float pageHeight)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return -1;

    Element* coreElement = coreFrame->document()->getElementById(AtomicString(id));
    if (!coreElement)
        return -1;
    return PrintContext::pageNumberForElement(coreElement, FloatSize(pageWidth, pageHeight));
}

guint webkit_web_frame_get_pending_unload_event_count(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), 0);

    return core(frame)->domWindow()->pendingUnloadEventListeners();
}

static void begin_print_callback(GtkPrintOperation* op, GtkPrintContext* context, gpointer user_data)
{
    PrintContext* printContext = reinterpret_cast<PrintContext*>(user_data);

    float width = gtk_print_context_get_width(context);
    float height = gtk_print_context_get_height(context);
    FloatRect printRect = FloatRect(0, 0, width, height);

    printContext->begin(width);

    // TODO: Margin adjustments and header/footer support
    float headerHeight = 0;
    float footerHeight = 0;
    float pageHeight; // height of the page adjusted by margins
    printContext->computePageRects(printRect, headerHeight, footerHeight, 1.0, pageHeight);
    gtk_print_operation_set_n_pages(op, printContext->pageCount());
}

static void draw_page_callback(GtkPrintOperation* op, GtkPrintContext* context, gint page_nr, gpointer user_data)
{
    PrintContext* printContext = reinterpret_cast<PrintContext*>(user_data);

    if (page_nr >= printContext->pageCount())
        return;

    cairo_t* cr = gtk_print_context_get_cairo_context(context);
    GraphicsContext ctx(cr);
    float width = gtk_print_context_get_width(context);
    printContext->spoolPage(ctx, page_nr, width);
}

static void end_print_callback(GtkPrintOperation* op, GtkPrintContext* context, gpointer user_data)
{
    PrintContext* printContext = reinterpret_cast<PrintContext*>(user_data);
    printContext->end();
}

GtkPrintOperationResult webkit_web_frame_print_full(WebKitWebFrame* frame, GtkPrintOperation* operation, GtkPrintOperationAction action, GError** error)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), GTK_PRINT_OPERATION_RESULT_ERROR);
    g_return_val_if_fail(GTK_IS_PRINT_OPERATION(operation), GTK_PRINT_OPERATION_RESULT_ERROR);

    GtkWidget* topLevel = gtk_widget_get_toplevel(GTK_WIDGET(webkit_web_frame_get_web_view(frame)));

#if GTK_CHECK_VERSION(2, 18, 0)
    if (!gtk_widget_is_toplevel(topLevel))
#else
    if (!GTK_WIDGET_TOPLEVEL(topLevel))
#endif
        topLevel = NULL;

    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return GTK_PRINT_OPERATION_RESULT_ERROR;

    PrintContext printContext(coreFrame);

    g_signal_connect(operation, "begin-print", G_CALLBACK(begin_print_callback), &printContext);
    g_signal_connect(operation, "draw-page", G_CALLBACK(draw_page_callback), &printContext);
    g_signal_connect(operation, "end-print", G_CALLBACK(end_print_callback), &printContext);

    return gtk_print_operation_run(operation, action, GTK_WINDOW(topLevel), error);
}

void webkit_web_frame_print(WebKitWebFrame* frame)
{
    g_return_if_fail(WEBKIT_IS_WEB_FRAME(frame));

    WebKitWebFramePrivate* priv = frame->priv;
    GtkPrintOperation* operation = gtk_print_operation_new();
    GError* error = 0;

    webkit_web_frame_print_full(frame, operation, GTK_PRINT_OPERATION_ACTION_PRINT_DIALOG, &error);
    g_object_unref(operation);

    if (error) {
        GtkWidget* window = gtk_widget_get_toplevel(GTK_WIDGET(priv->webView));
#if GTK_CHECK_VERSION(2, 18, 0)
        GtkWidget* dialog = gtk_message_dialog_new(gtk_widget_is_toplevel(window) ? GTK_WINDOW(window) : 0,
                                                   GTK_DIALOG_DESTROY_WITH_PARENT,
                                                   GTK_MESSAGE_ERROR,
                                                   GTK_BUTTONS_CLOSE,
                                                   "%s", error->message);
#else
        GtkWidget* dialog = gtk_message_dialog_new(GTK_WIDGET_TOPLEVEL(window) ? GTK_WINDOW(window) : 0,
                                                   GTK_DIALOG_DESTROY_WITH_PARENT,
                                                   GTK_MESSAGE_ERROR,
                                                   GTK_BUTTONS_CLOSE,
                                                   "%s", error->message);
#endif

        g_error_free(error);

        g_signal_connect(dialog, "response", G_CALLBACK(gtk_widget_destroy), NULL);
        gtk_widget_show(dialog);
    }
}

bool webkit_web_frame_pause_animation(WebKitWebFrame* frame, const gchar* name, double time, const gchar* element)
{
    ASSERT(core(frame));
    Element* coreElement = core(frame)->document()->getElementById(AtomicString(element));
    if (!coreElement || !coreElement->renderer())
        return false;
    return core(frame)->animation()->pauseAnimationAtTime(coreElement->renderer(), AtomicString(name), time);
}

bool webkit_web_frame_pause_transition(WebKitWebFrame* frame, const gchar* name, double time, const gchar* element)
{
    ASSERT(core(frame));
    Element* coreElement = core(frame)->document()->getElementById(AtomicString(element));
    if (!coreElement || !coreElement->renderer())
        return false;
    return core(frame)->animation()->pauseTransitionAtTime(coreElement->renderer(), AtomicString(name), time);
}

bool webkit_web_frame_pause_svg_animation(WebKitWebFrame* frame, const gchar* animationId, double time, const gchar* elementId)
{
    ASSERT(core(frame));
#if ENABLE(SVG)
    Document* document = core(frame)->document();
    if (!document || !document->svgExtensions())
        return false;
    Element* coreElement = document->getElementById(AtomicString(animationId));
    if (!coreElement || !SVGSMILElement::isSMILElement(coreElement))
        return false;
    return document->accessSVGExtensions()->sampleAnimationAtTime(elementId, static_cast<SVGSMILElement*>(coreElement), time);
#else
    return false;
#endif
}

unsigned int webkit_web_frame_number_of_active_animations(WebKitWebFrame* frame)
{
    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return 0;

    AnimationController* controller = coreFrame->animation();
    if (!controller)
        return 0;

    return controller->numberOfActiveAnimations();
}

gchar* webkit_web_frame_get_response_mime_type(WebKitWebFrame* frame)
{
    Frame* coreFrame = core(frame);
    WebCore::DocumentLoader* docLoader = coreFrame->loader()->documentLoader();
    String mimeType = docLoader->responseMIMEType();
    return g_strdup(mimeType.utf8().data());
}

WebKitLoadStatus webkit_web_frame_get_load_status(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), WEBKIT_LOAD_FINISHED);

    WebKitWebFramePrivate* priv = frame->priv;
    return priv->loadStatus;
}

void webkit_web_frame_clear_main_frame_name(WebKitWebFrame* frame)
{
    g_return_if_fail(WEBKIT_IS_WEB_FRAME(frame));

    core(frame)->tree()->clearName();
}

void webkit_gc_collect_javascript_objects()
{
    gcController().garbageCollectNow();
}

void webkit_gc_collect_javascript_objects_on_alternate_thread(gboolean waitUntilDone)
{
    gcController().garbageCollectOnAlternateThreadForDebugging(waitUntilDone);
}

gsize webkit_gc_count_javascript_objects()
{
    JSC::JSLock lock(JSC::SilenceAssertionsOnly);
    return JSDOMWindow::commonJSGlobalData()->heap.objectCount();

}

AtkObject* webkit_web_frame_get_focused_accessible_element(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), NULL);

#if HAVE(ACCESSIBILITY)
    if (!AXObjectCache::accessibilityEnabled())
        AXObjectCache::enableAccessibility();

    WebKitWebFramePrivate* priv = frame->priv;
    if (!priv->coreFrame || !priv->coreFrame->document())
        return NULL;

    RenderView* root = toRenderView(priv->coreFrame->document()->renderer());
    if (!root)
        return NULL;

    AtkObject* wrapper =  priv->coreFrame->document()->axObjectCache()->getOrCreate(root)->wrapper();
    if (!wrapper)
        return NULL;

    return webkit_accessible_get_focused_element(WEBKIT_ACCESSIBLE(wrapper));
#else
    return NULL;
#endif
}

GtkPolicyType webkit_web_frame_get_horizontal_scrollbar_policy(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), GTK_POLICY_AUTOMATIC);

    Frame* coreFrame = core(frame);
    FrameView* view = coreFrame->view();
    if (!view)
        return GTK_POLICY_AUTOMATIC;

    ScrollbarMode hMode = view->horizontalScrollbarMode();

    if (hMode == ScrollbarAlwaysOn)
        return GTK_POLICY_ALWAYS;

    if (hMode == ScrollbarAlwaysOff)
        return GTK_POLICY_NEVER;

    return GTK_POLICY_AUTOMATIC;
}

GtkPolicyType webkit_web_frame_get_vertical_scrollbar_policy(WebKitWebFrame* frame)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_FRAME(frame), GTK_POLICY_AUTOMATIC);

    Frame* coreFrame = core(frame);
    FrameView* view = coreFrame->view();
    if (!view)
        return GTK_POLICY_AUTOMATIC;

    ScrollbarMode vMode = view->verticalScrollbarMode();

    if (vMode == ScrollbarAlwaysOn)
        return GTK_POLICY_ALWAYS;

    if (vMode == ScrollbarAlwaysOff)
        return GTK_POLICY_NEVER;

    return GTK_POLICY_AUTOMATIC;
}

WebKitSecurityOrigin* webkit_web_frame_get_security_origin(WebKitWebFrame* frame)
{
    WebKitWebFramePrivate* priv = frame->priv;
    if (!priv->coreFrame || !priv->coreFrame->document() || !priv->coreFrame->document()->securityOrigin())
        return NULL;

    if (priv->origin && priv->origin->priv->coreOrigin.get() == priv->coreFrame->document()->securityOrigin())
        return priv->origin;

    if (priv->origin)
        g_object_unref(priv->origin);

    priv->origin = kit(priv->coreFrame->document()->securityOrigin());
    return priv->origin;
}

void webkit_web_frame_layout(WebKitWebFrame* frame)
{
    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return;

    FrameView* view = coreFrame->view();
    if (!view)
        return;

    view->layout();
}

WebKitNetworkResponse* webkit_web_frame_get_network_response(WebKitWebFrame* frame)
{
    Frame* coreFrame = core(frame);
    if (!coreFrame)
        return NULL;

    WebCore::DocumentLoader* loader = coreFrame->loader()->activeDocumentLoader();
    if (!loader)
        return NULL;

    return webkit_network_response_new_with_core_response(loader->response());
}
