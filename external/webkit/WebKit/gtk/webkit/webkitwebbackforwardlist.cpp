

#include "config.h"

#include "webkitwebbackforwardlist.h"
#include "webkitprivate.h"
#include "webkitwebhistoryitem.h"
#include "webkitwebview.h"

#include <glib.h>

#include "BackForwardList.h"
#include "HistoryItem.h"


using namespace WebKit;

struct _WebKitWebBackForwardListPrivate {
    WebCore::BackForwardList* backForwardList;
    gboolean disposed;
};

#define WEBKIT_WEB_BACK_FORWARD_LIST_GET_PRIVATE(obj)    (G_TYPE_INSTANCE_GET_PRIVATE((obj), WEBKIT_TYPE_WEB_BACK_FORWARD_LIST, WebKitWebBackForwardListPrivate))

G_DEFINE_TYPE(WebKitWebBackForwardList, webkit_web_back_forward_list, G_TYPE_OBJECT);

static void webkit_web_back_forward_list_dispose(GObject* object)
{
    WebKitWebBackForwardList* list = WEBKIT_WEB_BACK_FORWARD_LIST(object);
    WebCore::BackForwardList* backForwardList = core(list);
    WebKitWebBackForwardListPrivate* priv = list->priv;

    if (!priv->disposed) {
        priv->disposed = true;

        WebCore::HistoryItemVector items = backForwardList->entries();
        GHashTable* table = webkit_history_items();
        for (unsigned i = 0; i < items.size(); i++)
            g_hash_table_remove(table, items[i].get());
    }

    G_OBJECT_CLASS(webkit_web_back_forward_list_parent_class)->dispose(object);
}

static void webkit_web_back_forward_list_class_init(WebKitWebBackForwardListClass* klass)
{
    GObjectClass* object_class = G_OBJECT_CLASS(klass);

    object_class->dispose = webkit_web_back_forward_list_dispose;

    webkit_init();

    g_type_class_add_private(klass, sizeof(WebKitWebBackForwardListPrivate));
}

static void webkit_web_back_forward_list_init(WebKitWebBackForwardList* webBackForwardList)
{
    webBackForwardList->priv = WEBKIT_WEB_BACK_FORWARD_LIST_GET_PRIVATE(webBackForwardList);
}

WebKitWebBackForwardList* webkit_web_back_forward_list_new_with_web_view(WebKitWebView* webView)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_VIEW(webView), NULL);

    WebKitWebBackForwardList* webBackForwardList;

    webBackForwardList = WEBKIT_WEB_BACK_FORWARD_LIST(g_object_new(WEBKIT_TYPE_WEB_BACK_FORWARD_LIST, NULL));
    WebKitWebBackForwardListPrivate* priv = webBackForwardList->priv;

    priv->backForwardList = core(webView)->backForwardList();
    priv->backForwardList->setEnabled(TRUE);

    return webBackForwardList;
}

void webkit_web_back_forward_list_go_forward(WebKitWebBackForwardList* webBackForwardList)
{
    g_return_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList));

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);
    if (backForwardList->enabled())
        backForwardList->goForward();
}

void webkit_web_back_forward_list_go_back(WebKitWebBackForwardList* webBackForwardList)
{
    g_return_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList));

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);
    if (backForwardList->enabled())
        backForwardList->goBack();
}

gboolean webkit_web_back_forward_list_contains_item(WebKitWebBackForwardList* webBackForwardList, WebKitWebHistoryItem* webHistoryItem)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList), FALSE);
    g_return_val_if_fail(WEBKIT_IS_WEB_HISTORY_ITEM(webHistoryItem), FALSE);

    WebCore::HistoryItem* historyItem = core(webHistoryItem);

    g_return_val_if_fail(historyItem != NULL, FALSE);

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);

    return (backForwardList->enabled() ? backForwardList->containsItem(historyItem) : FALSE);
}

void webkit_web_back_forward_list_go_to_item(WebKitWebBackForwardList* webBackForwardList, WebKitWebHistoryItem* webHistoryItem)
{
    g_return_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList));
    g_return_if_fail(WEBKIT_IS_WEB_HISTORY_ITEM(webHistoryItem));

    WebCore::HistoryItem* historyItem = core(webHistoryItem);
    WebCore::BackForwardList* backForwardList = core(webBackForwardList);

    if (backForwardList->enabled() && historyItem)
        backForwardList->goToItem(historyItem);
}

GList* webkit_web_back_forward_list_get_forward_list_with_limit(WebKitWebBackForwardList* webBackForwardList, gint limit)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList), NULL);

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);
    if (!backForwardList || !backForwardList->enabled())
        return NULL;

    WebCore::HistoryItemVector items;
    GList* forwardItems = { 0 };

    backForwardList->forwardListWithLimit(limit, items);

    for (unsigned i = 0; i < items.size(); i++) {
        WebKitWebHistoryItem* webHistoryItem = kit(items[i]);
        forwardItems = g_list_prepend(forwardItems, webHistoryItem);
    }

    return forwardItems;
}

GList* webkit_web_back_forward_list_get_back_list_with_limit(WebKitWebBackForwardList* webBackForwardList, gint limit)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList), NULL);

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);
    if (!backForwardList || !backForwardList->enabled())
        return NULL;

    WebCore::HistoryItemVector items;
    GList* backItems = { 0 };

    backForwardList->backListWithLimit(limit, items);

    for (unsigned i = 0; i < items.size(); i++) {
        WebKitWebHistoryItem* webHistoryItem = kit(items[i]);
        backItems = g_list_prepend(backItems, webHistoryItem);
    }

    return backItems;
}

WebKitWebHistoryItem* webkit_web_back_forward_list_get_back_item(WebKitWebBackForwardList* webBackForwardList)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList), NULL);

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);
    if (!backForwardList || !backForwardList->enabled())
        return NULL;

    WebCore::HistoryItem* historyItem = backForwardList->backItem();

    return (historyItem ? kit(historyItem) : NULL);
}

WebKitWebHistoryItem* webkit_web_back_forward_list_get_current_item(WebKitWebBackForwardList* webBackForwardList)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList), NULL);

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);
    if (!backForwardList || !backForwardList->enabled())
        return NULL;

    WebCore::HistoryItem* historyItem = backForwardList->currentItem();

    return (historyItem ? kit(historyItem) : NULL);
}

WebKitWebHistoryItem* webkit_web_back_forward_list_get_forward_item(WebKitWebBackForwardList* webBackForwardList)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList), NULL);

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);
    if (!backForwardList || !backForwardList->enabled())
        return NULL;

    WebCore::HistoryItem* historyItem = backForwardList->forwardItem();

    return (historyItem ? kit(historyItem) : NULL);
}

WebKitWebHistoryItem* webkit_web_back_forward_list_get_nth_item(WebKitWebBackForwardList* webBackForwardList, gint index)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList), NULL);

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);
    if (!backForwardList)
        return NULL;

    WebCore::HistoryItem* historyItem = backForwardList->itemAtIndex(index);

    return (historyItem ? kit(historyItem) : NULL);
}

gint webkit_web_back_forward_list_get_back_length(WebKitWebBackForwardList* webBackForwardList)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList), 0);

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);
    if (!backForwardList || !backForwardList->enabled())
        return 0;

    return backForwardList->backListCount();
}

gint webkit_web_back_forward_list_get_forward_length(WebKitWebBackForwardList* webBackForwardList)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList), 0);

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);
    if (!backForwardList || !backForwardList->enabled())
        return 0;

    return backForwardList->forwardListCount();
}

gint webkit_web_back_forward_list_get_limit(WebKitWebBackForwardList* webBackForwardList)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList), 0);

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);
    if (!backForwardList || !backForwardList->enabled())
        return 0;

    return backForwardList->capacity();
}

void webkit_web_back_forward_list_set_limit(WebKitWebBackForwardList* webBackForwardList, gint limit)
{
    g_return_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList));

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);
    if (backForwardList)
        backForwardList->setCapacity(limit);
}

void webkit_web_back_forward_list_add_item(WebKitWebBackForwardList *webBackForwardList, WebKitWebHistoryItem *webHistoryItem)
{
    g_return_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList));

    g_object_ref(webHistoryItem);

    WebCore::BackForwardList* backForwardList = core(webBackForwardList);
    WebCore::HistoryItem* historyItem = core(webHistoryItem);

    backForwardList->addItem(historyItem);
}

WebCore::BackForwardList* WebKit::core(WebKitWebBackForwardList* webBackForwardList)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_BACK_FORWARD_LIST(webBackForwardList), NULL);

    return webBackForwardList->priv ? webBackForwardList->priv->backForwardList : 0;
}
