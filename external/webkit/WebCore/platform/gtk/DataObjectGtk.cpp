

#include "config.h"
#include "DataObjectGtk.h"

#include "markup.h"
#include <gtk/gtk.h>

namespace WebCore {

String DataObjectGtk::text()
{
    if (m_range)
        return m_range->text();
    return m_text;
}

String DataObjectGtk::markup()
{
    if (m_range)
        createMarkup(m_range.get(), 0, AnnotateForInterchange);
    return m_markup;
}

void DataObjectGtk::setText(const String& newText)
{
    m_range = 0;
    m_text = newText;
}

void DataObjectGtk::setMarkup(const String& newMarkup)
{
    m_range = 0;
    m_markup = newMarkup;
}

Vector<String> DataObjectGtk::files()
{
    Vector<KURL> uris(uriList());
    Vector<String> files;

    for (size_t i = 0; i < uris.size(); i++) {
        KURL& uri = uris[0];
        if (!uri.isValid() || !uri.isLocalFile())
            continue;

        files.append(uri.string());
    }

    return files;
}

String DataObjectGtk::url()
{
    Vector<KURL> uris(uriList());
    for (size_t i = 0; i < uris.size(); i++) {
        KURL& uri = uris[0];
        if (uri.isValid())
            return uri;
    }

    return String();
}

String DataObjectGtk::urlLabel()
{
    if (hasText())
        return text();

    if (hasURL())
        return url();

    return String();
}

bool DataObjectGtk::hasURL()
{
    return !url().isEmpty();
}

void DataObjectGtk::clear()
{
    m_text = "";
    m_markup = "";
    m_uriList.clear();
    m_image = 0;
    m_range = 0;
}

DataObjectGtk* DataObjectGtk::forClipboard(GtkClipboard* clipboard)
{
    static HashMap<GtkClipboard*, RefPtr<DataObjectGtk> > objectMap;

    if (!objectMap.contains(clipboard)) {
        RefPtr<DataObjectGtk> dataObject = DataObjectGtk::create();
        objectMap.set(clipboard, dataObject);
        return dataObject.get();
    }

    HashMap<GtkClipboard*, RefPtr<DataObjectGtk> >::iterator it = objectMap.find(clipboard);
    return it->second.get();
}

}
