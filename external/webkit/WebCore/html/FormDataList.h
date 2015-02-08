

#ifndef FormDataList_h
#define FormDataList_h

#include "CString.h"
#include "File.h"
#include "TextEncoding.h"

namespace WebCore {

class FormDataList {
public:
    FormDataList(const TextEncoding&);

    void appendData(const String& key, const String& value)
        { appendString(key); appendString(value); }
    void appendData(const String& key, const CString& value)
        { appendString(key); appendString(value); }
    void appendData(const String& key, int value)
        { appendString(key); appendString(String::number(value)); }
    void appendFile(const String& key, PassRefPtr<File> file)
        { appendString(key); m_list.append(file); }

    class Item {
    public:
        Item() { }
        Item(const CString& data) : m_data(data) { }
        Item(PassRefPtr<File> file) : m_file(file) { }

        const CString& data() const { return m_data; }
        File* file() const { return m_file.get(); }

    private:
        CString m_data;
        RefPtr<File> m_file;
    };

    const Vector<Item>& list() const { return m_list; }

private:
    void appendString(const CString&);
    void appendString(const String&);

    TextEncoding m_encoding;
    Vector<Item> m_list;
};

} // namespace WebCore

#endif // FormDataList_h
