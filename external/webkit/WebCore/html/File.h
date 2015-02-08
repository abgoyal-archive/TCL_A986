

#ifndef File_h
#define File_h

#include "Blob.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class File : public Blob {
public:
    static PassRefPtr<File> create(const String& path)
    {
        return adoptRef(new File(path));
    }

    const String& name() const { return m_name; }

    // FIXME: obsolete attributes. To be removed.
    const String& fileName() const { return m_name; }
    unsigned long long fileSize() const { return size(); }

private:
    File(const String& path);

    String m_name;
};

} // namespace WebCore

#endif // FileList_h
