

#ifndef FileList_h
#define FileList_h

#include "File.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

    class FileList : public RefCounted<FileList> {
    public:
        static PassRefPtr<FileList> create()
        {
            return adoptRef(new FileList);
        }

        unsigned length() const { return m_files.size(); }
        File* item(unsigned index) const;

        bool isEmpty() const { return m_files.isEmpty(); }
        void clear() { m_files.clear(); }
        void append(PassRefPtr<File> file) { m_files.append(file); }

    private:
        FileList();

        Vector<RefPtr<File> > m_files;
    };

} // namespace WebCore

#endif // FileList_h
