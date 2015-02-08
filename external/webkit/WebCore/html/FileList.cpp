

#include "config.h"
#include "FileList.h"

#include "File.h"

namespace WebCore {

FileList::FileList()
{
}

File* FileList::item(unsigned index) const
{
    if (index >= m_files.size())
        return 0;
    return m_files[index].get();
}

} // namespace WebCore
