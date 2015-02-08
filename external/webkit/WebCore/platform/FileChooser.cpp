

#include "config.h"
#include "FileChooser.h"

#include "Icon.h"

namespace WebCore {
    
FileChooserClient::~FileChooserClient()
{
}

inline FileChooser::FileChooser(FileChooserClient* client, const Vector<String>& initialFilenames)
    : m_client(client)
    , m_icon(Icon::createIconForFiles(initialFilenames))
{
    m_filenames = initialFilenames;
}

PassRefPtr<FileChooser> FileChooser::create(FileChooserClient* client, const Vector<String>& initialFilenames)
{
    return adoptRef(new FileChooser(client, initialFilenames));
}

FileChooser::~FileChooser()
{
}

void FileChooser::clear()
{
    m_filenames.clear();
    m_icon = 0;
}

void FileChooser::chooseFile(const String& filename)
{
    Vector<String> filenames;
    filenames.append(filename);
    chooseFiles(filenames);
}

void FileChooser::chooseFiles(const Vector<String>& filenames)
{
    if (m_filenames == filenames)
        return;
    m_filenames = filenames;
    m_icon = Icon::createIconForFiles(filenames);
    if (m_client)
        m_client->valueChanged();
}

}
