

#ifndef FileChooser_h
#define FileChooser_h

#include "PlatformString.h"
#include <wtf/Vector.h>

namespace WebCore {

class Font;
class Icon;

class FileChooserClient {
public:
    virtual void valueChanged() = 0;
    virtual bool allowsMultipleFiles() = 0;
    virtual String acceptTypes() = 0;
    virtual ~FileChooserClient();
};

class FileChooser : public RefCounted<FileChooser> {
public:
    static PassRefPtr<FileChooser> create(FileChooserClient*, const Vector<String>& initialFilenames);
    ~FileChooser();

    void disconnectClient() { m_client = 0; }
    bool disconnected() { return !m_client; }

    const Vector<String>& filenames() const { return m_filenames; }
    String basenameForWidth(const Font&, int width) const;

    Icon* icon() const { return m_icon.get(); }

    void clear(); // for use by client; does not call valueChanged

    void chooseFile(const String& path);
    void chooseFiles(const Vector<String>& paths);
    
    bool allowsMultipleFiles() const { return m_client ? m_client->allowsMultipleFiles() : false; }
    // Acceptable MIME types.  It's an 'accept' attribute value of the corresponding INPUT element.
    String acceptTypes() const { return m_client ? m_client->acceptTypes() : String(); }

private:
    FileChooser(FileChooserClient*, const Vector<String>& initialFilenames);

    FileChooserClient* m_client;
    Vector<String> m_filenames;
    RefPtr<Icon> m_icon;
};

} // namespace WebCore

#endif // FileChooser_h
