

#ifndef ChromiumDataObject_h
#define ChromiumDataObject_h

#include "KURL.h"
#include "PlatformString.h"
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

    // A data object for holding data that would be in a clipboard or moved
    // during a drag-n-drop operation.  This is the data that WebCore is aware
    // of and is not specific to a platform.
    class ChromiumDataObject : public RefCounted<ChromiumDataObject> {
    public:
        static PassRefPtr<ChromiumDataObject> create()
        {
            return adoptRef(new ChromiumDataObject);
        }

        PassRefPtr<ChromiumDataObject> copy() const
        {
            return adoptRef(new ChromiumDataObject(*this));
        }

        void clear();
        bool hasData() const;

        KURL url;
        String urlTitle;

        KURL downloadURL;
        String downloadMetadata;

        String fileExtension;
        Vector<String> filenames;

        String plainText;

        String textHtml;
        KURL htmlBaseUrl;

        String fileContentFilename;
        RefPtr<SharedBuffer> fileContent;

    private:
        ChromiumDataObject() {}
        ChromiumDataObject(const ChromiumDataObject&);
    };

} // namespace WebCore

#endif
