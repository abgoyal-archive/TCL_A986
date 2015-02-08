

#ifndef Blob_h
#define Blob_h

#include "ExceptionCode.h"
#include "PlatformString.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class Blob : public RefCounted<Blob> {
public:
    static PassRefPtr<Blob> create(const String& path)
    {
        return adoptRef(new Blob(path));
    }

    virtual ~Blob() { }

    const String& path() const { return m_path; }
    unsigned long long size() const;

protected:
    Blob(const String& path);

private:
    String m_path;
};

} // namespace WebCore

#endif // Blob_h
