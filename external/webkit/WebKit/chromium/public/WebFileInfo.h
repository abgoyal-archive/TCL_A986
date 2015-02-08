

#ifndef WebFileInfo_h
#define WebFileInfo_h

namespace WebKit {

struct WebFileInfo {
    // The last modification time of the file, in seconds.
    // The value 0.0 means that the time is not set.
    double modificationTime;

    WebFileInfo() : modificationTime(0.0) { }
};

} // namespace WebKit

#endif
