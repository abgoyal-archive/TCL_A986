

#ifndef ChromiumThreading_h
#define ChromiumThreading_h

namespace WTF {

// An interface to the embedding layer, which provides threading support.
class ChromiumThreading {
public:
    static void initializeMainThread();
    static void scheduleDispatchFunctionsOnMainThread();
};

} // namespace WTF

#endif // ChromiumThreading_h
