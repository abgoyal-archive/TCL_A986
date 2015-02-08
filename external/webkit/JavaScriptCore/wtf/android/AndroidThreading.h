

#ifndef AndroidThreading_h
#define AndroidThreading_h

namespace WTF {

// An interface to the embedding layer, which provides threading support.
class AndroidThreading {
public:
    static void scheduleDispatchFunctionsOnMainThread();
};

} // namespace WTF

#endif // AndroidThreading_h
