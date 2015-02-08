

#ifndef LoaderRunLoopCF_h
#define LoaderRunLoopCF_h

#if !PLATFORM(WIN)
#error This code is not needed on platforms other than Windows, because main thread's CFRunLoop can be used.
#endif

namespace WebCore {

CFRunLoopRef loaderRunLoop();

}

#endif // LoaderRunLoop_h
