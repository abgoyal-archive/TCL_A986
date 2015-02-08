

#ifndef ThreadableLoaderClient_h
#define ThreadableLoaderClient_h

namespace WebCore {

    class ResourceError;
    class ResourceResponse;

    class ThreadableLoaderClient : public Noncopyable {
    public:
        virtual void didSendData(unsigned long long /*bytesSent*/, unsigned long long /*totalBytesToBeSent*/) { }

        virtual void didReceiveResponse(const ResourceResponse&) { }
        virtual void didReceiveData(const char*, int /*lengthReceived*/) { }
        virtual void didFinishLoading(unsigned long /*identifier*/) { }
        virtual void didFail(const ResourceError&) { }
        virtual void didFailRedirectCheck() { }

        virtual void didReceiveAuthenticationCancellation(const ResourceResponse&) { }

    protected:
        virtual ~ThreadableLoaderClient() { }
    };

} // namespace WebCore

#endif // ThreadableLoaderClient_h
