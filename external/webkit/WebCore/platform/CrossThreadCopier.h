

#ifndef CrossThreadCopier_h
#define CrossThreadCopier_h

#include <memory>
#include <wtf/PassOwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/Threading.h>
#include <wtf/TypeTraits.h>

namespace WebCore {

    class KURL;
    class ResourceError;
    class ResourceRequest;
    class ResourceResponse;
    class String;
    struct CrossThreadResourceResponseData;
    struct CrossThreadResourceRequestData;
    struct ThreadableLoaderOptions;

    template<typename T> struct CrossThreadCopierPassThrough {
        typedef T Type;
        static Type copy(const T& parameter)
        {
            return parameter;
        }
    };

    template<bool isConvertibleToInteger, bool isThreadsafeShared, typename T> struct CrossThreadCopierBase;

    // Integers get passed through without any changes.
    template<typename T> struct CrossThreadCopierBase<true, false, T> : public CrossThreadCopierPassThrough<T> {
    };

    // Pointers get passed through without any significant changes.
    template<typename T> struct CrossThreadCopierBase<false, false, T*> : public CrossThreadCopierPassThrough<T*> {
    };

    template<> struct CrossThreadCopierBase<false, false, ThreadableLoaderOptions> : public CrossThreadCopierPassThrough<ThreadableLoaderOptions> {
    };

    // Custom copy methods.
    template<typename T> struct CrossThreadCopierBase<false, true, T> {
        typedef typename WTF::RemoveTemplate<T, RefPtr>::Type RefCountedType;
        typedef PassRefPtr<RefCountedType> Type;
        static Type copy(const T& refPtr)
        {
            return refPtr.get();
        }
    };

    template<typename T> struct CrossThreadCopierBase<false, false, PassOwnPtr<T> > {
        typedef PassOwnPtr<T> Type;
        static Type copy(const PassOwnPtr<T>& ownPtr)
        {
            return PassOwnPtr<T>(static_cast<T*>(ownPtr.release()));
        }
    };

    template<typename T> struct CrossThreadCopierBase<false, false, std::auto_ptr<T> > {
        typedef std::auto_ptr<T> Type;
        static Type copy(const std::auto_ptr<T>& autoPtr)
        {
            return std::auto_ptr<T>(*const_cast<std::auto_ptr<T>*>(&autoPtr));
        }
    };

    template<> struct CrossThreadCopierBase<false, false, KURL> {
        typedef KURL Type;
        static Type copy(const KURL&);
    };

    template<> struct CrossThreadCopierBase<false, false, String> {
        typedef String Type;
        static Type copy(const String&);
    };

    template<> struct CrossThreadCopierBase<false, false, ResourceError> {
        typedef ResourceError Type;
        static Type copy(const ResourceError&);
    };

    template<> struct CrossThreadCopierBase<false, false, ResourceRequest> {
        typedef std::auto_ptr<CrossThreadResourceRequestData> Type;
        static Type copy(const ResourceRequest&);
    };

    template<> struct CrossThreadCopierBase<false, false, ResourceResponse> {
        typedef std::auto_ptr<CrossThreadResourceResponseData> Type;
        static Type copy(const ResourceResponse&);
    };

    template<typename T> struct CrossThreadCopier : public CrossThreadCopierBase<WTF::IsConvertibleToInteger<T>::value,
                                                                                 WTF::IsSubclassOfTemplate<typename WTF::RemoveTemplate<T, RefPtr>::Type, ThreadSafeShared>::value,
                                                                                 T> {
    };

} // namespace WebCore

#endif // CrossThreadCopier_h
