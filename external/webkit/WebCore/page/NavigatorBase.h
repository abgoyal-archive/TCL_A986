

#ifndef NavigatorBase_h
#define NavigatorBase_h

namespace WebCore {

    class String;

    class NavigatorBase {
    public:
        String appName() const;
        String appVersion() const;
        virtual String userAgent() const = 0;
        String platform() const;

        String appCodeName() const;
        String product() const;
        String productSub() const;
        String vendor() const;
        String vendorSub() const;

        bool onLine() const;

    protected:
        virtual ~NavigatorBase();
    };

} // namespace WebCore

#endif // NavigatorBase_h
