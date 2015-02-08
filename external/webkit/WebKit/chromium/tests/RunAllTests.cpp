

// FIXME: Avoid this source dependency on Chromium's base module.
#include <base/test/test_suite.h>

#include "WebKit.h"
#include "WebKitClient.h"

// WebKitClient has a protected destructor, so we need to subclass.
class DummyWebKitClient : public WebKit::WebKitClient {
};

int main(int argc, char** argv)
{
    DummyWebKitClient dummyClient;
    WebKit::initialize(&dummyClient);

    int result = TestSuite(argc, argv).Run();

    WebKit::shutdown();
    return result;
}
