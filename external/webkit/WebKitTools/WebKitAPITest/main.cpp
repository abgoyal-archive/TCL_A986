

#include "TestsController.h"

using namespace WebKitAPITest;

int main(int, char*[])
{
    // FIXME: Remove this line once <http://webkit.org/b/32867> is fixed.
    OleInitialize(0);

    return !TestsController::shared().runAllTests();
}
