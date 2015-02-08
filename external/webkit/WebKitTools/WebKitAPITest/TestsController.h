

#ifndef TestsController_h
#define TestsController_h

#include <windows.h>
#include <wtf/Forward.h>
#include <wtf/Deque.h>
#include <wtf/Noncopyable.h>

namespace WebKitAPITest {

class Test;

class TestsController : public Noncopyable {
public:
    static TestsController& shared();

    // Returns true if all the tests passed, false otherwise.
    bool runAllTests();

    void addTest(PassOwnPtr<Test>);
    void testFailed(const char* file, int line, const char* message);

private:
    TestsController();
    ~TestsController();

    void runNextTest();
    void runNextTestSoon();

    static void registerWindowClass();
    static LRESULT CALLBACK wndProc(HWND, UINT uMsg, WPARAM, LPARAM);

    HWND m_window;
    Deque<Test*> m_tests;
    bool m_testFailed;
    bool m_anyTestFailed;
};

} // namespace WebKitAPITest

#endif // TestsController_h
