

#include "TestsController.h"

#include "Test.h"
#include <wtf/PassOwnPtr.h>

using namespace std;

namespace WebKitAPITest {

static const LPCWSTR testsControllerWindowClassName = L"TestsControllerWindowClass";

enum { runNextTestTimerID = 1 };

inline TestsController::TestsController()
    : m_testFailed(false)
    , m_anyTestFailed(false)
{
    registerWindowClass();
    m_window = CreateWindowExW(0, testsControllerWindowClassName, 0, WS_CHILD, 0, 0, 0, 0, HWND_MESSAGE, 0, GetModuleHandle(0), 0);
}

TestsController& TestsController::shared()
{
    static TestsController& shared = *new TestsController;
    return shared;
}

bool TestsController::runAllTests()
{
    if (m_tests.isEmpty())
        return true;

    MSG msg;
    BOOL result;
    while ((result = GetMessage(&msg, 0, 0, 0))) {
        if (result == -1)
            break;
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    if (msg.message != WM_QUIT)
        return false;

    return !m_anyTestFailed;
}

void TestsController::addTest(PassOwnPtr<Test> test)
{
    m_tests.append(test.release());
    runNextTestSoon();
}

void TestsController::testFailed(const char* file, int line, const char* message)
{
    ASSERT(!m_tests.isEmpty());

    m_testFailed = true;
    m_anyTestFailed = true;

    printf("FAIL: %s\n\t%s (%s:%d)\n", m_tests.first()->name(), message, file, line);
    fflush(stdout);
}

void TestsController::runNextTest()
{
    if (m_tests.isEmpty()) {
        PostQuitMessage(0);
        return;
    }

    Test* test = m_tests.first();

    m_testFailed = false;
    printf("RUN: %s\n", test->name());
    fflush(stdout);
    test->run();

    if (!m_testFailed) {
        printf("PASS: %s\n", test->name());
        fflush(stdout);
    }

    m_tests.removeFirst();
    delete test;

    runNextTestSoon();
}

void TestsController::runNextTestSoon()
{
    SetTimer(m_window, runNextTestTimerID, 0, 0);
}

void TestsController::registerWindowClass()
{
    static bool initialized;
    if (initialized)
        return;
    initialized = true;

    WNDCLASSEXW wndClass = {0};
    wndClass.cbSize = sizeof(wndClass);
    wndClass.lpfnWndProc = wndProc;
    wndClass.hCursor = LoadCursor(0, IDC_ARROW);
    wndClass.hInstance = GetModuleHandle(0);
    wndClass.lpszClassName = testsControllerWindowClassName;

    RegisterClassExW(&wndClass);
}

LRESULT TestsController::wndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    if (uMsg == WM_TIMER && wParam == runNextTestTimerID) {
        KillTimer(hWnd, runNextTestTimerID);
        TestsController::shared().runNextTest();
        return 0;
    }

    return DefWindowProcW(hWnd, uMsg, wParam, lParam);
}

} // namespace WebKitAPITest
