

#ifndef Test_h
#define Test_h

#include "TestsController.h"

namespace WebKitAPITest {

// Abstract base class that all tests inherit from.
class Test {
public:
    ~Test() { }
    virtual const char* name() const = 0;
    virtual void run() = 0;
};

#define TEST_CLASS_NAME(testCaseName, testName) testCaseName##_##testName##_Test

// Use this to define a new test.
#define TEST(testCaseName, testName) \
    class TEST_CLASS_NAME(testCaseName, testName) : public Test { \
    public: \
        virtual const char* name() const { return #testCaseName ": " #testName; } \
        virtual void run(); \
        static const bool initialized; \
    }; \
    \
    const bool TEST_CLASS_NAME(testCaseName, testName)::initialized = (TestsController::shared().addTest(new TEST_CLASS_NAME(testCaseName, testName)), true); \
    \
    void TEST_CLASS_NAME(testCaseName, testName)::run()

#define TEST_ASSERT(expression) do { if (!(expression)) { TestsController::shared().testFailed(__FILE__, __LINE__, #expression); return; } } while (0)

} // namespace WebKitAPITest

#endif // Test_h
