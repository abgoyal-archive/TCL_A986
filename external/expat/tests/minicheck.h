
#ifdef __cplusplus
extern "C" {
#endif

#define CK_NOFORK 0
#define CK_FORK   1

#define CK_SILENT  0
#define CK_NORMAL  1
#define CK_VERBOSE 2

#if defined(__osf__) && defined(__cplusplus)
#define __func__ __FUNCTION__
#endif

#define START_TEST(testname) static void testname(void) { \
    _check_set_test_info(__func__, __FILE__, __LINE__);   \
    {
#define END_TEST } }

#define fail(msg)  _fail_unless(0, __FILE__, __LINE__, msg)

typedef void (*tcase_setup_function)(void);
typedef void (*tcase_teardown_function)(void);
typedef void (*tcase_test_function)(void);

typedef struct SRunner SRunner;
typedef struct Suite Suite;
typedef struct TCase TCase;

struct SRunner {
    Suite *suite;
    int nchecks;
    int nfailures;
};

struct Suite {
    char *name;
    TCase *tests;
};

struct TCase {
    char *name;
    tcase_setup_function setup;
    tcase_teardown_function teardown;
    tcase_test_function *tests;
    int ntests;
    int allocated;
    TCase *next_tcase;
};


/* Internal helper. */
void _check_set_test_info(char const *function,
                          char const *filename, int lineno);



void _fail_unless(int condition, const char *file, int line, char *msg);
Suite *suite_create(char *name);
TCase *tcase_create(char *name);
void suite_add_tcase(Suite *suite, TCase *tc);
void tcase_add_checked_fixture(TCase *,
                               tcase_setup_function,
                               tcase_teardown_function);
void tcase_add_test(TCase *tc, tcase_test_function test);
SRunner *srunner_create(Suite *suite);
void srunner_run_all(SRunner *runner, int verbosity);
int srunner_ntests_failed(SRunner *runner);
void srunner_free(SRunner *runner);

#ifdef __cplusplus
}
#endif
