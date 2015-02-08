

#ifndef _INTLTESTDATADRIVENCALENDAR
#define _INTLTESTDATADRIVENCALENDAR

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "tsdate.h"
#include "uvector.h"
#include "unicode/calendar.h"
#include "fldset.h"

class TestDataModule;
class TestData;
class DataMap;
class CalendarFieldsSet;

class DataDrivenCalendarTest : public IntlTest {
	void runIndexedTest(int32_t index, UBool exec, const char* &name,
			char* par = NULL);
public:
	DataDrivenCalendarTest();
	virtual ~DataDrivenCalendarTest();
protected:

	void DataDrivenTest(char *par);
	void processTest(TestData *testData);
private:
	void testConvert(TestData *testData, const DataMap *settings, UBool fwd);
	void testOps(TestData *testData, const DataMap *settings);
	void testConvert(int32_t n, const CalendarFieldsSet &fromSet,
			Calendar *fromCal, const CalendarFieldsSet &toSet, Calendar *toCal,
			UBool fwd);
private:
	TestDataModule *driver;
};

#endif /* #if !UCONFIG_NO_COLLATION */

#endif
