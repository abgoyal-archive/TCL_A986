
#include <stdio.h>
#include <assert.h>

#define LOG_TAG "Quake"

#include <GLES/gl.h>
#include <utils/Log.h>
#include <utils/Timers.h>

#include <quakedef.h>

// Timer utilities

#define ENABLE_PMP
#define USE_LOG

#ifdef ENABLE_PMP

static nsecs_t baseTime;
static nsecs_t lastTime;
static const unsigned int kStartTimeStackSize = 100;
static nsecs_t startTimes[kStartTimeStackSize];
static unsigned int startTimeStackPointer;

static
void PMP_Common(const char* fmt, va_list ap, char type)
{
	char buf[1024];
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	
	// Note: Timer acually has less than microsecond resolution, so track time in microseconds:
	
	nsecs_t time = systemTime(SYSTEM_TIME_THREAD) / 1000;
	if(baseTime == 0)
	{
		baseTime = time;
	}
	time -= baseTime;
	switch(type)
	{
	case '<':
		{
			if(startTimeStackPointer < kStartTimeStackSize)
			{
				startTimes[startTimeStackPointer] = time;
			}
#ifdef USE_LOG
			LOGI("< %lld [%d] %s\n", time, startTimeStackPointer, buf);
#else
			fprintf(stderr, "Quake < %lld %d %s\n", time, startTimeStackPointer, buf);
#endif
			startTimeStackPointer++;
		}
		break;
	case '>':
		{
			nsecs_t elapsed = 0;
			if(startTimeStackPointer > 0)
			{
				--startTimeStackPointer;
				if(startTimeStackPointer < kStartTimeStackSize)
				{
					elapsed = time - startTimes[startTimeStackPointer];
				}
			}
#ifdef USE_LOG
			LOGI("> %lld [%d] %lld %s\n", time, startTimeStackPointer, elapsed, buf);
#else
			fprintf(stderr, "Quake > %lld [%d] %lld %s\n", time, startTimeStackPointer, elapsed, buf);
#endif
		}
		break;
	default:
#ifdef USE_LOG
		LOGI("= %lld %lld %s\n", time, time - lastTime, buf);
#else
		fprintf(stderr, "Quake = %lld %s\n", time, buf);
#endif
		break;
	}
	lastTime = time;
}

void PMP_Begin(const char* fmt,...)
{
	va_list ap;
	va_start(ap, fmt);
	PMP_Common(fmt, ap, '<');
}

void PMP_Event(const char* fmt,...)
{
	va_list ap;
	va_start(ap, fmt);
	PMP_Common(fmt, ap, '=');
}

void PMP_End(const char* fmt,...)
{
	va_list ap;
	va_start(ap, fmt);
	PMP_Common(fmt, ap, '>');
}

#endif // ENABLE_PMP
