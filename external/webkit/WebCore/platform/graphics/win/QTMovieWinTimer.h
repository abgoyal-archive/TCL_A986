

#ifndef QTMovieViewTimer_h
#define QTMovieViewTimer_h

// This header should not be included from WebCore.
// It is used by the QuickTime access DLL. It copies some WebCore code 
// which can't be used directly due to dependency issues.

void setSharedTimerFiredFunction(void (*)());
void setSharedTimerFireDelay(double delay);
void stopSharedTimer();
void setSharedTimerInstanceHandle(HINSTANCE handle);
double systemTime();

#endif
