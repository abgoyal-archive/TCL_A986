

#ifndef QTMovieWin_h
#define QTMovieWin_h

#include <Unicode.h>
#include <windows.h>

#ifdef QTMOVIEWIN_EXPORTS
#define QTMOVIEWIN_API __declspec(dllexport)
#else
#define QTMOVIEWIN_API __declspec(dllimport)
#endif

class QTMovieWin;
class QTMovieWinPrivate;

class QTMovieWinClient {
public:
    virtual void movieEnded(QTMovieWin*) = 0;
    virtual void movieLoadStateChanged(QTMovieWin*) = 0;
    virtual void movieTimeChanged(QTMovieWin*) = 0;
    virtual void movieNewImageAvailable(QTMovieWin*) = 0;
};

class QTMovieWinFullscreenClient {
public:
    virtual LRESULT fullscreenClientWndProc(HWND, UINT message, WPARAM, LPARAM) = 0;
};

enum {
    QTMovieLoadStateError = -1L,
    QTMovieLoadStateLoaded  = 2000L,
    QTMovieLoadStatePlayable = 10000L,
    QTMovieLoadStatePlaythroughOK = 20000L,
    QTMovieLoadStateComplete = 100000L
};

typedef const struct __CFURL * CFURLRef;

class QTMOVIEWIN_API QTMovieWin {
public:
    static bool initializeQuickTime();

    typedef void (*SetTaskTimerDelayFunc)(double);
    typedef void (*StopTaskTimerFunc)();
    static void setTaskTimerFuncs(SetTaskTimerDelayFunc, StopTaskTimerFunc);
    static void taskTimerFired();

    QTMovieWin(QTMovieWinClient*);
    ~QTMovieWin();

    void load(const UChar* url, int len, bool preservesPitch);
    long loadState() const;
    float maxTimeLoaded() const;

    void play();
    void pause();

    float rate() const;
    void setRate(float);

    float duration() const;
    float currentTime() const;
    void setCurrentTime(float) const;

    void setVolume(float);
    void setPreservesPitch(bool);

    unsigned dataSize() const;

    void getNaturalSize(int& width, int& height);
    void setSize(int width, int height);

    void setVisible(bool);
    void paint(HDC, int x, int y);
    void getCurrentFrameInfo(void*& buffer, unsigned& bitsPerPixel, unsigned& rowBytes, unsigned& width, unsigned& height);

    void disableUnsupportedTracks(unsigned& enabledTrackCount, unsigned& totalTrackCount);
    void setDisabled(bool);

    bool hasVideo() const;
    bool hasAudio() const;

    bool hasClosedCaptions() const;
    void setClosedCaptionsVisible(bool);

    static unsigned countSupportedTypes();
    static void getSupportedType(unsigned index, const UChar*& str, unsigned& len);

    // Returns the full-screen window created
    HWND enterFullscreen(QTMovieWinFullscreenClient*);
    void exitFullscreen();

private:
    void load(CFURLRef, bool preservesPitch);
    static LRESULT fullscreenWndProc(HWND, UINT message, WPARAM, LPARAM);

    QTMovieWinPrivate* m_private;
    bool m_disabled;
    friend class QTMovieWinPrivate;
};

#endif
