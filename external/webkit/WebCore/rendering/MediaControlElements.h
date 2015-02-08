

#ifndef MediaControlElements_h
#define MediaControlElements_h

#if ENABLE(VIDEO)

#include "HTMLDivElement.h"
#include "HTMLInputElement.h"
#include "HTMLMediaElement.h"
#include "RenderBlock.h"

// These are the shadow elements used in RenderMedia

namespace WebCore {

class Event;
class Frame;

// Must match WebKitSystemInterface.h
enum MediaControlElementType {
    MediaFullscreenButton = 0,
    MediaMuteButton,
    MediaPlayButton,
    MediaSeekBackButton,
    MediaSeekForwardButton,
    MediaSlider,
    MediaSliderThumb,
    MediaRewindButton,
    MediaReturnToRealtimeButton,
    MediaShowClosedCaptionsButton,
    MediaHideClosedCaptionsButton,
    MediaUnMuteButton,
    MediaPauseButton,
    MediaTimelineContainer,
    MediaCurrentTimeDisplay,
    MediaTimeRemainingDisplay,
    MediaStatusDisplay,
    MediaControlsPanel,
    MediaVolumeSliderContainer,
    MediaVolumeSlider,
    MediaVolumeSliderThumb
};

HTMLMediaElement* toParentMediaElement(RenderObject*);

class MediaControlShadowRootElement : public HTMLDivElement {
public:
    MediaControlShadowRootElement(Document*, HTMLMediaElement*);
    
    virtual bool isShadowNode() const { return true; }
    virtual Node* shadowParentNode() { return m_mediaElement; }

    void updateStyle();
    
private:
    HTMLMediaElement* m_mediaElement;    
};

// ----------------------------

class MediaControlElement : public HTMLDivElement {
public:
    MediaControlElement(Document*, PseudoId, HTMLMediaElement*);
    virtual void attach();
    virtual bool rendererIsNeeded(RenderStyle*);

    virtual PassRefPtr<RenderStyle> styleForElement();
    void attachToParent(Element*);
    void update();
    virtual void updateStyle();

    MediaControlElementType displayType() const { return m_displayType; }

    HTMLMediaElement* mediaElement() const { return m_mediaElement; }
    virtual bool isMediaControlElement() const { return true; }

protected:
    HTMLMediaElement* m_mediaElement;   
    PseudoId m_pseudoStyleId;
    MediaControlElementType m_displayType;  // some elements can show multiple types (e.g. play/pause)
};

// ----------------------------

class MediaControlTimelineContainerElement : public MediaControlElement {
public:
    MediaControlTimelineContainerElement(Document*, HTMLMediaElement*);
    virtual bool rendererIsNeeded(RenderStyle*);
};

// ----------------------------

class MediaControlVolumeSliderContainerElement : public MediaControlElement {
public:
    MediaControlVolumeSliderContainerElement(Document*, HTMLMediaElement*);
    virtual PassRefPtr<RenderStyle> styleForElement();
    void setVisible(bool);
    bool isVisible() { return m_isVisible; }
    void setPosition(int x, int y);
    bool hitTest(const IntPoint& absPoint);

private:
    bool m_isVisible;
    int m_x, m_y;
};

// ----------------------------

class MediaControlStatusDisplayElement : public MediaControlElement {
public:
    MediaControlStatusDisplayElement(Document*, HTMLMediaElement*);
    virtual void update();
    virtual bool rendererIsNeeded(RenderStyle*);
private:
    enum StateBeingDisplayed { Nothing, Loading, LiveBroadcast };
    StateBeingDisplayed m_stateBeingDisplayed;
};

// ----------------------------

class MediaControlInputElement : public HTMLInputElement {
public:
    MediaControlInputElement(Document*, PseudoId, const String& type, HTMLMediaElement*);
    virtual void attach();
    virtual bool rendererIsNeeded(RenderStyle*);

    virtual PassRefPtr<RenderStyle> styleForElement();
    void attachToParent(Element*);
    void update();
    void updateStyle();

    bool hitTest(const IntPoint& absPoint);
    MediaControlElementType displayType() const { return m_displayType; }

    HTMLMediaElement* mediaElement() const { return m_mediaElement; }
    virtual bool isMediaControlElement() const { return true; }

protected:
    virtual void updateDisplayType() { }
    void setDisplayType(MediaControlElementType);

    HTMLMediaElement* m_mediaElement;   
    PseudoId m_pseudoStyleId;
    MediaControlElementType m_displayType;
};

// ----------------------------

class MediaControlMuteButtonElement : public MediaControlInputElement {
public:
    MediaControlMuteButtonElement(Document*, HTMLMediaElement*);
    virtual void defaultEventHandler(Event*);
    virtual void updateDisplayType();
};

// ----------------------------

class MediaControlPlayButtonElement : public MediaControlInputElement {
public:
    MediaControlPlayButtonElement(Document*, HTMLMediaElement*);
    virtual void defaultEventHandler(Event*);
    virtual void updateDisplayType();
};

// ----------------------------

class MediaControlSeekButtonElement : public MediaControlInputElement {
public:
    MediaControlSeekButtonElement(Document*, HTMLMediaElement*, bool forward);
    virtual void defaultEventHandler(Event*);
    virtual void detach();
    void seekTimerFired(Timer<MediaControlSeekButtonElement>*);

private:
    bool m_forward;
    bool m_seeking;
    bool m_capturing;
    Timer<MediaControlSeekButtonElement> m_seekTimer;
};
    
// ----------------------------

class MediaControlRewindButtonElement : public MediaControlInputElement {
public:
    MediaControlRewindButtonElement(Document*, HTMLMediaElement*);
    virtual void defaultEventHandler(Event*);
};

// ----------------------------

class MediaControlReturnToRealtimeButtonElement : public MediaControlInputElement {
public:
    MediaControlReturnToRealtimeButtonElement(Document*, HTMLMediaElement*);
    virtual void defaultEventHandler(Event*);
};    

// ----------------------------

class MediaControlToggleClosedCaptionsButtonElement : public MediaControlInputElement {
public:
    MediaControlToggleClosedCaptionsButtonElement(Document*, HTMLMediaElement*);
    virtual void defaultEventHandler(Event*);
    virtual void updateDisplayType();
};    

// ----------------------------

class MediaControlTimelineElement : public MediaControlInputElement {
public:
    MediaControlTimelineElement(Document*, HTMLMediaElement*);
    virtual void defaultEventHandler(Event*);
    void update(bool updateDuration = true);
};

// ----------------------------

class MediaControlVolumeSliderElement : public MediaControlInputElement {
public:
    MediaControlVolumeSliderElement(Document*, HTMLMediaElement*);
    virtual void defaultEventHandler(Event*);
    virtual void update();
};

// ----------------------------

class MediaControlFullscreenButtonElement : public MediaControlInputElement {
public:
    MediaControlFullscreenButtonElement(Document*, HTMLMediaElement*);
    virtual void defaultEventHandler(Event*);
};

// ----------------------------

class MediaControlTimeDisplayElement : public MediaControlElement {
public:
    MediaControlTimeDisplayElement(Document*, PseudoId, HTMLMediaElement*);
    void setVisible(bool);
    virtual PassRefPtr<RenderStyle> styleForElement();

    void setCurrentValue(float);
    float currentValue() const { return m_currentValue; }

private:
    float m_currentValue;
    bool m_isVisible;
};

// ----------------------------

class RenderMediaControlShadowRoot : public RenderBlock {
public:
    RenderMediaControlShadowRoot(Element* e) : RenderBlock(e) { }
    void setParent(RenderObject* p) { RenderObject::setParent(p); }
};
    
// ----------------------------


} //namespace WebCore
#endif // enable(video)
#endif // MediaControlElements_h
