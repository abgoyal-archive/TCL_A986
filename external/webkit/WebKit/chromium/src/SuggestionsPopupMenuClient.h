

#include "PopupMenuClient.h"

#ifndef SuggestionsPopupMenuClient_h
#define SuggestionsPopupMenuClient_h

namespace WebCore {
class HTMLInputElement;
class PopupMenuStyle;
class RenderStyle;
}

namespace WebKit {
class WebString;
class WebViewImpl;
template <typename T> class WebVector;

// The Suggestions popup menu client, used to display a list of suggestions.
class SuggestionsPopupMenuClient : public WebCore::PopupMenuClient {
public:
    SuggestionsPopupMenuClient();
    virtual ~SuggestionsPopupMenuClient();

    // Returns the number of suggestions available.
    virtual unsigned getSuggestionsCount() const = 0;

    // Returns the suggestion at |listIndex|.
    virtual WebString getSuggestion(unsigned listIndex) const = 0;

    // Removes the suggestion at |listIndex| from the list of suggestions.
    virtual void removeSuggestionAtIndex(unsigned listIndex) = 0;

    // WebCore::PopupMenuClient methods:
    virtual void valueChanged(unsigned listIndex, bool fireEvents = true);
    virtual WebCore::String itemText(unsigned listIndex) const;
    virtual WebCore::String itemToolTip(unsigned lastIndex) const { return WebCore::String(); }
    virtual bool itemIsEnabled(unsigned listIndex) const { return true; }
    virtual WebCore::PopupMenuStyle itemStyle(unsigned listIndex) const;
    virtual WebCore::PopupMenuStyle menuStyle() const;
    virtual int clientInsetLeft() const { return 0; }
    virtual int clientInsetRight() const { return 0; }
    virtual int clientPaddingLeft() const;
    virtual int clientPaddingRight() const;
    virtual int listSize() const { return getSuggestionsCount(); }
    virtual int selectedIndex() const { return m_selectedIndex; }
    virtual void popupDidHide();
    virtual bool itemIsSeparator(unsigned listIndex) const { return false; }
    virtual bool itemIsLabel(unsigned listIndex) const { return false; }
    virtual bool itemIsSelected(unsigned listIndex) const { return false; }
    virtual bool shouldPopOver() const { return false; }
    virtual bool valueShouldChangeOnHotTrack() const { return false; }
    virtual void setTextFromItem(unsigned listIndex);
    virtual WebCore::FontSelector* fontSelector() const;
    virtual WebCore::HostWindow* hostWindow() const;
    virtual PassRefPtr<WebCore::Scrollbar> createScrollbar(
        WebCore::ScrollbarClient* client,
        WebCore::ScrollbarOrientation orientation,
        WebCore::ScrollbarControlSize size);

protected:
    void initialize(WebCore::HTMLInputElement* textField,
                    int defaultSuggestionIndex);

    int getSelectedIndex() const { return m_selectedIndex; }
    void setSelectedIndex(int index) { m_selectedIndex = index; }

    WebViewImpl* getWebView() const;
    WebCore::HTMLInputElement* getTextField() const { return m_textField.get(); }

private:
    WebCore::RenderStyle* textFieldStyle() const;

    RefPtr<WebCore::HTMLInputElement> m_textField;
    int m_selectedIndex;
    OwnPtr<WebCore::PopupMenuStyle> m_style;
};

} // namespace WebKit

#endif
