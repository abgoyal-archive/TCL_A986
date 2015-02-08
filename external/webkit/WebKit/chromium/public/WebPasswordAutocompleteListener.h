

#ifndef WebPasswordAutocompleteListener_h
#define WebPasswordAutocompleteListener_h

namespace WebKit {
class WebString;

class WebPasswordAutocompleteListener {
public:
    virtual ~WebPasswordAutocompleteListener() {}

    virtual void didBlurInputElement(
        const WebString& userInput) = 0;

    virtual void performInlineAutocomplete(
        const WebString& userInput,
        bool backSpaceOrDeletePressed,
        bool showSuggestions) = 0;
};

} // namespace WebKit


#endif
