

#ifndef CachedInput_H
#define CachedInput_H

#include "CachedDebug.h"
#include "HTMLInputElement.h"
#include "PlatformString.h"

namespace android {

class CachedInput {
public:
    CachedInput() {
        // Initiaized to 0 in its array, so nothing to do in the
        // constructor
    }
    void* formPointer() const { return mForm; }
    void init() {
        bzero(this, sizeof(CachedInput));
        mName = WebCore::String();
    }
    WebCore::HTMLInputElement::InputType inputType() const { return mInputType; }
    bool isRtlText() const { return mIsRtlText; }
    bool isTextField() const { return mIsTextField; }
    int maxLength() const { return mMaxLength; };
    const WebCore::String& name() const { return mName; }
    int paddingBottom() const { return mPaddingBottom; }
    int paddingLeft() const { return mPaddingLeft; }
    int paddingRight() const { return mPaddingRight; }
    int paddingTop() const { return mPaddingTop; }
    void setFormPointer(void* form) { mForm = form; }
    void setInputType(WebCore::HTMLInputElement::InputType type) { mInputType = type; }
    void setIsRtlText(bool isRtlText) { mIsRtlText = isRtlText; }
    void setIsTextField(bool isTextField) { mIsTextField = isTextField; }
    void setMaxLength(int maxLength) { mMaxLength = maxLength; }
    void setName(const WebCore::String& name) { mName = name; }
    void setPaddingBottom(int bottom) { mPaddingBottom = bottom; }
    void setPaddingLeft(int left) { mPaddingLeft = left; }
    void setPaddingRight(int right) { mPaddingRight = right; }
    void setPaddingTop(int top) { mPaddingTop = top; }
    void setTextSize(int textSize) { mTextSize = textSize; }
    int textSize() const { return mTextSize; }
private:
    void* mForm;
    WebCore::HTMLInputElement::InputType mInputType;
    int mMaxLength;
    WebCore::String mName;
    int mPaddingBottom;
    int mPaddingLeft;
    int mPaddingRight;
    int mPaddingTop;
    int mTextSize;
    bool mIsRtlText : 1;
    bool mIsTextField : 1;
#if DUMP_NAV_CACHE
public:
    class Debug {
public:
        CachedInput* base() const;
        void print() const;
    } mDebug;
#endif
};

}

#endif
