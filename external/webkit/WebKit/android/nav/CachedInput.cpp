

#include "CachedPrefix.h"
#include "CachedInput.h"

namespace android {

#if DUMP_NAV_CACHE

#define DEBUG_PRINT_BOOL(field) \
    DUMP_NAV_LOGD("// bool " #field "=%s;\n", b->field ? "true" : "false")

CachedInput* CachedInput::Debug::base() const {
    CachedInput* nav = (CachedInput*) ((char*) this - OFFSETOF(CachedInput, mDebug));
    return nav;
}

static void printWebCoreString(const char* label,
        const WebCore::String& string) {
    char scratch[256];
    size_t index = snprintf(scratch, sizeof(scratch), label);
    const UChar* ch = string.characters();
    while (ch && *ch && index < sizeof(scratch)) {
        UChar c = *ch++;
        if (c < ' ' || c >= 0x7f) c = ' ';
        scratch[index++] = c;
    }
    DUMP_NAV_LOGD("%.*s\"\n", index, scratch);
}

void CachedInput::Debug::print() const
{
    CachedInput* b = base();
    printWebCoreString("// char* mName=\"", b->mName);
    DUMP_NAV_LOGD("// void* mForm=%p;", b->mForm);
    DUMP_NAV_LOGD("// int mMaxLength=%d;\n", b->mMaxLength);
    DUMP_NAV_LOGD("// int mTextSize=%d;\n", b->mTextSize);
    DUMP_NAV_LOGD("// int mInputType=%d;\n", b->mInputType);
    DUMP_NAV_LOGD("// int mPaddingLeft=%d;\n", b->mPaddingLeft);
    DUMP_NAV_LOGD("// int mPaddingTop=%d;\n", b->mPaddingTop);
    DUMP_NAV_LOGD("// int mPaddingRight=%d;\n", b->mPaddingRight);
    DUMP_NAV_LOGD("// int mPaddingBottom=%d;\n", b->mPaddingBottom);
    DEBUG_PRINT_BOOL(mIsRtlText);
    DEBUG_PRINT_BOOL(mIsTextField);
}

#endif

}
