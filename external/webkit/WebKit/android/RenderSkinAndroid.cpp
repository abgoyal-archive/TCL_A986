

#define LOG_TAG "WebCore"

#include "config.h"
#include "RenderSkinAndroid.h"
#include "RenderSkinButton.h"
#include "RenderSkinCombo.h"
#include "RenderSkinMediaButton.h"
#include "RenderSkinRadio.h"
#include "SkImageDecoder.h"

#include "utils/AssetManager.h"
#include "utils/Asset.h"

namespace WebCore {
 
RenderSkinAndroid::RenderSkinAndroid()
    : m_height(0)
    , m_width(0)
{}

void RenderSkinAndroid::Init(android::AssetManager* am, String drawableDirectory)
{
    RenderSkinButton::Init(am, drawableDirectory);
    RenderSkinCombo::Init(am, drawableDirectory);
    RenderSkinMediaButton::Init(am, drawableDirectory);
    RenderSkinRadio::Init(am, drawableDirectory);
}

bool RenderSkinAndroid::DecodeBitmap(android::AssetManager* am, const char* fileName, SkBitmap* bitmap)
{
    android::Asset* asset = am->open(fileName, android::Asset::ACCESS_BUFFER);
    if (!asset) {
        asset = am->openNonAsset(fileName, android::Asset::ACCESS_BUFFER);
        if (!asset) {
            LOGD("RenderSkinAndroid: File \"%s\" not found.\n", fileName);
            return false;
        }
    }
    
    bool success = SkImageDecoder::DecodeMemory(asset->getBuffer(false), asset->getLength(), bitmap);
    if (!success) {
        LOGD("RenderSkinAndroid: Failed to decode %s\n", fileName);
    }

    delete asset;
    return success;
}

} // namespace WebCore
