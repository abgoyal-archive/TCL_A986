

#include "config.h"
#include "DumpRenderTree.h"
#include "LayoutTestController.h"
#include "PixelDumpSupport.h"
#include <wtf/Assertions.h>
#include <wtf/RefPtr.h>
#include <wtf/RetainPtr.h>

#if PLATFORM(CG)
#include "PixelDumpSupportCG.h"
#elif PLATFORM(CAIRO)
#include "PixelDumpSupportCairo.h"
#endif

void dumpWebViewAsPixelsAndCompareWithExpected(const std::string& expectedHash)
{
    RefPtr<BitmapContext> context = createBitmapContextFromWebView(gLayoutTestController->testOnscreen(), gLayoutTestController->testRepaint(), gLayoutTestController->testRepaintSweepHorizontally(), gLayoutTestController->dumpSelectionRect());
    ASSERT(context);
    
    // Compute the hash of the bitmap context pixels
    char actualHash[33];
    computeMD5HashStringForBitmapContext(context.get(), actualHash);
    printf("\nActualHash: %s\n", actualHash);
    
    // Check the computed hash against the expected one and dump image on mismatch
    bool dumpImage = true;
    if (expectedHash.length() > 0) {
        ASSERT(expectedHash.length() == 32);
        
        printf("\nExpectedHash: %s\n", expectedHash.c_str());
        
        if (expectedHash == actualHash)     // FIXME: do case insensitive compare
            dumpImage = false;
    }
    
    if (dumpImage)
      dumpBitmap(context.get());
}

void printPNG(const unsigned char* data, const size_t dataLength)
{
    printf("Content-Type: %s\n", "image/png");
    printf("Content-Length: %lu\n", static_cast<unsigned long>(dataLength));

    const size_t bytesToWriteInOneChunk = 1 << 15;
    size_t dataRemainingToWrite = dataLength;
    while (dataRemainingToWrite) {
        size_t bytesToWriteInThisChunk = std::min(dataRemainingToWrite, bytesToWriteInOneChunk);
        size_t bytesWritten = fwrite(data, 1, bytesToWriteInThisChunk, stdout);
        if (bytesWritten != bytesToWriteInThisChunk)
            break;
        dataRemainingToWrite -= bytesWritten;
        data += bytesWritten;
    }
}
