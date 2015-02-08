

#include "config.h"
#include "Logging.h"

#include "PlatformString.h"
#include <wtf/OwnArrayPtr.h>

namespace WebCore {

static inline void initializeWithUserDefault(WTFLogChannel& channel)
{
    DWORD length = GetEnvironmentVariableA(channel.defaultName, 0, 0);
    if (!length)
        return;

    OwnArrayPtr<char> buffer(new char[length]);

    if (!GetEnvironmentVariableA(channel.defaultName, buffer.get(), length))
        return;

    String variableValue(buffer.get());

    static const String& hexadecimalPrefix = *new String("0x");
    if (variableValue.length() < 3 || !variableValue.startsWith(hexadecimalPrefix, false)) {
        LOG_ERROR("Unable to parse hex value for %s (%s), logging is off", channel.defaultName, buffer.get());
        return;
    }

    String unprefixedValue = variableValue.substring(2);

    // Now parse the unprefixed string as a hexadecimal number.
    bool parsedSuccessfully = false;
    unsigned logLevel = unprefixedValue.toUIntStrict(&parsedSuccessfully, 16);

    if (!parsedSuccessfully) {
        LOG_ERROR("Unable to parse hex value for %s (%s), logging is off", channel.defaultName, buffer.get());
        return;
    }

    if ((logLevel & channel.mask) == channel.mask)
        channel.state = WTFLogChannelOn;
    else
        channel.state = WTFLogChannelOff;
}

void InitializeLoggingChannelsIfNecessary()
{
    static bool haveInitializedLoggingChannels = false;
    if (haveInitializedLoggingChannels)
        return;
    haveInitializedLoggingChannels = true;

    initializeWithUserDefault(LogNotYetImplemented);
    initializeWithUserDefault(LogFrames);
    initializeWithUserDefault(LogLoading);
    initializeWithUserDefault(LogPopupBlocking);
    initializeWithUserDefault(LogEvents);
    initializeWithUserDefault(LogEditing);
    initializeWithUserDefault(LogTextConversion);
    initializeWithUserDefault(LogIconDatabase);
    initializeWithUserDefault(LogSQLDatabase);
    initializeWithUserDefault(LogSpellingAndGrammar);
    initializeWithUserDefault(LogBackForward);
    initializeWithUserDefault(LogHistory);
    initializeWithUserDefault(LogPageCache);
    initializeWithUserDefault(LogPlatformLeaks);
    initializeWithUserDefault(LogNetwork);
    initializeWithUserDefault(LogFTP);
    initializeWithUserDefault(LogThreading);
    initializeWithUserDefault(LogStorageAPI);
    initializeWithUserDefault(LogMedia);
    initializeWithUserDefault(LogPlugins);
    initializeWithUserDefault(LogArchives);
}

} // namespace WebCore
