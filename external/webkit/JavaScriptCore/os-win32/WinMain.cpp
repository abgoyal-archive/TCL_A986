

#include "config.h"
#include "Vector.h"
#include <winbase.h>
#include <winnls.h>
#include <wtf/UnusedParam.h>

int main(int argc, char** argv);

static inline char* convertToUtf8(LPCWSTR widecharString, int length)
{
    int requiredSize = WideCharToMultiByte(CP_UTF8, 0, widecharString, length, 0, 0, 0, 0);
    char* multibyteString = new char[requiredSize + 1];

    WideCharToMultiByte(CP_UTF8, 0, widecharString, length, multibyteString, requiredSize, 0, 0);
    multibyteString[requiredSize] = '\0';

    return multibyteString;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow)
{
    UNUSED_PARAM(hInstance);
    UNUSED_PARAM(hPrevInstance);
    UNUSED_PARAM(nCmdShow);

    Vector<char*> arguments;
    TCHAR buffer[MAX_PATH];

    int length = GetModuleFileNameW(0, buffer, MAX_PATH);
    arguments.append(convertToUtf8(buffer, length));

    WCHAR* commandLine = lpCmdLine;
    while (commandLine[0] != '\0') {
        int commandLineLength = 1;
        WCHAR endChar = ' ';

        while (commandLine[0] == ' ')
            ++commandLine;

        if (commandLine[0] == '\"') {
            ++commandLine;
            endChar = '\"';
        }

        while (commandLine[commandLineLength] != endChar && commandLine[commandLineLength] != '\0')
            ++commandLineLength;

        arguments.append(convertToUtf8(commandLine, commandLineLength));

        commandLine += commandLineLength;
        if (endChar != ' ' && commandLine[0] != '\0')
            ++commandLine;
    }

    int res = main(arguments.size(), arguments.data());

    for (size_t i = 0; i < arguments.size(); i++)
        delete arguments[i];

    return res;
}
