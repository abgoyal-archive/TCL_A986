

#include "resource.h"
#include <shlwapi.h>
#include <tchar.h>
#include <windows.h>

static LPTSTR getStringValue(HKEY key, LPCTSTR valueName)
{
    DWORD type = 0;
    DWORD bufferSize = 0;
    if (RegQueryValueEx(key, valueName, 0, &type, 0, &bufferSize) != ERROR_SUCCESS || type != REG_SZ)
        return 0;

    LPTSTR buffer = static_cast<LPTSTR>(malloc(bufferSize));
    if (RegQueryValueEx(key, valueName, 0, &type, reinterpret_cast<LPBYTE>(buffer), &bufferSize) != ERROR_SUCCESS) {
        free(buffer);
        return 0;
    }

    return buffer;
}

static LPTSTR applePathFromRegistry(LPCTSTR key, LPCTSTR value)
{
    HKEY applePathKey = 0;
    LONG error = RegOpenKeyEx(HKEY_LOCAL_MACHINE, key, 0, KEY_READ, &applePathKey);
    if (error != ERROR_SUCCESS)
        return 0;
    LPTSTR path = getStringValue(applePathKey, value);
    RegCloseKey(applePathKey);
    return path;
}

static LPTSTR safariInstallDir()
{
    return applePathFromRegistry(TEXT("SOFTWARE\\Apple Computer, Inc.\\Safari"), TEXT("InstallDir"));
}

static LPTSTR safariBrowserExe()
{
    return applePathFromRegistry(TEXT("SOFTWARE\\Apple Computer, Inc.\\Safari"), TEXT("BrowserExe"));
}

int APIENTRY _tWinMain(HINSTANCE instance, HINSTANCE, LPTSTR commandLine, int)
{
    LPTSTR path = safariInstallDir();
    LPTSTR browserExe = safariBrowserExe();
    if (!path || !browserExe) {
        MessageBox(0, TEXT("Safari must be installed to run a WebKit nightly. You can download Safari from http://www.apple.com/safari/download"), TEXT("Safari not found"), MB_ICONSTOP);
        return 1;
    }

    // Set WEBKITNIGHTLY environment variable to point to the nightly bits
    TCHAR exePath[MAX_PATH];
    if (!GetModuleFileName(0, exePath, ARRAYSIZE(exePath)))
        return 1;
    if (!PathRemoveFileSpec(exePath))
        return 1;
    SetEnvironmentVariable(TEXT("WEBKITNIGHTLY"), exePath);

    // Launch Safari as a child process
    STARTUPINFO startupInfo = {0};
    startupInfo.cb = sizeof(startupInfo);
    PROCESS_INFORMATION processInfo = {0};
    if (!CreateProcess(browserExe, commandLine, 0, 0, FALSE, NORMAL_PRIORITY_CLASS | CREATE_UNICODE_ENVIRONMENT, 0, path, &startupInfo, &processInfo))
        MessageBox(0, TEXT("Safari could not be launched. Please make sure you have the latest version of Safari installed and try again. You can download Safari from http://www.apple.com/safari/download"), TEXT("Safari launch failed"), MB_ICONSTOP);

    free(browserExe);
    free(path);
    return 0;
}
