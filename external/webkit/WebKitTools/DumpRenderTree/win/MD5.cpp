

#include "config.h"
#include "MD5.h"

#include <windows.h>

typedef void (WINAPI*initPtr)(MD5_CTX*);
typedef void (WINAPI*updatePtr)(MD5_CTX*, unsigned char*, unsigned);
typedef void (WINAPI*finalPtr)(MD5_CTX*);

static HMODULE cryptDLL()
{
    static HMODULE module = LoadLibraryW(L"Cryptdll.dll");
    return module;
}

static initPtr init()
{
    static initPtr ptr = reinterpret_cast<initPtr>(GetProcAddress(cryptDLL(), "MD5Init"));
    return ptr;
}

static updatePtr update()
{
    static updatePtr ptr = reinterpret_cast<updatePtr>(GetProcAddress(cryptDLL(), "MD5Update"));
    return ptr;
}

static finalPtr final()
{
    static finalPtr ptr = reinterpret_cast<finalPtr>(GetProcAddress(cryptDLL(), "MD5Final"));
    return ptr;
}

void MD5_Init(MD5_CTX* context)
{
    init()(context);
}

void MD5_Update(MD5_CTX* context, unsigned char* input, unsigned length)
{
    update()(context, input, length);
}

void MD5_Final(unsigned char hash[16], MD5_CTX* context)
{
    final()(context);

    for (int i = 0; i < 16; ++i)
        hash[i] = context->digest[i];
}
