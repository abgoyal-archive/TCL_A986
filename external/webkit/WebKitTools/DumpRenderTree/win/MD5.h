

#ifndef MD5_h
#define MD5_h

typedef unsigned long ULONG;

struct MD5_CTX {
    ULONG i[2];
    ULONG buf[4];
    unsigned char in[64];
    unsigned char digest[16];
};

void MD5_Init(MD5_CTX*);
void MD5_Update(MD5_CTX*, unsigned char* input, unsigned length);
void MD5_Final(unsigned char hash[16], MD5_CTX*);

#endif // MD5_h
