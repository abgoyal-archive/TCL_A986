
#ifndef _RANDOM_H_
#define _RANDOM_H_

struct mp_int;

void seedrandom();
void reseedrandom();
void genrandom(unsigned char* buf, int len);
void addrandom(unsigned char* buf, int len);
void gen_random_mpint(mp_int *max, mp_int *rand);

#endif /* _RANDOM_H_ */
