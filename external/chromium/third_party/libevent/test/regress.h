#ifndef _REGRESS_H_
#define _REGRESS_H_

#ifdef __cplusplus
extern "C" {
#endif

void http_suite(void);
void http_basic_test(void);

void rpc_suite(void);

void dns_suite(void);
	
#ifdef __cplusplus
}
#endif

#endif /* _REGRESS_H_ */
