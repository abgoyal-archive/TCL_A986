

#ifndef ELOOP_H
#define ELOOP_H

#define ELOOP_ALL_CTX (void *) -1

typedef enum {
	EVENT_TYPE_READ = 0,
	EVENT_TYPE_WRITE,
	EVENT_TYPE_EXCEPTION
} eloop_event_type;

typedef void (*eloop_sock_handler)(int sock, void *eloop_ctx, void *sock_ctx);

typedef void (*eloop_event_handler)(void *eloop_data, void *user_ctx);

typedef void (*eloop_timeout_handler)(void *eloop_data, void *user_ctx);

typedef void (*eloop_signal_handler)(int sig, void *eloop_ctx,
				     void *signal_ctx);

int eloop_init(void *user_data);

int eloop_register_read_sock(int sock, eloop_sock_handler handler,
			     void *eloop_data, void *user_data);

void eloop_unregister_read_sock(int sock);

int eloop_register_sock(int sock, eloop_event_type type,
			eloop_sock_handler handler,
			void *eloop_data, void *user_data);

void eloop_unregister_sock(int sock, eloop_event_type type);

int eloop_register_event(void *event, size_t event_size,
			 eloop_event_handler handler,
			 void *eloop_data, void *user_data);

void eloop_unregister_event(void *event, size_t event_size);

int eloop_register_timeout(unsigned int secs, unsigned int usecs,
			   eloop_timeout_handler handler,
			   void *eloop_data, void *user_data);

int eloop_cancel_timeout(eloop_timeout_handler handler,
			 void *eloop_data, void *user_data);

int eloop_is_timeout_registered(eloop_timeout_handler handler,
				void *eloop_data, void *user_data);

int eloop_register_signal(int sig, eloop_signal_handler handler,
			  void *user_data);

int eloop_register_signal_terminate(eloop_signal_handler handler,
				    void *user_data);

int eloop_register_signal_reconfig(eloop_signal_handler handler,
				   void *user_data);

void eloop_run(void);

void eloop_terminate(void);

void eloop_destroy(void);

int eloop_terminated(void);

void eloop_wait_for_read_sock(int sock);

void * eloop_get_user_data(void);

#endif /* ELOOP_H */
