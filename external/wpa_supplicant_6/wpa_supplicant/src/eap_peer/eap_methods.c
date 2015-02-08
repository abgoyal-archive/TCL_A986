

#include "includes.h"
#ifdef CONFIG_DYNAMIC_EAP_METHODS
#include <dlfcn.h>
#endif /* CONFIG_DYNAMIC_EAP_METHODS */

#include "common.h"
#include "eap_i.h"
#include "eap_methods.h"


static struct eap_method *eap_methods = NULL;


const struct eap_method * eap_peer_get_eap_method(int vendor, EapType method)
{
	struct eap_method *m;
	for (m = eap_methods; m; m = m->next) {
		if (m->vendor == vendor && m->method == method)
			return m;
	}
	return NULL;
}


EapType eap_peer_get_type(const char *name, int *vendor)
{
	struct eap_method *m;
	for (m = eap_methods; m; m = m->next) {
		if (os_strcmp(m->name, name) == 0) {
			*vendor = m->vendor;
			return m->method;
		}
	}
	*vendor = EAP_VENDOR_IETF;
	return EAP_TYPE_NONE;
}


const char * eap_get_name(int vendor, EapType type)
{
	struct eap_method *m;
	for (m = eap_methods; m; m = m->next) {
		if (m->vendor == vendor && m->method == type)
			return m->name;
	}
	return NULL;
}


size_t eap_get_names(char *buf, size_t buflen)
{
	char *pos, *end;
	struct eap_method *m;
	int ret;

	if (buflen == 0)
		return 0;

	pos = buf;
	end = pos + buflen;

	for (m = eap_methods; m; m = m->next) {
		ret = os_snprintf(pos, end - pos, "%s%s",
				  m == eap_methods ? "" : " ", m->name);
		if (ret < 0 || ret >= end - pos)
			break;
		pos += ret;
	}
	buf[buflen - 1] = '\0';

	return pos - buf;
}


char ** eap_get_names_as_string_array(size_t *num)
{
	struct eap_method *m;
	size_t array_len = 0;
	char **array;
	int i = 0, j;

	for (m = eap_methods; m; m = m->next)
		array_len++;

	array = os_zalloc(sizeof(char *) * (array_len + 1));
	if (array == NULL)
		return NULL;

	for (m = eap_methods; m; m = m->next) {
		array[i++] = os_strdup(m->name);
		if (array[i - 1] == NULL) {
			for (j = 0; j < i; j++)
				os_free(array[j]);
			os_free(array);
			return NULL;
		}
	}
	array[i] = NULL;

	if (num)
		*num = array_len;

	return array;
}


const struct eap_method * eap_peer_get_methods(size_t *count)
{
	int c = 0;
	struct eap_method *m;

	for (m = eap_methods; m; m = m->next)
		c++;
	
	*count = c;
	return eap_methods;
}


#ifdef CONFIG_DYNAMIC_EAP_METHODS
int eap_peer_method_load(const char *so)
{
	void *handle;
	int (*dyn_init)(void);
	int ret;

	handle = dlopen(so, RTLD_LAZY);
	if (handle == NULL) {
		wpa_printf(MSG_ERROR, "EAP: Failed to open dynamic EAP method "
			   "'%s': %s", so, dlerror());
		return -1;
	}

	dyn_init = dlsym(handle, "eap_peer_method_dynamic_init");
	if (dyn_init == NULL) {
		dlclose(handle);
		wpa_printf(MSG_ERROR, "EAP: Invalid EAP method '%s' - no "
			   "eap_peer_method_dynamic_init()", so);
		return -1;
	}

	ret = dyn_init();
	if (ret) {
		dlclose(handle);
		wpa_printf(MSG_ERROR, "EAP: Failed to add EAP method '%s' - "
			   "ret %d", so, ret);
		return ret;
	}

	/* Store the handle for this shared object. It will be freed with
	 * dlclose() when the EAP method is unregistered. */
	eap_methods->dl_handle = handle;

	wpa_printf(MSG_DEBUG, "EAP: Loaded dynamic EAP method: '%s'", so);

	return 0;
}


int eap_peer_method_unload(struct eap_method *method)
{
	struct eap_method *m, *prev;
	void *handle;

	m = eap_methods;
	prev = NULL;
	while (m) {
		if (m == method)
			break;
		prev = m;
		m = m->next;
	}

	if (m == NULL || m->dl_handle == NULL)
		return -1;

	if (prev)
		prev->next = m->next;
	else
		eap_methods = m->next;

	handle = m->dl_handle;

	if (m->free)
		m->free(m);
	else
		eap_peer_method_free(m);

	dlclose(handle);

	return 0;
}
#endif /* CONFIG_DYNAMIC_EAP_METHODS */


struct eap_method * eap_peer_method_alloc(int version, int vendor,
					  EapType method, const char *name)
{
	struct eap_method *eap;
	eap = os_zalloc(sizeof(*eap));
	if (eap == NULL)
		return NULL;
	eap->version = version;
	eap->vendor = vendor;
	eap->method = method;
	eap->name = name;
	return eap;
}


void eap_peer_method_free(struct eap_method *method)
{
	os_free(method);
}


int eap_peer_method_register(struct eap_method *method)
{
	struct eap_method *m, *last = NULL;

	if (method == NULL || method->name == NULL ||
	    method->version != EAP_PEER_METHOD_INTERFACE_VERSION)
		return -1;

	for (m = eap_methods; m; m = m->next) {
		if ((m->vendor == method->vendor &&
		     m->method == method->method) ||
		    os_strcmp(m->name, method->name) == 0)
			return -2;
		last = m;
	}

	if (last)
		last->next = method;
	else
		eap_methods = method;

	return 0;
}


int eap_peer_register_methods(void)
{
	int ret = 0;

#ifdef EAP_MD5
	if (ret == 0) {
		int eap_peer_md5_register(void);
		ret = eap_peer_md5_register();
	}
#endif /* EAP_MD5 */

#ifdef EAP_TLS
	if (ret == 0) {
		int eap_peer_tls_register(void);
		ret = eap_peer_tls_register();
	}
#endif /* EAP_TLS */

#ifdef EAP_MSCHAPv2
	if (ret == 0) {
		int eap_peer_mschapv2_register(void);
		ret = eap_peer_mschapv2_register();
	}
#endif /* EAP_MSCHAPv2 */

#ifdef EAP_PEAP
	if (ret == 0) {
		int eap_peer_peap_register(void);
		ret = eap_peer_peap_register();
	}
#endif /* EAP_PEAP */

#ifdef EAP_TTLS
	if (ret == 0) {
		int eap_peer_ttls_register(void);
		ret = eap_peer_ttls_register();
	}
#endif /* EAP_TTLS */

#ifdef EAP_GTC
	if (ret == 0) {
		int eap_peer_gtc_register(void);
		ret = eap_peer_gtc_register();
	}
#endif /* EAP_GTC */

#ifdef EAP_OTP
	if (ret == 0) {
		int eap_peer_otp_register(void);
		ret = eap_peer_otp_register();
	}
#endif /* EAP_OTP */

#ifdef EAP_SIM
	if (ret == 0) {
		int eap_peer_sim_register(void);
		ret = eap_peer_sim_register();
	}
#endif /* EAP_SIM */

#ifdef EAP_LEAP
	if (ret == 0) {
		int eap_peer_leap_register(void);
		ret = eap_peer_leap_register();
	}
#endif /* EAP_LEAP */

#ifdef EAP_PSK
	if (ret == 0) {
		int eap_peer_psk_register(void);
		ret = eap_peer_psk_register();
	}
#endif /* EAP_PSK */

#ifdef EAP_AKA
	if (ret == 0) {
		int eap_peer_aka_register(void);
		ret = eap_peer_aka_register();
	}
#endif /* EAP_AKA */

#ifdef EAP_AKA_PRIME
	if (ret == 0) {
		int eap_peer_aka_prime_register(void);
		ret = eap_peer_aka_prime_register();
	}
#endif /* EAP_AKA_PRIME */

#ifdef EAP_FAST
	if (ret == 0) {
		int eap_peer_fast_register(void);
		ret = eap_peer_fast_register();
	}
#endif /* EAP_FAST */

#ifdef EAP_PAX
	if (ret == 0) {
		int eap_peer_pax_register(void);
		ret = eap_peer_pax_register();
	}
#endif /* EAP_PAX */

#ifdef EAP_SAKE
	if (ret == 0) {
		int eap_peer_sake_register(void);
		ret = eap_peer_sake_register();
	}
#endif /* EAP_SAKE */

#ifdef EAP_GPSK
	if (ret == 0) {
		int eap_peer_gpsk_register(void);
		ret = eap_peer_gpsk_register();
	}
#endif /* EAP_GPSK */

#ifdef EAP_WSC
	if (ret == 0) {
		int eap_peer_wsc_register(void);
		ret = eap_peer_wsc_register();
	}
#endif /* EAP_WSC */

#ifdef EAP_IKEV2
	if (ret == 0) {
		int eap_peer_ikev2_register(void);
		ret = eap_peer_ikev2_register();
	}
#endif /* EAP_IKEV2 */

#ifdef EAP_VENDOR_TEST
	if (ret == 0) {
		int eap_peer_vendor_test_register(void);
		ret = eap_peer_vendor_test_register();
	}
#endif /* EAP_VENDOR_TEST */

#ifdef EAP_TNC
	if (ret == 0) {
		int eap_peer_tnc_register(void);
		ret = eap_peer_tnc_register();
	}
#endif /* EAP_TNC */

	return ret;
}


void eap_peer_unregister_methods(void)
{
	struct eap_method *m;
#ifdef CONFIG_DYNAMIC_EAP_METHODS
	void *handle;
#endif /* CONFIG_DYNAMIC_EAP_METHODS */

	while (eap_methods) {
		m = eap_methods;
		eap_methods = eap_methods->next;

#ifdef CONFIG_DYNAMIC_EAP_METHODS
		handle = m->dl_handle;
#endif /* CONFIG_DYNAMIC_EAP_METHODS */

		if (m->free)
			m->free(m);
		else
			eap_peer_method_free(m);

#ifdef CONFIG_DYNAMIC_EAP_METHODS
		if (handle)
			dlclose(handle);
#endif /* CONFIG_DYNAMIC_EAP_METHODS */
	}
}
