

#include "config.h"

#include "webkiterror.h"

GQuark webkit_network_error_quark(void)
{
    return g_quark_from_static_string("webkit-network-error-quark");
}

GQuark webkit_policy_error_quark(void)
{
    return g_quark_from_static_string("webkit-policy-error-quark");
}

GQuark webkit_plugin_error_quark(void)
{
    return g_quark_from_static_string("webkit-plugin-error-quark");
}
