"""Correct signatures, wrong file name.
"""


def web_socket_do_extra_handshake(request):
    pass


def web_socket_transfer_data(request):
    request.connection.write(
            'sub/no_wsh_at_the_end.py is called for %s, %s' %
            (request.ws_resource, request.ws_protocol))


# vi:sts=4 sw=4 et
