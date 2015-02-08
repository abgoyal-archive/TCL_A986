from mod_pywebsocket import msgutil


_GOODBYE_MESSAGE = 'Goodbye'


def web_socket_do_extra_handshake(request):
    pass  # Always accept.


def web_socket_transfer_data(request):
    while True:
        line = msgutil.receive_message(request)
        msgutil.send_message(request, line)
        if line == _GOODBYE_MESSAGE:
            return


# vi:sts=4 sw=4 et
