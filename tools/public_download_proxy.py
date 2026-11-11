"""Loopback-only CONNECT tunnel for public model/package download hosts.

No TLS interception or credential logging. Used via a user-owned SSH reverse
forward to route slow GB10 public downloads through the laptop's connection.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import select
import socket


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_CONNECT(self):
        host, separator, port = self.path.rpartition(':')
        host = host.lower()
        allowed = ('pytorch.org', 'pythonhosted.org', 'pypi.org', 'huggingface.co', 'hf.co')
        if not separator or port != '443' or not any(host == name or host.endswith('.'+name) for name in allowed):
            self.send_error(403)
            return
        try:
            with socket.create_connection((host, 443), timeout=20) as upstream:
                self.send_response(200, 'Connection established')
                self.end_headers()
                sockets = [self.connection, upstream]
                while True:
                    readable, _, errors = select.select(sockets, [], sockets, 60)
                    if errors or not readable:
                        return
                    for source in readable:
                        block = source.recv(1024*256)
                        if not block:
                            return
                        (upstream if source is self.connection else self.connection).sendall(block)
        except (OSError, TimeoutError):
            self.close_connection = True


if __name__ == '__main__':
    server = ThreadingHTTPServer(('127.0.0.1', 18081), Handler)
    server.daemon_threads = True
    server.serve_forever()
