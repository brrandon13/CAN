import socket

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('0,0,0,0',8000))
server.listen(5)

while True:
    conn, addr = server.accept()
    from_client = ''





sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('0.0.0.0', 8000))

sock.send("sending message")

sock.recv(24)

sock.close()