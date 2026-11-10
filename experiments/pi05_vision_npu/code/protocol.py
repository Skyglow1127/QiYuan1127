"""Length-prefixed JSON metadata and fixed-shape FP32 arrays on a Unix socket."""
import json
import struct

def recv_into(sock, buffer):
    view=memoryview(buffer).cast('B')
    while view:
        n=sock.recv_into(view)
        if n==0: raise EOFError('vision worker socket closed')
        view=view[n:]

def send_json(sock,value):
    data=json.dumps(value).encode()
    sock.sendall(struct.pack('!I',len(data)))
    sock.sendall(data)

def recv_json(sock):
    size=bytearray(4); recv_into(sock,size)
    data=bytearray(struct.unpack('!I',size)[0]); recv_into(sock,data)
    return json.loads(data)
