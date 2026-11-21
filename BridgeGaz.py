import socket, select, subprocess
WSL_IP = subprocess.check_output(["hostname", "-I"]).decode().split()[0]
s_ext = socket.socket(2, 2); s_ext.bind((WSL_IP, 9002))
s_loc = socket.socket(2, 2); s_loc.bind(("127.0.0.1", 0))
win_addr = None
print(f"Bridge {WSL_IP}:9002 <-> 127.0.0.1:9002 ready")
while True:
    r, _, _ = select.select([s_ext, s_loc], [], [])
    if s_ext in r:
        data, addr = s_ext.recvfrom(65535)
        if win_addr != addr: print(f"Packet from Docker: {addr}, len={len(data)}")
        win_addr = addr
        s_loc.sendto(data, ("127.0.0.1", 9002))
    if s_loc in r:
        data, _ = s_loc.recvfrom(65535)
        #print(f"Reply from Gazebo! len={len(data)}")
        if win_addr: s_ext.sendto(data, win_addr)
