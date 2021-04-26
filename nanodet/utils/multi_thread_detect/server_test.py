import time
import socket

server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
server.bind(('127.0.0.1',8000))
server.listen(5)
print("waiting msg ...")
conn, clint_add = server.accept()
while True:
    time.sleep(0.005)
    all_box = [14.1,1.2,2.55,3.45,5.16,0]
    send_data_byte = bytes(0)
    if len(all_box) == 0:
        leftup_rightdown_corner = [-1, 0, 0, 0, 0,time.time(),'b']
        for i in range(len(leftup_rightdown_corner)):
            #print(pickup_leftup_rightdown_corner[i])
            pickup_senddata = str(leftup_rightdown_corner[i]) + ','
            # print(pickup_senddata.encode())
            send_data_byte += pickup_senddata.encode()
            # print(send_data_byte)
        conn.send(send_data_byte)
    else:
        x0, y0, x1, y1 = [0.0,1.0,2.0,3.0]
        leftup_rightdown_corner = [1, x0, y0, x1, y1,time.time(),'a']
        for i in range(len(leftup_rightdown_corner)):
            target_senddata = str(leftup_rightdown_corner[i]) + ','
            send_data_byte += target_senddata.encode()
        #print(send_data_byte)
        #print(len(send_data_byte))
        conn.send(send_data_byte)
