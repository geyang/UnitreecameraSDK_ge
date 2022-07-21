import numpy as np
import socket
import sys
from PIL import Image
import time

if __name__ == "__main__":

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("socket successfully created!")
    except socket.error as err:
        print("socket creation failed with error %s" %(err))

    port = 8080

    try:
        host_ip = socket.gethostbyname('127.0.0.1')
    except socket.gaierror:
        print("there was an error resolving the host")
        sys.exit()

    s.connect((host_ip, port))

    print("the socket was successfully connected")

    while True:
        try:
            data = s.recv(1856*800*3)
            if data:
                #if(i_Read_Image == 2) & (image_cols != 0) & (image_rows != 0):
                #print(data)
                data2 = np.fromstring(data, dtype='uint8')
                print(data2)
                img = Image.fromarray(data2.reshape(1856, 800, 3), 'RGB')
            else:
                print('Client disconnected!')
            time.sleep(0.1)
        except:
            s.close()
    
