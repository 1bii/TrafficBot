# Echo client program
import socket
import sys
import numpy as np
import matplotlib.pyplot as plt

def openSocket():
    HOST = '172.20.10.2'    # The remote host
    PORT = 5200             # The same port as used by the server
    s = None
    for res in socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC, socket.SOCK_STREAM):
        af, socktype, proto, canonname, sa = res
        try:
            s = socket.socket(af, socktype, proto)
        except socket.error as msg:
            s = None
            continue
        try:
            s.connect(sa)
        except socket.error as msg:
            s.close()
            s = None
            continue
        break
    if s is None:
        print 'could not open socket'
        sys.exit(1)
    return s

def receiveImg(s):
    print "Reading Picture Size"
    data = ''
    while(1):
        data = s.recv(1024)
        if(len(data)>0):
            break;
    print 'Received', repr(data)
    sp = data.split('\n')
    width = int(sp[0])
    height = int(sp[1])
    print 'width: ', width
    print 'height: ', height
    s.send('a')

    while(1):
        data = s.recv(width*height*3)
        if(len(data)>0):
            break;
    """
    data = s.recv(width*height*3)
    l = len(data)
    while(l< width*height*3):
        data+=s.recv(width*height*3 - l)
        l = len(data)
    """
    print "Received length", len(data)

    img_arr = np.zeros([height, width, 3], dtype = 'uint8')
    for i in range(width):
        for j in range(height):
            idx = j*width + i 
            img_arr[j,i,0] = int(ord(data[idx*3]))
            img_arr[j,i,1] = int(ord(data[idx*3+1]))
            img_arr[j,i,2] = int(ord(data[idx*3+2]))
    return img_arr
    #print img_arr

if __name__ == '__main__':
    s = openSocket()
    print "Open successfully!"
    img = receiveImg(s)
    plt.imshow(img)
    plt.show()
    s.close()
#print 'Received', repr(data)