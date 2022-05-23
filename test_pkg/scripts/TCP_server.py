#!/usr/bin/env python3

# Python libs
import sys, time

# numpy and scipy
import numpy as np
#from scipy.ndimage import filters

# OpenCV
import cv2
# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError
import socket 
import os
import struct
FORMAT = "utf-8"      

def sub_server(indirizzo, backlog=1): # blacklog quante richieste può accettare  
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(indirizzo) 
        s.listen(backlog)  
        print("server initializated, listening...")
        count=0
        
    except socket.error as error:
        print("server crash {error}")
        print("try to rerun the server")
        sub_server(indirizzo, backlog=1)

    dim=0 
    start=time.time()

    while True:
        conn, indirizzo_client = s.accept() # accetto la richiesta di un client, 
                                            # funzione che ritorna la connessione (il socket del client) e l'inidrizzo del client 
        string = ''
        buf = bytes(string,encoding="utf-8")
        while len(buf)<4:
            buf += conn.recv(4 - len(buf))
        size = struct.unpack('!i', buf)
        #print("receiving %s bytes" % size)
        #print(indirizzo_client)
        data1=''
        while True:
            data = conn.recv(65536)
            #print("received data")
            #count=count+1
            #print(count) 
            #print(data)
            #print('last msg dimension: ',sys.getsizeof(data),"Bytes") 
            #data1=data1+data
            dim=dim+sys.getsizeof(data)
            if not data:
                #print('BREAK')
                break

        if(time.time()-start>1):
            print("speed: ",round(dim/(1000*(time.time()-start)),0) ,"Kbyte/s")
            dim=0
            start=time.time()

        #print('last msg dimension: ',sys.getsizeof(data)) 
        #print(data1)
    conn.close() 

if __name__ == "__main__":
    '''
        initialize the node and the publisher
    '''

    rospy.init_node('TCP_server', anonymous=True)

    
	## getting the IP address 
    ip_add = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]
    print("IP Address: ", ip_add)
    
    #creating the server at the specified ip and port
    sub_server((ip_add,8081))

