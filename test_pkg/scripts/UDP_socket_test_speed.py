#!/usr/bin/env python3

import sys, time
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
import socket 
import os
import struct
import datetime
import timeit
FORMAT = "utf-8"      
Dt=1

def UDP_server(localIP,localPort): 
    # Create UDP server socket
    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    UDPServerSocket.bind((localIP,localPort))
    print("UDP server up and listening")
    dim=0
    countMsgs = 0
    i=0

    start=time.time()
    while not rospy.is_shutdown():
        bytesAddressPair = UDPServerSocket.recvfrom(65536)
        data = bytesAddressPair[0]
        #address = bytesAddressPair[1]
        countMsgs = countMsgs+1   
        #print('last msg dimension: ',sys.getsizeof(data))    
        #dim=dim+sys.getsizeof(data)
        if(time.time()-start>Dt):
            #print("speed: ",round(dim/(1000*(time.time()-start)),0) ,"Kbyte/s")
            print(countMsgs,"messages arrived, speed: ",countMsgs*50*Dt ,"Kbyte/s, ",i)
            countMsgs=0
            i=i+1
            #dim=0
            start=time.time()
        
        

if __name__ == "__main__":
    rospy.init_node('UDP_socket_test_speed')
    #image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)

	# getting the IP address 
    ip_add = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]
    print("IP Address: ", ip_add)
    
    #creating the server at the specified ip and port
    UDP_server(ip_add,8081)

    #uncomment this if the auto-recognition of the Ip doesn't work
    #sub_server(("192.168.1.195",8888)) 
