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

if __name__ == "__main__":
    '''
        initialize the node and the publisher
    '''

    rospy.init_node('UDP_client', anonymous=True)
    
	## getting the IP address 
    ip_add = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]
    print("IP Address: ", ip_add)
    msgFromClient       = "Hello UDP Server"
    bytesToSend         = str.encode(msgFromClient)
    serverAddressPort   = (ip_add, 8888)
    bufferSize          = 1024
    # Create a UDP socket at client side
    UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    # Send to server using created UDP socket
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)
