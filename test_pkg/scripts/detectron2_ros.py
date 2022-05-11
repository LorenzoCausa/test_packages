#!/usr/bin/env python3

# IMPORTS
import rospy
import roslib
import numpy as np
import cv2
import time
from datetime import datetime
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as SensImage

import argparse
import os
import sys

from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

import torch
import detectron2
print(f"Detectron2 version is {detectron2.__version__}")
from detectron2.engine import DefaultPredictor
from detectron2 import model_zoo
from detectron2.utils.logger import setup_logger
setup_logger()
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import Metadata
from detectron2.utils.visualizer import ColorMode
from detectron2.data import MetadataCatalog, DatasetCatalog

from PIL import Image 

#GLOBAL VARIABLES
cv_image=None

# FUNCTIONS
def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='default', help='model path(s)')
    parser.add_argument('--source', nargs='+', type=str, default='ros', help='path to an image, default works with ros')
    opt = parser.parse_args()
    return opt

def showSegmentation(visualizer,out,plot=True,pub=None):
    out_vis = visualizer.draw_instance_predictions(out["instances"].to("cpu"))
    if plot:
        cv2.imshow("Detection",out_vis.get_image()[:, :, ::-1])
    if pub is not None:
        cv_bridge=CvBridge()
        pub.publish(cv_bridge.cv2_to_imgmsg(out_vis.get_image()[:, :, ::-1], 'bgr8'))
    #cv2.waitKey(0)

def getMask(out):
    masks = np.asarray(out["instances"].pred_masks.to("cpu"))
    if(len(masks[:,0,0])>0):
        total_mask = np.zeros([len(masks[0,:,0]),len(masks[0,0,:]),1],dtype=np.uint8)

        for i in range(len(masks[:,0,0])):
            # Pick an item to mask
            item_mask = masks[i]

            # Create a PIL image out of the mask
            mask = Image.fromarray((item_mask * 255).astype('uint8'))
            mask_imageBlackWhite = np.array(mask)
            mask_image = cv2.cvtColor(mask_imageBlackWhite,cv2.COLOR_GRAY2RGB)
            total_mask=np.add(total_mask,mask_image)
            # Display the image
            mask_i="Mask "+str(i)
            #cv2.imshow(mask_i,mask_image)
            #cv2.waitKey(0)
    
    else:
        total_mask=None
    return(total_mask)

def getOrientedBoxes(mask,plot,pub=None):
    if mask is None:
        return None,None
    
    # Convert image to grayscale
    gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

    # Convert image to binary
    _, bw = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)
 
    # Find all the contours in the thresholded image
    contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    
    angles=[]
    centers=[]

    for i, c in enumerate(contours): 
        # Calculate the area of each contour
        area = cv2.contourArea(c)
 
        # Ignore contours that are too small or too large
        #if area < 3700 or 100000 < area:
        #   continue

        #cv.minAreaRect returns:
        #(center(x, y), (width, height), angle of rotation) = cv2.minAreaRect(c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # Retrieve the key parameters of the rotated bounding box
        center = (int(rect[0][0]),int(rect[0][1])) 
        width = int(rect[1][0])
        height = int(rect[1][1])
        angle = int(rect[2])

        if width < height:
            angle = 90 - angle
        else:
            angle = 180 - angle

        angles.append(angle)
        centers.append(center)

        label = str(angle) + " degrees"
        #textbox = cv2.rectangle(mask, (center[0]-35, center[1]-25), (center[0] + 295, center[1] + 10), (255,255,255), -1)
        cv2.drawContours(mask,[box],0,(0,0,255),2)
        cv2.putText(mask, label, (center[0]-0, center[1]-25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,0), 1, cv2.LINE_AA)      

    #print(angles)
    #print(centers)
    tot_angle=0
    tot_center=[0,0]

    for i in range(len(angles)):
        tot_angle=tot_angle+angles[i]
        tot_center[0]=tot_center[0]+centers[i][0]
        tot_center[1]=tot_center[1]+centers[i][1]
    
    avg_angle=tot_angle/len(angles)
    avg_center=[tot_center[0]/len(centers),tot_center[1]/len(centers)]
    avg_center=[int(avg_center[0]),int(avg_center[1])]

    #print(avg_angle)
    #print(avg_center)

    mask = cv2.circle(mask, (avg_center[0], avg_center[1]), radius=10, color=(0, 0, 255), thickness=3) # draw the center

    if plot:
        cv2.imshow("Mask with boxes", mask)

    if pub is not None:
        cv_bridge=CvBridge()
        pub.publish(cv_bridge.cv2_to_imgmsg(mask, 'bgr8'))

    return center,avg_angle,

# SUBSCRIBERs CALLBACK
def callback(startImg):
    global cv_image
    cv_bridge=CvBridge()
    cv_image = cv_bridge.imgmsg_to_cv2(startImg, 'bgr8')
    #print("image receveid")

def main():
    rospy.init_node('detectron2', anonymous=False)
    args= parse_opt()
    cfg = get_cfg()
    # add project-specific config (e.g., TensorMask) here if you're not running a model in detectron2's core library
    cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.9  # set threshold for this model
    cfg.MODEL.DEVICE='cpu' # Run on CPU
    # Find a model from detectron2's model zoo. You can use the https://dl.fbaipublicfiles... url as well
    if(args.weights=="default"):
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
        my_metadata = MetadataCatalog.get(cfg.DATASETS.TRAIN[0])

    else:
        cfg.MODEL.ROI_HEADS.NUM_CLASSES=1
        cfg.MODEL.WEIGHTS = os.path.join(ROOT,"detectron2_weights",args.weights[0])
        my_metadata = Metadata()
        my_metadata.set(thing_classes = ['rail'])

    predictor = DefaultPredictor(cfg)
    

    # For test and debug
    if(args.source!='ros'):
        img = cv2.imread(args.source[0])
        #cv2.imshow("my image",img)
        #cv2.waitKey()
        now = datetime.now()   
        outputs = predictor(img)
        
        v = Visualizer(img[:, :, ::-1],
               metadata=my_metadata, 
               scale=1, 
               instance_mode=ColorMode.IMAGE_BW    # remove the colors of unsegmented pixels. This option is only available for segmentation models
               )

        mask=getMask(outputs)
        getOrientedBoxes(mask,True)
        showSegmentation(v,outputs)
        print("total time: ", datetime.now()-now)
        cv2.waitKey(0)

    else:
        pub_boxes = rospy.Publisher('boxes_and_mask', SensImage, queue_size=1)
        pub_segm = rospy.Publisher('segmentation', SensImage, queue_size=1)
        rospy.Subscriber("output/image_raw", SensImage, callback,queue_size=1)
        time.sleep(1)

        while not rospy.is_shutdown():
            v = Visualizer(cv_image[:, :, ::-1],
               metadata=my_metadata, 
               scale=1, 
               instance_mode=ColorMode.IMAGE_BW    # remove the colors of unsegmented pixels. This option is only available for segmentation models
               )
            outputs = predictor(cv_image)
            mask=getMask(outputs)
            [center,angle]=getOrientedBoxes(mask,False,pub_boxes)
            showSegmentation(v,outputs,False,pub_segm)

    #rospy.spin()


if __name__ == "__main__":
    main()