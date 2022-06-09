import cv2
import os
import sys
from os.path import isfile, join
import random

pathIn="original_images/"
pathOut="augmented_images"

files = [f for f in os.listdir(pathIn) if isfile(join(pathIn, f))]
#for sorting the file names properly
files.sort()
print("augmenting...")
for i in range(len(files)):
    filename=pathIn + files[i]
    #reading each files
    img = cv2.imread(filename)
    height, width, layers = img.shape
    size = (width,height)
    #print("adding: ",filename)

    augmented = cv2.flip(img, 0)
    cv2.imwrite(os.path.join(pathOut,"augmented_flip0_"+str(i)+".jpg"),augmented)

    augmented = cv2.flip(img, 1)
    cv2.imwrite(os.path.join(pathOut,"augmented_flip1_"+str(i)+".jpg"),augmented)

    #augmented = cv2.flip(img, -1)
    #cv2.imwrite(os.path.join(pathOut,"augmented_flip-1_"+str(i)+".jpg"),augmented)

    contrast_coeff = random.random()*2 + 0.1
    #print(contrast_coeff)
    augmented = img * contrast_coeff 
    cv2.imwrite(os.path.join(pathOut,"augmented_contrast_"+str(i)+".jpg"),augmented)

    #brightness_coeff=random.randint(0, 50)
    #print(brightness_coeff)
    #augmented = img + brightness_coeff
    #cv2.imwrite(os.path.join(pathOut,"augmented_brightness_"+str(i)+".jpg"),augmented)