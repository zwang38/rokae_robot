import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import cv2
import numpy as np
import matplotlib as plt 
# import imutils
import os
import sys
sys.path.remove('/usr/bin/python') 
sys.path.append('/usr/bin/python3') 
image_path='/home/nuc/Desktop/rokae_robot/rokae/src/gloal_image_file/camera_image.jpeg'
#   You can see how the program works by using included test.jpg files

# imgtemp = cv.imread(image_path) 
# print(imgtemp.shape)
# cv.imshow('2', img)
# img2 = imgtemp[100:840, 200:550]   # roi 区域
# print(img2.shape)
# cv.imshow('kontury_budynkow', img2)

def cal_pos(image_path):
    img_clr =cv.imread(image_path) 
    img = cv2.cvtColor(img_clr,cv2.COLOR_BGR2GRAY)

    # resize image
    print(img.shape)
    scale = 800.0 / img.shape[1]
    resized = cv2.resize(img, (int(img.shape[1] * scale), int(img.shape[0] * scale)))
    img_clr = cv2.resize(img, (int(img_clr.shape[1] * scale), int(img_clr.shape[0] * scale)))
    
    # Threshold
    kernel = np.ones((1, 3), np.uint8)
    im = cv2.morphologyEx(resized, cv2.MORPH_BLACKHAT, kernel, anchor=(1, 0))
    thresh, im = cv2.threshold(resized, 140, 255, cv2.THRESH_BINARY)
    
    # dilation and erosion
    kernel = np.ones((1, 3), np.uint8)
    im = cv2.morphologyEx(im, cv2.MORPH_DILATE, kernel, anchor=(2, 0), iterations=2) 
    im = cv2.morphologyEx(im, cv2.MORPH_CLOSE, kernel, anchor=(2, 0), iterations=2)  
    
    # Remove elements that are too small to fit/excess noise removal
    kernel = np.ones((3, 3), np.uint8)
    im = cv2.morphologyEx(im, cv2.MORPH_OPEN, kernel, iterations=1)
    
    # contour detection
    # # im2, contours, hierarchy = cv2.findContours(im,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # cnts = cv2.findContours(im.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cnts = imutils.grab_contours(cnts)
    
    contours = cv2.findContours(im,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    unscale = 1.0 / scale
    contours = sorted(contours, key = cv2.contourArea)[:]
    c=0
    rect_c = 0
    if contours != None:
        for contour in contours:
            c = c+1
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
            # segment based on area
            print(str(c)+ " Contour area: ", cv2.contourArea(contour))
            if (cv2.contourArea(contour) >=  401135  or cv2.contourArea(contour) <= 20825 ):
                rect_c = rect_c+1
                print("Missed : "+str(rect_c)+" Area of missed: "+ str(cv2.contourArea(contour)))
                continue
            #draw rect of smallest possible size for contour    
            rect = cv2.minAreaRect(contour)
            #print("Detected Contour area: ", cv2.minAreaRect(contour))
            #rect = ((int(rect[0][0] * unscale), int(rect[0][1] * unscale)),(int(rect[1][0] * unscale), int(rect[1][1] * unscale)),rect[2])
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(img_clr,[box],0,(0,0,255),thickness=2)
    
            (x,y),radius = cv2.minEnclosingCircle(contour)
    
            print('x',int(x))
            print('y',int(y))
            print('radius',int(radius))
    
    return int(x),int(y)
    # cv2.imshow("img",img_clr)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()



if __name__ == "__main__":
    x,y=cal_pos(image_path)
    print('xq',int(x))
    print('yq',int(y))