from PIL import Image
import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import cv2
import matplotlib.pyplot as plt
import numpy as np

filename='/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/bolt.jpg'


img = cv2.imread(filename)
GrayImage=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
GrayImage= cv2.medianBlur(GrayImage,5)
ret,th1 = cv2.threshold(GrayImage,127,255,cv2.THRESH_BINARY)
th2 = cv2.adaptiveThreshold(GrayImage,255,cv2.ADAPTIVE_THRESH_MEAN_C,  cv2.THRESH_BINARY,3,5)  
th3 = cv2.adaptiveThreshold(GrayImage,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,3,5)


kernel = np.ones((5,5),np.uint8)
erosion = cv2.erode(th2,kernel,iterations=1)
dilation = cv2.dilate(erosion,kernel,iterations=1)

imgray=cv2.Canny(erosion,30,100, apertureSize=5)  #apertureSize参数默认其实就是3

circles = cv2.HoughCircles(imgray,cv2.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=20,maxRadius=50)

circles = np.uint16(np.around(circles))
for i in circles[0,:]:
    # draw the outer circle
    cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
print(len(circles[0,:]))

cv2.imshow('detected circles',img)

cv2.waitKey(0)
cv2.destroyAllWindows()