# import os
# import cv2
# import numpy as np
# import math
# filename='/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/bolt.jpg'

# smarties = cv2.imread(filename)
# gray_img= cv2.cvtColor(smarties,cv2.COLOR_BGR2GRAY)
# #进行中值滤波
# img = cv2.medianBlur(gray_img,5)
# circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=35,minRadius=0,maxRadius=0)
# #对数据进行四舍五入变为整数
# circles = np.uint16(np.around(circles))


# x_position=0
# y_position=0


# for i in circles[0,:]:
#     #画出来圆的边界
#     cv2.circle(smarties,(i[0],i[1]),i[2],(0,0,255),2)
#     #画出来圆心
#     cv2.circle(smarties,(i[0],i[1]),2,(0,255,255),3)
#     x_position=i[0]
#     y_position=i[1]



# print( "x={0},y={1}" .format(x_position, y_position))
# cv2.imshow("Circle",smarties)
# cv2.waitKey()
# cv2.destroyAllWindows()




# # 找到harris corner
# gray = np.float32(gray_img)
# dst = cv2.cornerHarris(gray, 2, 3, 0.04)
# dst = cv2.dilate(dst, None)
# ret, dst = cv2.threshold(dst, 0.01 * dst.max(), 255, 0)
# dst = np.uint8(dst)
 
# # 找到质心
# ret, labels, states, centroids = cv2.connectedComponentsWithStats(dst)
# # 定义停止和改进角落的标注
# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
# corners = cv2.cornerSubPix(gray, np.float32(centroids), (5, 5), (-1, -1), criteria)


# for corner    in range(len(corners)) :
#     # print(corners[corner])
#     distance=math.sqrt( math.pow(x_position-corners[corner][0],2)+math.pow(y_position-corners[corner][1],2))
#     if distance<20:
#         # 现在绘制它们
#         res = np.hstack((centroids, corners))
#         res = np.int0(res)
#         img[res[:, 1], res[:, 0]] = [0, 0, 255]  # 红色
#         img[res[:, 3], res[:, 2]] = [0, 255, 0]  # 绿色
#         print(distance)

#         # cv2.imwrite('subpixel5.png', img)
#         # cv2.imshow('res', img)
#         # cv2.waitKey(0) & 0xFF
#         # cv2.destroyAllWindows()
        

# # # 现在绘制它们
# # res = np.hstack((centroids, corners))
# # res = np.int0(res)
# # img[res[:, 1], res[:, 0]] = [0, 0, 255]  # 红色
# # img[res[:, 3], res[:, 2]] = [0, 255, 0]  # 绿色

# #将KeyPoint格式数据中的xy坐标提取出来。
# # print(corners)

# cv2.imwrite('subpixel5.png', img)
# cv2.imshow('res', img)
# cv2.waitKey(0) & 0xFF
# cv2.destroyAllWindows()



















# # import cv2
# # import numpy as np
# # from matplotlib import pyplot as plt
# # # Import the modules

# # testID='/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/bolt.jpg'

# # # reading image
# # img = cv2.imread(testID)

# # # converting image into grayscale image
# # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# # im_gray = cv2.GaussianBlur(gray, (5, 5), 0)

# # # setting threshold of gray image
# # _, threshold = cv2.threshold(im_gray, 127, 255, cv2.THRESH_BINARY)

# # # using a findContours() function
# # _img, contours, hierarchy =  cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# # for contour in contours:

# # 	# cv2.approxPloyDP() function to approximate the shape
# # 	approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
	
# # 	# using drawContours() function
# # 	cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)

# # 	# finding center point of shape
# # 	M = cv2.moments(contour)
# # 	if M['m00'] != 0.0:
# # 		x = int(M['m10']/M['m00'])
# # 		y = int(M['m01']/M['m00'])

# # 	# putting shape name at center of each shape
# # 	if  len(approx) == 6:
# # 		cv2.putText(img,'Hexagon',(x, y),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
# #         print( "x={0},y={1}" .format(  x, y  ))


# # # displaying the image after drawing contours
# # cv2.imshow('shapes', img)
# # cv2.waitKey(0)
# # cv2.destroyAllWindows()







import os
import cv2
import numpy as np

testID='/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/bolt.jpg'

smarties = cv2.imread(testID)
gray_img= cv2.cvtColor(smarties,cv2.COLOR_BGR2GRAY)
#进行中值滤波
img = cv2.medianBlur(gray_img,5)


circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=35,minRadius=0,maxRadius=0)
#对数据进行四舍五入变为整数
circles = np.uint16(np.around(circles))

x_position=0
y_position=0

for i in circles[0,:]:
    #画出来圆的边界
    cv2.circle(smarties,(i[0],i[1]),i[2],(0,0,255),2)
    #画出来圆心
    cv2.circle(smarties,(i[0],i[1]),2,(0,255,255),3)
    x_position=i[0]
    y_position=i[1]



print( "x={0},y={1}" .format(x_position, y_position))
cv2.imshow("Circle",smarties)
cv2.waitKey()
cv2.destroyAllWindows()
























# import cv2
# import numpy as np
# from matplotlib import pyplot as plt
# # Import the modules

# testID='/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/bolt.jpg'

# # reading image
# img = cv2.imread(testID)

# # converting image into grayscale image
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# im_gray = cv2.GaussianBlur(gray, (5, 5), 0)

# # setting threshold of gray image
# _, threshold = cv2.threshold(im_gray, 127, 255, cv2.THRESH_BINARY)

# # using a findContours() function
# _img, contours, hierarchy =  cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# for contour in contours:

# 	# cv2.approxPloyDP() function to approximate the shape
# 	approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
	
# 	# using drawContours() function
# 	cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)

# 		x = int(M['m10']/M['m00'])
# 		y = int(M['m01']/M['m00'])

# 	# putting shape name at center of each shape
# 	if  len(approx) == 6:
# 		cv2.putText(img,'Hexagon',(x, y),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
#         print( "x={0},y={1}" .format(  x, y  ))


# # displaying the image after drawing contours
# cv2.imshow('shapes', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()




