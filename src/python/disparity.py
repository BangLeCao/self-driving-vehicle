# python3 disparity.py
import cv2
import matplotlib.pyplot as plt
import time


start = time.time()
img_1 = cv2.imread('left.jpg')
img_2 = cv2.imread('right.jpg')

win_size = 5
min_disp = 0
max_disp = 64 #min_disp * 9
num_disp = max_disp - min_disp # Needs to be divisible by 16
#Create Block matching object. 
stereo = cv2.StereoSGBM_create(minDisparity= min_disp,
 numDisparities = num_disp,
 blockSize = 5,
 uniquenessRatio = 0,
 speckleWindowSize = 60,
 speckleRange = 10,
 disp12MaxDiff = 30,
 P1 = 8*3*win_size**2,#8*3*win_size**2,
 P2 = 32*3*win_size**2) #32*3*win_size**2)
 
#Compute disparity map
print ("\nComputing the disparity  map...")
disparity_map = stereo.compute(img_1, img_2)
end = time.time()
print('Execution time:', end - start) # 0.11445856094360352s

#Show disparity map before generating 3D cloud to verify that point cloud will be usable. 
plt.imshow(disparity_map, 'gray')
plt.show()
