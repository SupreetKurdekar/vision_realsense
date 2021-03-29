import numpy as np
import cv2

# Load an color image in grayscale
img = cv2.imread('data/battery_highprec/JPEGImages/0.jpg')
depth_image = cv2.imread("data/battery_highprec/depth/0.png")

# gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# # Otsu's thresholding after Gaussian filtering
# blur = cv2.GaussianBlur(gray_img,(5,5),0)
# ret3,th3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
depth_scale = 0.001
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale


# depth_image = np.asanyarray(aligned_depth_frame.get_data())
print(depth_image.shape)
color_image = img
print(color_image.shape)
# Remove background - Set pixels further than clipping_distance to grey
grey_color = 153
# depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
bg_removed = np.where((depth_image > clipping_distance) | (depth_image <= 0), grey_color, color_image)

# Render images:
#   depth align to color on left
#   depth on right
depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.02), cv2.COLORMAP_JET)
images = np.hstack((bg_removed, depth_colormap))

cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
cv2.imshow('Align Example', images)

# cv2.imshow('image',depth_image)
cv2.waitKey(0)
cv2.destroyAllWindows()