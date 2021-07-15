import cv2
import numpy as np
import math
 
img = cv2.imread('frame0000.jpg')

# Create our shapening kernel, it must equal to one eventually
kernel_sharpening = np.array([[-1,-1,-1], 
                              [-1, 9,-1],
                              [-1,-1,-1]])
# applying the sharpening kernel to the input image & displaying it.
sharpen = cv2.filter2D(img, -1, kernel_sharpening)
dst = cv2.Canny(sharpen, 200, 250, None, 3)

linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
if linesP is not None:
    for i in range(0, len(linesP)):
        l = linesP[i][0]
        cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
cv2.imshow('Image Sharpening', img)
cv2.waitKey(0)
cv2.destroyAllWindows()


