import cv2
import sys


# OpenCV는 영상 데이터를 numpy.ndarray로 표현한다
img_gray = cv2.imread('joyuri.jpg', cv2.IMREAD_GRAYSCALE)
img_color = cv2.imread('joyuri.jpg', cv2.IMREAD_COLOR)

if img_gray is None or img_color is None:
    print('Image load failed!')
    sys.exit()


# (h, w) or (h, w, 3)
print('type(img_gray):', type(img_gray))
print('img_gray.shape:', img_gray.shape)
print('img_color.shape:', img_color.shape)
print('img_gray.dtype:', img_gray.dtype)
print('img_color.dtype:', img_color.dtype)  


# 영상의 크기 참조
h, w = img_color.shape[:2]
print('img_color size: {} x {}'.format(w, h))


if len(img_gray.shape) == 2:
    print('img_gray is a grayscale image')
elif len(img_gray.shape) == 3:
    print('img_gray is a truecolor image')

cv2.imshow('img_gray', img_gray)
cv2.imshow('img_color', img_color)
cv2.waitKey()
cv2.destroyAllWindows()