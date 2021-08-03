import numpy as np
import cv2


# 새 영상 생성하기 (h, w) or (h, w, 3) / dtype 명시해줘야 함
img1 = np.empty((240, 320), dtype=np.uint8)       # grayscale image, 임의의 값으로 초기화
img2 = np.zeros((240, 320, 3), dtype=np.uint8)    # color image
img3 = np.ones((240, 320), dtype=np.uint8) * 255  # white
img4 = np.full((240, 320, 3), (0, 255, 255), dtype=np.uint8)  # yellow

cv2.imshow('img1', img1)
cv2.imshow('img2', img2)
cv2.imshow('img3', img3)
cv2.imshow('img4', img4)
cv2.waitKey()
cv2.destroyAllWindows()



# 영상 복사
img1 = cv2.imread('joyuri.jpg')

img2 = img1
img3 = img1.copy()

# img1.fill(255)
img1[:50, :] = (0, 255, 255)

cv2.imshow('img1', img1)
cv2.imshow('img2', img2)
cv2.imshow('img3', img3)
cv2.waitKey()
cv2.destroyAllWindows()


# 부분 영상 추출 -> indexing, slicing
img1 = cv2.imread('joyuri.jpg')

img2 = img1[40:200, 30:300]  # numpy.ndarray의 슬라이싱
img3 = img1[40:200, 30:300].copy()

img2.fill(0)

cv2.imshow('img1', img1)
cv2.imshow('img2', img2)
cv2.imshow('img3', img3)
cv2.waitKey()
cv2.destroyAllWindows()



# ROI(Region Of Interest, 관심영역) 설정 가능!
# circle
img1 = cv2.imread('joyuri.jpg')

img2 = img1[200:400, 30:300]  # numpy.ndarray의 슬라이싱
img3 = img1[200:400, 30:300].copy()

cv2.circle(img2, (50, 50), 20, (0, 0, 255), 2)

cv2.imshow('img1', img1)
cv2.imshow('img2', img2)
cv2.imshow('img3', img3)
cv2.waitKey()
cv2.destroyAllWindows()
