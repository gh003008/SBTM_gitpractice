import sys
import cv2


# 카메라(비디오) 열기
cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture('video.mp4')

if not cap.isOpened():
    print("Camera open failed!")
    sys.exit()

# 카메라 프레임 크기(double type) 출력
print('Frame width:', int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
print('Frame height:', int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

# FPS
fps = cap.get(cv2.CAP_PROP_FPS)
print('FPS:', fps)

delay = round(1000 / fps)

'''
# 카메라 프레임 크기 변경(카메라만 가능, 정해진 값만 설정 가능하다)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

print('Frame width:', int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
print('Frame height:', int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
'''

# 카메라 프레임 처리
while True:
    ret, frame = cap.read()

    if not ret: # boolean
        break

    '''
    inversed = ~frame  # 반전
    edge = cv2.Canny(frame, 50, 150)
    '''

    cv2.imshow('frame', frame)

    '''
    cv2.imshow('inversed', inversed)
    cv2.imshow('edge', edge)
    '''

    if cv2.waitKey(delay) == 27:   # ESC
        break

cap.release()
cv2.destroyAllWindows()
