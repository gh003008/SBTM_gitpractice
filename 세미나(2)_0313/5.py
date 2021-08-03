import sys
import cv2


cap = cv2.VideoCapture('video.mp4')

if not cap.isOpened():
    print("Camera open failed!")
    sys.exit()

w = round(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

'''
Fourcc: 동영상 파일의 코덱, 압축 방식, 생성, 픽셀 포맷 등을 정의하는 정수 값
DIVX, XVID, FMP4, X264, MJPG
'''

fourcc = cv2.VideoWriter_fourcc(*'DIVX') # *'DIVX' == 'D', 'I', 'V', 'X'
delay = round(1000 / fps)

out = cv2.VideoWriter('output.avi', fourcc, fps, (w, h))

if not out.isOpened():
    print('File open failed!')
    cap.release()
    sys.exit()

while True:
    ret, frame = cap.read()

    if not ret:
        break

    edge = cv2.Canny(frame, 50, 150)
    edge_color = cv2.cvtColor(edge, cv2.COLOR_GRAY2BGR)

    out.write(edge_color)

    cv2.imshow('edge_color', edge_color)

    if cv2.waitKey(delay) == 27:
        break

cap.release()
out.release()
cv2.destroyAllWindows()
