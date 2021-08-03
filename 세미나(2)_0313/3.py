import numpy as np
import cv2

img = np.full((400, 400, 3), 255, np.uint8)

text = 'Hello? OpenCV ' + cv2.__version__
# img, text, (x, y), font, font size, color, thickness, lineType
cv2.putText(img, text, (50, 350), cv2.FONT_HERSHEY_TRIPLEX, 0.8, 
            (0, 0, 255), 1, cv2.LINE_AA)

cv2.imshow("img", img)
cv2.waitKey()
cv2.destroyAllWindows()