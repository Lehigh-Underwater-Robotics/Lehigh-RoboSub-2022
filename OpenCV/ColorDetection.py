import numpy as np
import cv2
import time

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    width = int(cap.get(3))
    height = int(cap.get(4))

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_color = np.array([0, 125, 175]) #3, 215, 180 // 0, 100, 175 // 3, 226, 226 // 0, 209, 209 // specifically for zemichaels camera 0, 178, 255 //
    upper_color = np.array([14, 255, 255]) #255, 255, 255 // 40, 255, 255

    mask = cv2.inRange(hsv, lower_color, upper_color)

    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('frame', result)
    cv2.imshow('frame2', frame)
    if cv2.waitKey(1) == ord('q'):
        break

print('size', result.size)
print('final shape:', result.shape)

cap.release()
cv2.destroyAllWindows()

#  BGR_color = np.array([[[31, 103, 255]]])
