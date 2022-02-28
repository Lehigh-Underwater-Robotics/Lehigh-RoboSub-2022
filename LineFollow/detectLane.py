import numpy as np
import cv2

cap = cv2.VideoCapture('lineDetect.mp4')


def findColor(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([0, 0, 0])
    upper_blue = np.array([32, 79, 134])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    return mask


def getContours(frame, detect_black):
    contours, hierarchy = cv2.findContours(
        detect_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    biggest = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(biggest)

    cx = x + w//2
    cy = y + h//2

    cv2.drawContours(frame, biggest, -1, (255, 0, 255), 7)
    cv2.circle(frame, (cx, cy), 10, (0, 255, 0), cv2.FILLED)


while True:
    ret, frame = cap.read()
    frame = frame[:-1, 760:1160]
    width = int(cap.get(3))
    height = int(cap.get(4))

    detect_black = findColor(frame)

    getContours(frame, detect_black)

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cap.destroyAllWindows()
