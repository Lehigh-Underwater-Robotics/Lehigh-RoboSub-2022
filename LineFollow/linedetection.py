
import numpy as np
import cv2
import math


def findColor(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([0, 0, 0])
    upper_blue = np.array([32, 79, 134])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    result = cv2.bitwise_and(frame, frame, mask=mask)

    return result


def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    #channel_count = img.shape[2]
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def drawLines(img, lines):
    img = np.copy(img)
    lineImage = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    for line in lines:
        for x1, y1, x2, y2 in line:
            angle = math.atan((y2-y1)/(x2-x1))
            if (angle < 85):
                cv2.line(lineImage, (x1, y1), (x2, y2), (0, 255, 0), 3)

    img = cv2.addWeighted(img, 0.8, lineImage, 1, 0.0)
    return img


#image = cv2.imread("lane.jpg")

# converting the image into rgb format
#image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
def process(image):
    height = image.shape[0]
    width = image.shape[1]

    region_vertices = [(0, height),
                       (width/2, height/2),
                       (width, height)]

    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    canny_image = cv2.Canny(gray_image, 100, 200)
    cropped_image = region_of_interest(canny_image,
                                       np.array([region_vertices], np.int32))

    lines = cv2.HoughLinesP(cropped_image, rho=6, theta=np.pi/60,
                            threshold=160, lines=np.array([]), minLineLength=40, maxLineGap=25)

    lined_image = drawLines(image, lines)

    return lined_image


cap = cv2.VideoCapture('lineDetect.mp4')
n = 0
while(cap.isOpened(), n):
    ret, frame = cap.read()
    frame = process(frame)
    cv2.imshow('video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()


#cv2.imshow("lane", lined_image)
# cv2.waitKey(0)
