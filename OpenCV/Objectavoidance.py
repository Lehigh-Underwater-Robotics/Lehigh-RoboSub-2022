# install opencv "pip install opencv-python"
import cv2
from pymavlink import mavutil
import time
import imutils
import numpy as np
master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')


def wait_conn():
    """
    Sends a ping to stabilish the UDP communication and awaits for a response
    """
    msg = None
    while not msg:
        master.mav.ping_send(
            int(time.time() * 1e6),  # Unix time in microseconds
            0,  # Ping number
            0,  # Request ping of all systems
            0  # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)


def manualControl(x, y, z):
  
    for i in range(200):
        master.mav.manual_control_send(
            master.target_system,
            x,  # x
            y,  # y
            z,  # z
            0,  # r
            0)  # buttons

wait_conn()
print("<<<<<<CONNECTION ESTABLISHED>>>>>>")
boot_time = time.time()
master.wait_heartbeat()
print("<<<<<<<HEARTBEAT RECEIVED>>>>>>")


# ARMING:
master.arducopter_arm()
time.sleep(1)
print("<<<<<<ARMED>>>>>>")
# Setting the mode to manual
mode = 'MANUAL'
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)


 # buttons
# distance from camera to object(face) measured
# centimeter
Known_distance = 150

# width of face in the real world or Object Plane
# centimeter
Known_width = 10

# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (255, 0, 0)

#lower and upper bound colors
lower = np.array([0, 114, 114])
upper = np.array([28, 247, 255])

# defining the fonts
fonts = cv2.FONT_HERSHEY_SIMPLEX
Kernal = np.ones((3, 3), np.uint8)


# focal length finder function
def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):

    # finding the focal length
    focal_length = (width_in_rf_image * measured_distance) / real_width
    return focal_length

# distance estimation function
def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):

    distance = (real_face_width * Focal_Length)/face_width_in_frame

    # return the distance
    return distance

    # return the face width in pixel
def filter(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lb = lower
    ub = upper
    mask = cv2.inRange(gray, lb, ub)   
    edged = cv2.morphologyEx(mask, cv2.MORPH_OPEN, Kernal) 
        # find the contours in the edged image and keep the largest one;
        # we'll assume that this is our piece of paper in the image
    cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    cnts = imutils.grab_contours(cnts)
    c= max(cnts, key = cv2.contourArea)
    marker=cv2.minAreaRect(c)
    box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
    box = np.int0(box)
    cv2.drawContours(image, [box], -1, (0, 255, 0), 2)

    return marker[1][0]

# reading reference_image from directory
ref_image = cv2.imread("OpenCV/bucket.jpg")

# find the face width(pixels) in the reference_image
ref_image_face_width = filter(ref_image)

# get the focal by calling "Focal_Length_Finder"
# face width in reference(pixels),
# Known_distance(centimeters),
# known_width(centimeters)
Focal_length_found = Focal_Length_Finder(
    Known_distance, Known_width, ref_image_face_width)

print(Focal_length_found)

# show the reference image
#cv2.imshow("ref_image", ref_image)

# initialize the camera object so that we
# can get frame from it
cap = cv2.VideoCapture(0)

# looping through frame, incoming from
# camera/video
while True:
    _, frame = cap.read()
    frame2 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)         ##BGR to HSV
    lb = lower
    ub = upper
    mask = cv2.inRange(frame2, lb, ub)                      ##Create Mask

    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, Kernal)        ##Morphology

    res = cv2.bitwise_and(frame, frame, mask= opening)             ##Apply mask on original image

    contours, hierarchy = cv2.findContours(opening, cv2.RETR_LIST,      ##Find contours
                                           cv2.CHAIN_APPROX_NONE)[-2:]
    if len(contours) != 0:

    # calling face_data function to find
    # the width of face(pixels) in the frame
        face_width_in_frame = filter(frame)

        # check if the face is zero then not
        # find the distance
        if face_width_in_frame != 0:
        
            # finding the distance by calling function
            # Distance distnace finder function need
            # these arguments the Focal_Length,
            # Known_width(centimeters),
            # and Known_distance(centimeters)
            Distance = Distance_finder(
                Focal_length_found, Known_width, face_width_in_frame)

            # draw line as background of text
            cv2.line(frame, (30, 30), (320, 30), RED, 32)
            cv2.line(frame, (30, 30), (320, 30), WHITE, 28)
    
            # Drawing Text on the screen
            cv2.putText(
                frame, f"Distance to bucket: {round(Distance,2)} CM", (30, 35),
            fonts, 0.6, RED, 2)
            if Distance<=150: 
                cv2.line(frame, (450, 30), (600, 30), RED, 32)
                cv2.line(frame, (450, 30), (600, 30), WHITE, 28)
                cv2.putText(
                frame, f"ROV COLLISION ALLERT", (450, 35),
            fonts, 0.5, RED, 2)
                master.arducopter_disarm()
                print(">>>>>ROV DISARMED<<<<<<<")
                
        # show the frame on the screen
    else:
        cv2.line(frame, (30, 30), (320, 30), GREEN, 32)
        cv2.line(frame, (30, 30), (320, 30), WHITE, 28)
        cv2.putText(
                frame, f"Bucket not detected", (30, 35),
            fonts, 0.6, GREEN, 2)
    cv2.imshow("frame", frame)
    # quit the program if you press 'q' on keyboard
    if cv2.waitKey(1) == ord("q"):
        break
# closing the camera
cap.release()

# closing the the windows that are opened
cv2.destroyAllWindows()
