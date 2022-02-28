# install opencv "pip install opencv-python"
import cv2

import imutils
import numpy as np
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK


if __name__ == '__main__':
    # Create the video object
    # Add port= if is necessary to use a different one
    video = Video()



cap = video.frame()

cv2.imshow("frame", cap)
       
        

# distance from camera to object(face) measured
# centimeter
Known_distance = 40

# width of face in the real world or Object Plane
# centimeter
Known_width = 10

# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (255, 0, 0)

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
    lb = np.array([153, 119, 212])
    ub = np.array([255, 255, 255])
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
ref_image = cv2.imread("bucket.jpg")

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

# looping through frame, incoming from
# camera/video
while True:
    frame=cap
    # reading the frame from camera
    frame2 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)         ##BGR to HSV
    lb = np.array([153, 119, 212])
    ub = np.array([255, 255, 255])

    mask = cv2.inRange(frame2, lb, ub)                      ##Create Mask

    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, Kernal)        ##Morphology
  

    res = cv2.bitwise_and(frame, frame, mask= opening)             ##Apply mask on original image

    contours, hierarchy, _ = cv2.findContours(opening, cv2.RETR_LIST,      ##Find contours
                                           cv2.CHAIN_APPROX_NONE)

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

            if Distance<=50: 
                cv2.line(frame, (1000, 30), (1220, 30), RED, 32)
                cv2.line(frame, (1000, 30), (1220, 30), WHITE, 28)
                cv2.putText(
                frame, f"ROV COLLISION ALLERT", (1000, 35),
            fonts, 0.6, RED, 2)
                
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

