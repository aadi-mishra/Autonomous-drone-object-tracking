# use open cv to show new images from AirSim 

from AirSimClient import *
# requires Python 3.5.3 :: Anaconda 4.4.0
# pip install opencv-python
import cv2
import time
import math
import sys
from threading import Timer,Thread,Event
import datetime as dt1
import time
import pprint

client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoff()

# you must first press "1" in the AirSim view to turn on the depth capture

# get depth image
yaw = 0
pi = 3.14159265483
vx = 0
vy = 0
driving = 0
help = False



def computeCoordinates(center_x,center_y):
    global rel_altitude
    
    FRAME_HEIGHT = 480.0
    FRAME_WIDTH = 640.0 
    heightofcar=0.23
    altitude=rel_altitude
    xview = 90
    yview = 90
    xgimbal=0
    ygimbal=0
    xground = (altitude-heightofcar)*((math.tan(math.radians(xgimbal+0.5*xview)))-(math.tan(math.radians(xgimbal-0.5*xview))))
    yground = (altitude-heightofcar)*((math.tan(math.radians(ygimbal+0.5*yview)))-(math.tan(math.radians(ygimbal-0.5*yview))))
    x_px = (320-center_x)
    y_px = (240-center_y)
    coordinates_x = (x_px*xground/FRAME_WIDTH)
    coordinates_y = y_px*yground/FRAME_HEIGHT
    return coordinates_x,coordinates_y



def segment_colour(frame):
    
    hsv_roi =  cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_1 = cv2.inRange(hsv_roi, np.array([160.,120.,40.]), np.array([190.,255.,255.]))
    ycr_roi=cv2.cvtColor(frame,cv2.COLOR_BGR2YCrCb,)
    mask_2=cv2.inRange(ycr_roi, np.array((30.,165.,0.)), np.array((255.,255.,255.)))
    mask = mask_1 | mask_2 
    kern_dilate = np.ones((8,8),np.uint8)
    kern_erode  = np.ones((3,3),np.uint8)
    mask= cv2.erode(mask,kern_erode)
    mask=cv2.dilate(mask,kern_dilate)
    return mask

def find_blob(blob):
    
    largest_contour=0
    cont_index=0
    _,contours,_= cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    print(contours)
    for idx, contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if (area > largest_contour):
            largest_contour=area
            cont_index=idx
    r=(0,0,2,2)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])

    return r

AirSimClientBase.wait_key('Press any key to takeoff')
b_new=client.getGpsLocation()
alt_base=b_new.altitude
print ("alt_base:",alt_base)
client.takeoff()

AirSimClientBase.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
client.moveToPosition(-0, 0, -30, 5)

AirSimClientBase.wait_key('Press any key to start tracking')
b_new=client.getGpsLocation()
alt_h=b_new.altitude
print ("alt_height:",alt_h)
a=client.getPosition()
print("position:",a.z_val)
PRY=client.getPitchRollYaw()
print("angles",PRY)
'''
n1=dt.datetime.now()
start_time = time.time()

# your code
global prev_time
prev_time=0


class MyThread(Thread):
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event

    def run(self):
        global prev_time
        while not self.stopped.wait(0.001):
            now_time=time.time()
            elapsed_time = now_time-prev_time
            print("my thread",elapsed_time)
            prev_time=now_time

stopFlag = Event()
thread = MyThread(stopFlag)
thread.start()
'''
# this will stop the timer

while True:
    result = client.simGetImage(3, AirSimImageType.Scene)
    
    
    if (result == "\0"):
        if (not help):
            help = True
            print("Please press '1' in the AirSim view to enable the Depth camera view")
    else:

        lis = np.arange(-1.05,1.06,0.01)
        
        #client.setCameraOrientation(3,AirSimClientBase.toQuaternion(0, 0, 0)) 
        
        rawImage = np.fromstring(result, np.int8)
        png = cv2.imdecode(rawImage, cv2.IMREAD_UNCHANGED)
        mask_red=segment_colour(png)
        loct=find_blob(mask_red)
        x,y,w,h=loct
        a=client.getPosition()

       
        rel_altitude=-a.z_val
        #rel_altitude=-a.z_val
        if (w*h<=10):
            detected = 0
           
        else:
            detected=1
            
            center_x = x + (w / 2)
            center_y = y + (h / 2)
            contour_width = w
            contour_height = h
            
          
            [coordinates_x,coordinates_y]=computeCoordinates(center_x,center_y)
            
            target_coordinates = [x,y]
            print(target_coordinates)


            
            ###########################################################################
            # For gimbal control
            # define the parameters
            height = rel_altitude - 0.23
            print("Height :",height)
            yaw_fov_x = 90
            pitch_fov_y = 90
            # x_gimbal = 0
            # y_gimbal = 0
            # Image Dimensions
            length, width = png.shape[0:2]
            print("Length :", length, "Width :", width)
            # get the bottom right(maximum extremes) of the image window
            # bottom_right = png[length-1, width-1]
            # print("Extremes :", bottom_right)

            # parseList = [0] * len(target_coordinates)

            # # Normalizing pixel coordinates
            # for i in range (0, len(target_coordinates)):
            #     parseList[i] = target_coordinates[i]
            # print(parseList)

            # for i in range (0, len(target_coordinates)):
            #     x_normalized = (parseList[i])/bottom_right[0]
            #     y_normalized = (parseList[1])/bottom_right[1]

            radx = math.radians(yaw_fov_x/2)
            rady = math.radians(pitch_fov_y/2) 

            x_norm = height * math.tan(radx)
            y_norm = height * math.tan(rady)

            # x_norm = height * (math.tan(x_gimbal + radx) - math.tan(x_gimbal - radx))
            # y_norm = height * (math.tan(y_gimbal + rady) - math.tan(y_gimbal - rady))

            degree = [0] * 2
            degree[0] = math.atan(x_norm/height)
            degree[1] = math.atan(y_norm/height)
            
            #############################################
            # normalise tracking coordinates
            target_x = target_coordinates[0] - 0.5*width
            target_y = target_coordinates[1] - 0.5*length
            kpx = x_norm/(0.5*width)
            kpy = y_norm/(0.5*length)


            target_x_norm = target_x * kpx
            target_y_norm = target_y * kpy
            theta = math.atan(target_x_norm/height)
            phi = math.atan(-target_y_norm/height)
           
            
            print("theta :", theta)
            print("phi :", phi)

            print("X Normalised :",x_norm , "Y Normalised :", y_norm)  

            deg = pprint.pformat(degree)
            print("degree", deg)


            client.setCameraOrientation(3,AirSimClientBase.toQuaternion(0, 0, theta))

            camera_info = client.getCameraInfo(3)
            orientation = AirSimClientBase.toEulerianAngle(camera_info.pose.orientation)
            x_gimbal = orientation[2]
            y_gimbal = orientation[0]
          

            cv2.rectangle(png, (x,y), ((x + w), (y + h)), 255, 2)
            cv2.circle(png, (int(center_x), int(center_y)), 3, (0, 110, 255), -1)
            
            
            
        collision=client.getCollisionInfo()
        
        cv2.imshow("Top", png)
        

    key = cv2.waitKey(1) & 0xFF;
    if (key == 27 or key == ord('q') or key == ord('x')):
        #stopFlag.set()
        break;






