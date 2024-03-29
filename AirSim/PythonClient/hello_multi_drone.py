"""
For connecting to the AirSim drone environment and testing API functionality
"""

import os
import sys
import tempfile
import pprint

from AirSimClient import *

if len(sys.argv) != 2:
    print("Usage: python PythonClient/hello_multi_drone.py <number of spawns>")
    exit()

num_drones  = int(sys.argv[1])
base_port   = 41451

for i in range(num_drones):
    port = base_port + i
    # connect to the AirSim simulator
    client = MultirotorClient(port=port)
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    print("Drone Client on Port Number %d" % port)

    state = client.getMultirotorState()
    s = pprint.pformat(state)
    print("state: %s" % s)

    AirSimClientBase.wait_key('Press any key to takeoff')
    client.takeoff()

    state = client.getMultirotorState()
    print("state: %s" % pprint.pformat(state))

    AirSimClientBase.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
    client.moveToPosition(-10, 10, -10, 5)

    client.hover()

    state = client.getMultirotorState()
    print("state: %s" % pprint.pformat(state))

    AirSimClientBase.wait_key('Press any key to take images')
    # get camera images from the car
    responses = client.simGetImages([
        ImageRequest(0, AirSimImageType.DepthVis),  #depth visualiztion image
        ImageRequest(1, AirSimImageType.DepthPerspective, True), #depth in perspective projection
        ImageRequest(1, AirSimImageType.Scene), #scene vision image in png format
        ImageRequest(1, AirSimImageType.Scene, False, False)])  #scene vision image in uncompressed RGBA array
    print('Retrieved images: %d' % len(responses))

    tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
    print ("Saving images to %s" % tmp_dir)
    try:
        os.makedirs(tmp_dir)
    except OSError:
        if not os.path.isdir(tmp_dir):
            raise

    for idx, response in enumerate(responses):

        filename = os.path.join(tmp_dir, str(idx))

        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            AirSimClientBase.write_pfm(os.path.normpath(filename + '.pfm'), AirSimClientBase.getPfmArray(response))
        elif response.compress: #png format
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            AirSimClientBase.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
        else: #uncompressed array
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) #get numpy array
            img_rgba = img1d.reshape(response.height, response.width, 4) #reshape array to 4 channel image array H X W X 4
            img_rgba = np.flipud(img_rgba) #original image is fliped vertically
            img_rgba[:,:,1:2] = 100 #just for fun add little bit of green in all pixels
            AirSimClientBase.write_png(os.path.normpath(filename + '.greener.png'), img_rgba) #write to png

    AirSimClientBase.wait_key('Press any key to reset to original state')
    client.armDisarm(False)

    #restore to original state
    client.reset()

    # that's enough fun for now. let's quit cleanly
    client.enableApiControl(False)
