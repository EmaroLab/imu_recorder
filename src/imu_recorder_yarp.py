#!/usr/bin/python2.7

from sensor_msgs.msg import Imu
import numpy as np
import rospy
import yarp
import time
import json
import os


class imuRecorder(object):

    def __init__(self, file_name, file_path):
        self.file_name = file_name
        self.file_path = file_path
        self.data = []

        self.recording = True
        self.prev_time = -1
        self.file_counter = 0

        self.threshold = 25
    
    # Enable disable new samples recording
    def set_recording(self, flag):
        self.recording = flag
        
    # Saves recorded data and clears all the recording
    def save_reset(self):
        self.set_recording(False)
        self.save_file()
        self.data = []
        self.prev_time = -1
        self.file_counter += 1
        self.set_recording(True)

    # Sets file and path name
    def set_file_name(self, name, path):
        self.file_name = name
        if(path != ""):
            temp_directory = path + "/Experiment_"
        else:
            temp_directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))+"/Data/Experiment_"
        
        i = 0
        while os.path.isdir(temp_directory + str(i)):
            i = i+1
        self.file_path = temp_directory + str(i) + "/"
        os.makedirs(self.file_path)

    # Saves recording to a json file 
    def save_file(self):
        if (len(recorder.data) != 0):
            print "saving in " + self.file_path
            fp_name = self.file_path + self.file_name + "_" + str(self.file_counter) + ".json"

            exists = os.path.isfile(fp_name)
            if exists: 
                print "ATTENTION: the file already existed, name got changed"
                fp_name = self.file_path + self.file_name + "_" + str(self.file_counter) + "_" + str(int(time.time()))  + ".json"

            with open(fp_name, 'w') as outfile:
                json_str = json.dumps(self.data, indent=4)
                outfile.write(json_str)
                print "\n" + fp_name + " saved\n"         
        else:
            print "No recording to save"   

    # Adds a line to the data recordings
    def add_line(self, message, yarp_time):
        if self.recording:
            self.check_time(message)
            self.data.append(self.to_dict(message, yarp_time))
        
            print self.file_name + "-" + str(len(recorder.data)) + " samples recorded"
    
    # Checks the delay between successive lines and return a warning if it is higher then a threshold
    def check_time(self, message):
        curr_time = message.header.stamp.secs + message.header.stamp.nsecs/1e9
        if (self.prev_time != -1):            
            diff = (curr_time - self.prev_time)/1e3

            if (diff > self.threshold):
                print self.file_name + "-" + "ATTENTION: delay on IMU data" + str(diff) + " ms"

        self.prev_time = curr_time

    # Converts the ros message into a dictionary
    def to_dict(self, msg, yarp_time):
        imu_dict = {}
        
        imu_dict['linear_acceleration_x'] = msg.linear_acceleration.x
        imu_dict['linear_acceleration_y'] = msg.linear_acceleration.y
        imu_dict['linear_acceleration_z'] = msg.linear_acceleration.z
        
        imu_dict['angular_velocity_x'] = msg.angular_velocity.x
        imu_dict['angular_velocity_y'] = msg.angular_velocity.y
        imu_dict['angular_velocity_z'] = msg.angular_velocity.z 
        
        imu_dict['stamp_secs'] = msg.header.stamp.secs
        imu_dict['stamp_nsecs'] = msg.header.stamp.nsecs
        imu_dict['stamp_android'] = msg.header.frame_id
        
        imu_dict['yarp_time'] = yarp_time

        return imu_dict

# yarp initialisation
yarp.Network.init()
Stamp = yarp.Stamp()

# recorder initialisation
recorder = imuRecorder("new","")

# ros callback
def callback(data):
    Stamp.update()
    recorder.add_line(data, Stamp.getTime())
    
def main():
    # ros node initialisation
    rospy.init_node('imu_recorder', disable_signals=True)

    # reading file and path name from the ros parameter server
    file_name = rospy.get_param('~file_name','recording')
    file_path = rospy.get_param('~file_path','')

    # setting the file and path name for the recorder
    recorder.set_file_name(file_name, file_path)

    # subscribing to the inertial data topic
    rospy.Subscriber("/imu_data", Imu, callback)
    
    # setting yarp port for key pressure
    inputPortname = '/smartwatch/keyboard:i'
    inputPort = yarp.BufferedPortBottle()
    inputPort.open(inputPortname)

    # setting keyboard letters behavior
    reset = rospy.get_param('~reset_letter','nan')
    stop = rospy.get_param('~stop_letter','nan')

    if reset == "nan" or stop == "nan":
        print "\n ERROR: set the reset and stop parameter in the launch file \n"
        return

    r = rospy.Rate(100)
    while True:
        try:
            # reading yarp message
            messageBottle = inputPort.read(False)

            # handling keyboard pressure
            if messageBottle != None:
                receivedMessage = messageBottle.toString()
                if receivedMessage == reset:
                    recorder.save_reset()    
                elif receivedMessage == stop:
                    recorder.set_recording(False)
                    recorder.save_file()
                    break            
            r.sleep()
        except KeyboardInterrupt:    
            recorder.set_recording(False)
            recorder.save_file()
            inputPort.close()     
            break
            

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

