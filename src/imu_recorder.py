#!/usr/bin/python2.7

import rospy
import os
import numpy as np
from sensor_msgs.msg import Imu

import time
import json


class imuRecorder(object):

    def __init__(self, file_name, file_path):
        self.file_name = file_name
        self.file_path = file_path
        self.start_time = time.time()
        self.data = []
        
        self.prev_time = -1
        self.threshold = 25

    def set_file_name(self, name, path):
        self.file_name = name
        if(path != ""):
            self.file_path = path
        else:
            self.file_path =  os.path.dirname(os.path.abspath(__file__))+"/"

    def save_file(self):
        if (len(recorder.data) != 0):
            print "saving in " + self.file_path
            fp_name = self.file_path + self.file_name  + ".json"

            exists = os.path.isfile(fp_name)
            if exists: 
                print "ATTENTION: the file already existed, name got changed"
                self.file_name = self.file_name + "_" + str(int(time.time()))
                fp_name = self.file_path + self.file_name  + ".json"

            with open(fp_name, 'w') as outfile:
                json_str = json.dumps(self.data, indent=4)
                outfile.write(json_str)
                print "\n" + self.file_name + ".json saved\n"         
        else:
            print "No recording to save"   

    def add_line(self, message):
        self.check_time(message)
        self.data.append(self.to_dict(message))
        
        print self.file_name + "-" + str(len(recorder.data)) + " samples recorded"
    
    def check_time(self, message):
        curr_time = message.header.stamp.secs + message.header.stamp.nsecs/1e9
        if (self.prev_time != -1):            
            diff = (curr_time - self.prev_time)/1e3

            if (diff > self.threshold):
                print self.file_name + "-" + "ATTENTION: delay on IMU data" + str(diff) + " ms"

        self.prev_time = curr_time

    def to_dict(self, msg):
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


        return imu_dict

recorder = imuRecorder("new","./")

def callback(data):
    recorder.add_line(data)
    
def main():
    rospy.init_node('imu_recorder', disable_signals=True)

    file_name = rospy.get_param('~file_name','new')
    file_path = rospy.get_param('/handover_experiment/file_path','./')

    recorder.set_file_name(file_name, file_path)

    rospy.Subscriber("/imu_data", Imu, callback)
    
    r = rospy.Rate(100)
    while True:
        try:
            r.sleep()
        except KeyboardInterrupt:            
            recorder.save_file()
            break
            

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

