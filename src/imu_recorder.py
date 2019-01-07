#!/usr/bin/python

import rospy
import os
from mqtt_ros_bridge.msg import *
import numpy as np

import time
import json


class imuRecorder(object):

    def __init__(self, file_name, file_path):
        self.file_name = file_name
        self.file_path = file_path
        self.start_time = time.time()
        self.data = []
        
        self.last_acc_time = -1
        self.last_gyro_time = -1
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
        if (self.last_acc_time != -1):
            diff_acc = message.linear_acceleration[0].time.data - self.last_acc_time
            diff_gyro = message.angular_velocity[0].time.data -self.last_gyro_time
            if (diff_acc > self.threshold):
                print self.file_name + "-" + "ATTENTION: delay on linear acceleration" + str(diff_acc) + " ms"

            if (diff_gyro > self.threshold):
                print self.file_name + "-" + "ATTENTION: delay on angular velocity " + str(diff_gyro) + " ms"

        self.last_acc_time = message.linear_acceleration[-1].time.data
        self.last_gyro_time = message.angular_velocity[-1].time.data

    def to_dict(self, msg):
        n_acc = len(msg.linear_acceleration)
        n_vel = len(msg.angular_velocity)
        imu_dict = {}
        imu_dict['linear_acceleration'] = []
        imu_dict['angular_velocity'] = []
        imu_dict['timestamp'] = time.time() - self.start_time
        for i in range(n_acc):
            a_vector_dict = {}        

            a_vector_dict['x'] = msg.linear_acceleration[i].vector.x
            a_vector_dict['y'] = msg.linear_acceleration[i].vector.y
            a_vector_dict['z'] = msg.linear_acceleration[i].vector.z
            a_vector_dict['t'] = msg.linear_acceleration[i].time.data
            
            imu_dict['linear_acceleration'].append(a_vector_dict)

        for i in range(n_vel):
            v_vector_dict = {}

            v_vector_dict['x'] = msg.angular_velocity[i].vector.x
            v_vector_dict['y'] = msg.angular_velocity[i].vector.y
            v_vector_dict['z'] = msg.angular_velocity[i].vector.z
            v_vector_dict['t'] = msg.angular_velocity[i].time.data
            
            imu_dict['angular_velocity'].append(v_vector_dict)


        return imu_dict

recorder = imuRecorder("new","./")

def callback(data):
    recorder.add_line(data)
    
def main():
    rospy.init_node('imu_recorder', disable_signals=True)

    file_name = rospy.get_param('~file_name','new')
    file_path = rospy.get_param('~file_path','./')

    recorder.set_file_name(file_name, file_path)

    rospy.Subscriber("/imu_data", ImuPackage, callback)
    
    r = rospy.Rate(10)
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

