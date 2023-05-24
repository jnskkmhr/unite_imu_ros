#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu

class IMUFusionNode:
    def __init__(self, camera_name):
        rospy.init_node('imu_fusion_node', anonymous=True)
        self.camera_name = camera_name
        self.imu_pub = rospy.Publisher('/'+camera_name+'/imu', Imu, queue_size=100)
        self.gyro_sub = rospy.Subscriber('/'+camera_name+'/gyro/sample', Imu, self.gyro_callback)
        self.accel_sub = rospy.Subscriber('/'+camera_name+'/accel/sample', Imu, self.accel_callback)

        self.accel_prev = None #  accel_t0
        self.accel_prev_header = None # header_t0
        self.accel_prev_time = None #t0
        self.accel_current = None # accel_t1
        self.accel_current_header = None # header_t1
        self.accel_current_time = None # t1
        self.gyro_data = None # gyro_t
        self.gyro_header = None # header_t
        self.gyro_time = None # t

    def gyro_callback(self, data):
        self.gyro_data = data.angular_velocity
        self.gyro_header = data.header
        self.gyro_time = data.header.stamp.secs + 1e-9*data.header.stamp.nsecs
        self.publish_imu() # everytime when gyro is streamed, publish imu (gyro and interpolated accel) message
    
    def accel_callback(self, data):
        self.accel_prev = self.accel_current
        self.accel_prev_header = self.accel_current_header
        self.accel_prev_time = self.accel_current_time
        self.accel_current = data.linear_acceleration
        self.accel_current_header = data.header
        self.accel_current_time = data.header.stamp.secs + 1e-9*data.header.stamp.nsecs
    
    def publish_imu(self):
        # interpolate accel data to gyro data
        # c.f. https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc#L448-L485
        interpolated_imu = Imu()
        if self.accel_prev_time is None or self.accel_current_time is None:
            print("no accel data is published yet, dropping imu message !")
            pass
        elif self.gyro_time > self.accel_current_time:
            interpolated_imu.linear_acceleration = self.accel_current
            interpolated_imu.angular_velocity = self.gyro_data
        elif self.gyro_time > self.accel_prev_time:
            ratio = (self.gyro_time - self.accel_prev_time)/(self.accel_current_time - self.accel_prev_time)
            interpolated_imu.linear_acceleration.x = self.accel_prev.x + ratio*(self.accel_current.x - self.accel_prev.x)
            interpolated_imu.linear_acceleration.y = self.accel_prev.y + ratio*(self.accel_current.y - self.accel_prev.y)
            interpolated_imu.linear_acceleration.z = self.accel_prev.z + ratio*(self.accel_current.z - self.accel_prev.z)
            interpolated_imu.angular_velocity = self.gyro_data
        else:
            interpolated_imu.linear_acceleration = self.accel_prev
            interpolated_imu.angular_velocity = self.gyro_data
        
        interpolated_imu.header = self.gyro_header
        interpolated_imu.header.frame_id = self.camera_name+"_imu_optical_frame"
        self.imu_pub.publish(interpolated_imu)

if __name__ == "__main__":
    try:
        camera_name = "d435i" # change the name according to the rosbag
        imu_fusion_node = IMUFusionNode(camera_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass