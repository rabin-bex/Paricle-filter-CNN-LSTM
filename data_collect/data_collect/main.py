#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped as PWC
from cv_bridge import CvBridge
import csv
import math

def quaternion_to_euler(w,x,y,z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z


def euler_to_quaternion(roll, pitch, yaw):
    """Converts Euler angles (roll, pitch, yaw) to a quaternion.

    Args:
        roll: Rotation about the x-axis (in radians).
        pitch: Rotation about the y-axis (in radians).
        yaw: Rotation about the z-axis (in radians).

    Returns:
        q: Quaternion represented as a 4-element vector (w, x, y, z).
    """
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    q = [w, x, y, z]
    return q


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_collect_node')
        #for simulation
        # self.subscription_image_raw = self.create_subscription(Image,'/camera/image_raw',self.image_callback,10)
        #for real robot
        self.subscription_image_raw = self.create_subscription(Image,'/image_raw',self.image_callback,10)
        self.subscription_amcl_pos = self.create_subscription(PWC,'/amcl_pose',self.amcl_pos_callback,10)
        self.count=0
        self.bridge = CvBridge()
        self.x=0.0
        self.y=0.0
        self.w=0.0
        self.theta=0.0
        self.yaw=0.0
        self.pose_flag=False

    def image_callback(self, msg):
        try:
            # Convert the Image message to an OpenCV image
            # self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # self.cv_image = cv2.resize(self.cv_image, (224, 224))
            

        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return
    def timer_callback(self):
        try:
            png_filename = "/home/rabin/ros2/nav_ws/src/data_collect/images/output"+str(self.count)+".png"
            cv2.imwrite(png_filename, self.cv_image)
            self.get_logger().info(f"converting image: {self.count}")
            self.count+=1
        
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return

    def amcl_pos_callback(self,msg):
        self.pose_flag=True
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        self.theta=msg.pose.pose.orientation.z
        self.w=msg.pose.pose.orientation.w
        roll,pitch,self.yaw=quaternion_to_euler(self.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
        self.get_logger().info(f"amcl position received...{roll},{pitch},{self.yaw}")

def main(args=None):
    rclpy.init(args=args)
    data_logger = DataLogger()
    fields = ['Image','Pos_x', 'Pos_y', 'Yaw']
    csv_filename='/home/rabin/ros2/nav_ws/src/data_collect/images/data.csv'
    count=0
    with open(csv_filename,'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(fields)
        while rclpy.ok():
          rclpy.spin_once(data_logger)
          if data_logger.pose_flag==True:
              data_logger.pose_flag=False
              fields=['output'+str(count)+'.png',data_logger.x,data_logger.y,data_logger.yaw]
              png_filename = "/home/rabin/ros2/nav_ws/src/data_collect/images/output"+str(count)+".png"
              cv2.imwrite(png_filename, data_logger.cv_image)
              csvwriter.writerow(fields)
              count+=1
    data_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    