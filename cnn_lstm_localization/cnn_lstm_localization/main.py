#this program is for real robot
import rclpy
from rclpy.node import Node
from keras.models import load_model, Model
from keras.preprocessing.image import img_to_array
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped as PWC, PoseStamped as PS, Twist 
from std_msgs.msg import Header
from rclpy.time import Time
import numpy as np
from cv_bridge import CvBridge
import queue
from collections import deque
from keras.applications.vgg16 import VGG16, preprocess_input
import math

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

def publish_rotation_command(node):
    twist=Twist()
    twist.linear.x=0.0
    twist.linear.y=0.0
    twist.linear.z=0.0
    twist.angular.x=0.0
    twist.angular.y=0.0
    twist.angular.z=0.2
    node.publishers_cmdvel.publish(twist)

def publish_initial_pose_for_amcl(node):
    initialpose=PWC()
    initialpose.header.stamp=node.get_clock().now().to_msg()
    initialpose.header.frame_id="map"
    initialpose.pose.pose.position.x=node.x
    initialpose.pose.pose.position.y=node.y
    initialpose.pose.pose.position.z=0.0
    initialpose.pose.pose.orientation.w=node.w
    initialpose.pose.pose.orientation.x=0.0
    initialpose.pose.pose.orientation.y=0.0
    initialpose.pose.pose.orientation.z=node.orientation_z
    initialpose.pose.covariance[0]=0.1 #0.25
    initialpose.pose.covariance[7]=0.1 #0.25
    initialpose.pose.covariance[35]=0.06853891909122467
    node.publishers_initpos.publish(initialpose)



class ImageQueue:
    def __init__(self, size):
        self.size = size
        self.queue = deque(maxlen=size)

    def add(self, value):
        self.queue.append(value)
    
    def get_running_Image(self):
        if len(self.queue) < self.size:
            return None
        return self.queue

    

class CNN_LSTM_Pose_Publisher(Node):
    def __init__(self):
        super().__init__('cnn_lstm_node')
        self.subscription_image_raw = self.create_subscription(Image,'/image_raw',self.image_callback,10)
        self.publisher_cnn_lstm_pos = self.create_publisher(PS, '/cnn_lstm_pose', 10)
        self.publishers_initpos=self.create_publisher(PWC,"/initialpose",10)
        self.publishers_cmdvel=self.create_publisher(Twist,"/cmd_vel",10)
        self.pose_flag=False
        self.img_flag=False
        self.x=0.0
        self.y=0.0
        self.w=0.0
        self.orientation_z=0.0
        self.bridge = CvBridge()

    def amcl_pos_callback(self,msg):
        self.pose_flag=True

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.cv_image = cv2.resize(self.cv_image, (224, 224))
            self.img_flag=True

        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return
   


def main(args=None):
    image_queue=ImageQueue(15)
    rclpy.init(args=args)
    pose_publisher = CNN_LSTM_Pose_Publisher()
    model=load_model('/home/rabin/ros2/nav_ws/src/cnn_lstm_localization/cnn_lstm_localization/update_lstm_model.h5')
    model_vgg=VGG16(weights='imagenet')
    fc6_layer=model_vgg.get_layer('fc1')
    encoder_model=Model(inputs=model_vgg.input, outputs=fc6_layer.output)
    count=0
    time=Time()
    init_flag=True
    ref=pose_publisher.get_clock().now().to_msg().sec
    while rclpy.ok():
        rclpy.spin_once(pose_publisher)
        if pose_publisher.img_flag==True:
            pose_publisher.img_flag=False
            count+=1
            np_image = np.array(pose_publisher.cv_image)
            image=np.expand_dims(np_image,axis=0)
            image=preprocess_input(image)
            encoder_image=encoder_model.predict(image).reshape(4096)
            image_queue.add(encoder_image)
            images=image_queue.get_running_Image()
            print(len(image_queue.queue))
            if images is not None:
                images=np.array([images])
                output=model.predict(images)
                msg=PS()
                msg.header.stamp=pose_publisher.get_clock().now().to_msg()
                msg.header.frame_id="map"
                msg.pose.position.x=float(output[0][0])
                msg.pose.position.y=float(output[0][1])
                w,x,y,z=euler_to_quaternion(0.0,0.0,float(output[0][2]))
                msg.pose.orientation.w=w
                msg.pose.orientation.x=x
                msg.pose.orientation.y=y
                msg.pose.orientation.z=z
                pose_publisher.x=float(output[0][0])
                pose_publisher.y=float(output[0][1])
                pose_publisher.w=w
                pose_publisher.orientation_z=z
                pose_publisher.publisher_cnn_lstm_pos.publish(msg)
        if init_flag==True:
            if (pose_publisher.get_clock().now().to_msg().sec-ref)<16:
                publish_rotation_command(pose_publisher)
            elif (pose_publisher.get_clock().now().to_msg().sec-ref)>18:
                publish_initial_pose_for_amcl(pose_publisher)
                init_flag=False

        
             

    pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()