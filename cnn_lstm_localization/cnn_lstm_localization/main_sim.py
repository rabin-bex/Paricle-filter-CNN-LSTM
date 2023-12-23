import rclpy
from rclpy.node import Node
from keras.models import load_model, Model
from keras.preprocessing.image import img_to_array
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped as PWC, PoseStamped as PS
from std_msgs.msg import Header
from rclpy.time import Time
import numpy as np
from cv_bridge import CvBridge
import queue
from collections import deque
from keras.applications.vgg16 import VGG16, preprocess_input

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

    

class CNN_LSTMPublisher(Node):
    def __init__(self):
        super().__init__('cnn_lstm_node')
        self.subscription_image_raw = self.create_subscription(Image,'/camera/image_raw',self.image_callback,10)
        self.subscription_amcl_pos = self.create_subscription(PWC,'/amcl_pose',self.amcl_pos_callback,10)
        self.publisher_cnn_lstm_pos = self.create_publisher(PS, 'cnn_lstm_pose', 10)
        self.pose_flag=False
        self.img_flag=False
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
    image_queue=ImageQueue(20)
    rclpy.init(args=args)
    pose_publisher = CNN_LSTMPublisher()
    model=load_model('cnn_lstm_localization/cnn_lstm_localization/update_lstm_model_sim.h5')
    model_vgg=VGG16(weights='imagenet')
    fc6_layer=model_vgg.get_layer('fc1')
    encoder_model=Model(inputs=model_vgg.input, outputs=fc6_layer.output)
    count=0
    time=Time()
    while rclpy.ok():
        rclpy.spin_once(pose_publisher)
        # if pose_publisher.pose_flag==True:
        if True:
            pose_publisher.pose_flag=False
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
                    print(str(float(output[0][0]))+","+str(float(output[0][1])))
                    # msg.pose.orientation.z=float(output[0][2])
                    # msg.pose.orientation.w=float(output[0][3])
                    pose_publisher.publisher_cnn_lstm_pos.publish(msg)
             

    pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
