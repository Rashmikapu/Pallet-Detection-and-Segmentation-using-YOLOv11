import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile

class ImagePublisher(Node):
    def __init__(self):
        # Initialize the node name
        super().__init__('camera_publisher')
        # Create a ros2 opencv bridge.
        self.bridge = CvBridge()
        # Create a qos profile
        qos_profile = QoSProfile(depth=10)
        # Uncomment the below line to read frames from default camera device.
        self.cap = cv2.VideoCapture('/dev/video0')
        
        # test image - Comment out
        # self.test_img = cv2.imread('/ros_ws/src/pallet_detection/val_images/img1.jpeg')
        
        # Create a publisher with topic name '/source_image'
        self.publisher = self.create_publisher(Image, 'source_image', qos_profile)
        
        # Create timer to publish each frame
        self.timer = self.create_timer(0.01, self.img_publisher) # call every 10ms to publish each frame
        self.count = 0 # keep frames count
        
    def img_publisher(self):          
        ret, frame = self.cap.read()
        if ret:
            self.count = self.count + 1
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        
            # Publishing the test image continuously - Comment out
            # msg = self.bridge.cv2_to_imgmsg(self.test_img, "bgr8")
            
            # Publish the image
            self.publisher.publish(msg)
            self.get_logger().info("Image frame published: " + str(self.count))
            return
        else:
            self.cap.release()
            self.timer.cancel()
    
def main():
    rclpy.init()
    pub_node = ImagePublisher()
    rclpy.spin(pub_node)
    pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
