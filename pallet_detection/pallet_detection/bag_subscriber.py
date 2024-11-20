import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class ImageSubscriber(Node):
    read_image = True
    def __init__(self):
        # Initialize the node name
        super().__init__('bag_subscriber')
        
        # Create a ros2 opencv bridge.
        self.bridge = CvBridge()
        
        # Create a subscriber for the topic '/source_image' with QoS profile
        qos_profile = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT, depth = 1)
        self.subscriber = self.create_subscription(Image, '/robot1/zed2i/left/image_rect_color', self.img_callback, qos_profile)

        # Load the yolo model for inferencing from the package shared directory
        package_share_directory = get_package_share_directory('pallet_detection')
        self.seg_model = YOLO(package_share_directory + '/seg_2class.pt')
        self.count = 0 # keep frames count
        
    def img_callback(self, msg):
        try:
            self.count = self.count + 1
            # Convert the Image to opencv readable image using bridge.
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Inference the image with confidence score
            res = self.seg_model(img)
            # Get the annotated frame and imshow
            seg_ann_frame = res[0].plot(boxes=False)
            det_ann_frame = res[0].plot(masks=False)
            cv2.imshow("Detection_Frame", det_ann_frame)
            cv2.imshow("Segmentation_Frame", seg_ann_frame)
            cv2.waitKey(1) # waits for 1ms before closing the window
            self.get_logger().info("Read frame: " + str(self.count))
        except Exception as e:
            self.get_logger().error(e.with_traceback)
    
def main():
    rclpy.init()
    sub_node = ImageSubscriber()
    rclpy.spin(sub_node)
    sub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
