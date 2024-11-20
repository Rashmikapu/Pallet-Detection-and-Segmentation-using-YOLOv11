import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

def read_ros_bag(bag_file):
    # Create a SequentialReader to read from the ROS bag
    reader = rosbag2_py.SequentialReader()
    package_share_directory = get_package_share_directory('pallet_detection')

    seg_model = YOLO(package_share_directory + '/seg_2class.pt')
    count = 0

    # Set storage options (URI points to the bag file, default storage is SQLite3)
    storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id="sqlite3")

    # Set converter options (uses the ROS 2 serialization format)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    # Open the ROS bag for reading
    reader.open(storage_options, converter_options)

    # Get metadata about topics in the bag file
    topics_info = reader.get_all_topics_and_types()
    print("Topics in the bag file:")
    for topic in topics_info:
        print(f"- {topic.name} ({topic.type})")

    print("\nReading messages from the bag file...\n")
    bridge = CvBridge()
    # Read messages from the bag
    while reader.has_next():
        topic, serialized_msg, timestamp = reader.read_next()

        # Get the message type dynamically based on the topic
        msg_type = get_message(topics_info[0].type)  # Assumes a single topic type for simplicity
        msg = deserialize_message(serialized_msg, msg_type)

        if topic == "/robot1/zed2i/left/image_rect_color":  # Adjust topic as needed
            try:
                # Convert ROS 2 Image to OpenCV Image
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                res = seg_model(cv_image)
                count += 1
                # Get the annotated frame and imshow
                seg_ann_frame = res[0].plot(boxes=False)
                det_ann_frame = res[0].plot(masks=False)
                cv2.imshow("Detection_Frame", det_ann_frame)
                cv2.imshow("Segmentation_Frame", seg_ann_frame)
                cv2.waitKey(1) # waits for 1ms before closing the window
                print("Read frame: " + str(count))

                # Display the image
                # cv2.imwrite(f"output_image_{timestamp}.jpg", cv_image)
                # print(f"Saved image at timestamp {timestamp}")
            except Exception as e:
                print(f"Failed to convert and display image {count}: {e}")
        else:
            print(f"Skipped message on topic {topic}")

def main():

    bag_file = './src/pallet_detection/internship_assignment_sample_bag/internship_assignment_sample_bag_0.db3'
    read_ros_bag(bag_file)
