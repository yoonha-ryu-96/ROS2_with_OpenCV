import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class ImgProcessor(Node):
    def __init__(self):
        super().__init__('rgb_sub')
        self.subscription = self.create_subscription(Image,'/camera_original',self.image_callback,10)
        self.subscription  # prevent unused variable warning

        self.red_pub = self.create_publisher(Image, '/red_image', 1)
        self.green_pub = self.create_publisher(Image, '/green_image', 1)
        self.blue_pub = self.create_publisher(Image, '/blue_image', 1)
        self.cv_bridge = CvBridge()

        self.lock = threading.Lock()  # Lock for thread safety

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Create threads for processing each channel
        red_thread = threading.Thread(target=self.process_image, args=(frame, 0.5, 0.5, 1.5, self.red_pub))
        green_thread = threading.Thread(target=self.process_image, args=(frame, 0.5, 1.5, 0.5, self.green_pub))
        blue_thread = threading.Thread(target=self.process_image, args=(frame, 1.5, 0.5, 0.5, self.blue_pub))

        # Start the threads
        red_thread.start()
        green_thread.start()
        blue_thread.start()

        # Wait for all threads to complete
        red_thread.join()
        green_thread.join()
        blue_thread.join()

    def process_image(self, frame, b_factor, g_factor, r_factor, publisher):
        # Split channels
        b, g, r = cv2.split(frame)

        # Apply factors to each channel
        red_img = cv2.merge([np.clip(b * b_factor, 0, 255).astype(np.uint8),
                             np.clip(g * g_factor, 0, 255).astype(np.uint8),
                             np.clip(r * r_factor, 0, 255).astype(np.uint8)])

        # Convert processed image to ROS Image message
        img_msg = self.cv_bridge.cv2_to_imgmsg(red_img, "bgr8")

        # Publish the processed image
        with self.lock:
            publisher.publish(img_msg)

def main():
    rclpy.init()
    node = ImgProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
