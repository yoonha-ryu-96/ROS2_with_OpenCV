import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from opencv_msgs.srv import Capture
import cv2
from datetime import datetime
import os

class CameraCapture(Node):
    def __init__(self):
        super().__init__('camera_capture')
        self.bridge = CvBridge()
        self.last_images = {}
        self.base_directory = self.declare_parameter('file_path', '../opencv_oneday/src/opencv_msgs/cap/').get_parameter_value().string_value

        # Predefined topics
        self.topics = [
            "/camera_original",
            "/red_image",
            "/green_image",
            "/blue_image",
            "/camera_canny",
            "/camera_with_canny",
            "/gausian_blur_camera"
        ]

        # Create subscriptions for all predefined topics
        for topic in self.topics:
            self.create_subscription(Image, topic, lambda msg, t=topic: self.image_callback(msg, t), 10)
        
        # Service setup
        self.save_srv = self.create_service(Capture, 'save_image', self.save_image_callback)

    def image_callback(self, msg, topic):
        # Store the latest image received for each topic
        self.last_images[topic] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def save_image_callback(self, request, response):
        topic = request.topic
        if topic in self.last_images and self.last_images[topic] is not None:
            try:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = os.path.join(self.base_directory, f"{topic.replace('/', '_')}_{timestamp}.png")
                if not os.path.exists(self.base_directory):
                    os.makedirs(self.base_directory)
                cv2.imwrite(filename, self.last_images[topic])
                response.success = True
                response.message = "Image successfully saved."
                response.filename = filename
            except Exception as e:
                response.success = False
                response.message = f"Failed to save image: {str(e)}"
        else:
            response.success = False
            response.message = "No image available for the requested topic."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CameraCapture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
