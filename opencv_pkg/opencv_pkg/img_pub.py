import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class ImgPublisher(Node):
    def __init__(self):
        super().__init__('img_pub')
        self.original_pub = self.create_publisher(Image, '/camera_original', 1)
        self.canny_pub = self.create_publisher(Image, '/camera_canny', 1)
        self.overlay_pub = self.create_publisher(Image, '/camera_with_canny', 1)
        self.Gausian_blur_pub = self.create_publisher(Image, '/gausian_blur_camera', 1)

        self.timer = self.create_timer(0.01, self.time_callback)
        self.cap = cv2.VideoCapture(0)
        self.cv_bridge = CvBridge()

    def time_callback(self):
        ret, frame = self.cap.read()

        # Canny
        edges = cv2.Canny(frame, 100, 200)

        # Edge with cam
        color_edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        overlay_image = cv2.addWeighted(frame, 0.4, color_edges, 0.4, 0.2)

        # Grayscale and Gaussian Blur
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

        original_msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
        canny_msg = self.cv_bridge.cv2_to_imgmsg(edges, "mono8")
        overlay_msg = self.cv_bridge.cv2_to_imgmsg(overlay_image, "bgr8")
        blurred_msg = self.cv_bridge.cv2_to_imgmsg(blurred_image, "mono8")

        # Publishing
        self.original_pub.publish(original_msg)
        self.canny_pub.publish(canny_msg)
        self.overlay_pub.publish(overlay_msg)
        self.Gausian_blur_pub.publish(blurred_msg)


def main():
    rclpy.init()
    node = ImgPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()