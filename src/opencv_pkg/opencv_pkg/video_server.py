import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from opencv_msgs.srv import Record
import cv2
import os
from threading import Thread, Lock
import time

class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        self.bridge = CvBridge()
        self.record_srv = self.create_service(Record, 'record_video', self.record_video_callback)
        self.lock = Lock()
        self.base_directory = self.declare_parameter('file_path', '/not/care/your/path').get_parameter_value().string_value
        self.last_images = {}
        self.video_writers = {}

        # Predefined topics to initialize video writers
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
            self.last_images[topic] = None

    def image_callback(self, msg, topic):
        with self.lock:
            try:
                self.last_images[topic] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.get_logger().error(f'Error converting image for topic {topic}: {str(e)}')

    def record_video_callback(self, request, response):
        if not os.path.exists(self.base_directory):
            os.makedirs(self.base_directory)

        duration = 10  # Duration of recording in seconds
        for topic in request.topics:
            if topic in self.last_images and self.last_images[topic] is not None:
                frame = self.last_images[topic]
                video_path = os.path.join(self.base_directory, f"{topic.replace('/', '_')}.avi")
                if not os.path.exists(self.base_directory):
                    os.makedirs(self.base_directory)
                height, width, _ = frame.shape
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                video_writer = cv2.VideoWriter(video_path, fourcc, 20.0, (width, height))
                self.video_writers[topic] = video_writer
                Thread(target=self.start_recording, args=(topic, duration)).start()
        
        response.success = True
        response.message = "Recording started for the selected topics."
        return response

    def start_recording(self, topic, duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            frame = self.last_images.get(topic)
            if frame is not None:
                self.video_writers[topic].write(frame)
            time.sleep(1/20)  # Assume 20 FPS
        self.video_writers[topic].release()
        del self.video_writers[topic]

def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
