import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open video device")
            exit()

        self.timer = self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Error: Couldn't read frame")
            return
        
        _, buffer = cv2.imencode('.jpg', frame)
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = np.array(buffer).tobytes()

        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
