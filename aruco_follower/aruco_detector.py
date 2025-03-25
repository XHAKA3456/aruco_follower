import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import json


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_aruco', 10)
        self.image_sub = self.create_subscription(CompressedImage, '/camera/image/compressed', self.image_callback, 10)
        self.marker_pub = self.create_publisher(String, '/aruco/marker_info', 10)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float64)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float64)
        self.marker_length = 0.08
        self.frame_width = 640
        self.frame_center = self.frame_width // 2
        self.center_threshold = 50

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        twist_msg = Twist()
        marker_info = []

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                distance = tvecs[i][0][2]
                marker_info.append({"ID": int(ids[i][0]), "distance": round(distance, 2)})

                x_min = int(corners[i][0][0][0])
                x_max = int(corners[i][0][2][0])
                x_center = (x_min + x_max) // 2

                if abs(x_center - self.frame_center) > self.center_threshold:
                    twist_msg.angular.z = 0.25 if x_center < self.frame_center else -0.25
                    twist_msg.linear.x = 0.0
                else:
                    twist_msg.angular.z = 0.0
                    twist_msg.linear.x = 0.2 if distance > 0.5 else 0.0

                self.vel_pub.publish(twist_msg)

        # ArUco 마커 정보를 JSON 문자열로 변환하여 발행
        marker_msg = String()
        marker_msg.data = json.dumps(marker_info)
        self.marker_pub.publish(marker_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
