import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import Twist

class ArucoFollower(Node):
    def __init__(self):
        super().__init__('aruco_follower')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_aruco', 10)
        self.cap = cv2.VideoCapture(2)

        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open video device")
            exit()

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()
        self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float64)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float64)
        self.marker_length = 0.08
        self.frame_width = 640
        self.frame_center = self.frame_width // 2
        self.center_threshold = 50

        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Error: Couldn't read frame")
            return

        if self.frame_width == 640 and frame.shape[1] != 640:
            self.frame_width = frame.shape[1]
            self.frame_center = self.frame_width // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        twist_msg = Twist()

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                distance = tvecs[i][0][2]
                x_min = int(corners[i][0][0][0])
                x_max = int(corners[i][0][2][0])
                x_center = (x_min + x_max) // 2

                text = f"ID {ids[i][0]}: {distance:.2f}m"
                cv2.putText(frame, text, (x_min, int(corners[i][0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.03)

                if abs(x_center - self.frame_center) > self.center_threshold:
                    twist_msg.angular.z = 0.25 if x_center < self.frame_center else -0.25
                    twist_msg.linear.x = 0.0
                else:
                    twist_msg.angular.z = 0.0
                    twist_msg.linear.x = 0.2 if distance > 0.5 else 0.0

                self.vel_pub.publish(twist_msg)

        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()