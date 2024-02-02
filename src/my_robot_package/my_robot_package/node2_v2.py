# node2_v2.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int16
import cv2
from cv_bridge import CvBridge
import json  # Importing for easy stringification of list or dict

class CameraSubscriberV2(Node):
    """
    Subscribes to video frames, processes them to detect edges, 
    calculates specific error based on position of detected edges,
    and calculates right and left motor speeds based on this error.
    """
    def __init__(self):
        super().__init__('video_subscriber_v2')
        self.camera_subscriber = self.create_subscription(Image, '/rpi_video', self.camera_callback, 10)
        self.bridge = CvBridge()
        
        self.error = 0
        
        # New publisher for motor speeds
        self.speed_publisher = self.create_publisher(String, '/motor_speeds', 10)
        
        # Existing publisher and functionality remains for processed image visualization
        self.image_publisher = self.create_publisher(Image, '/processed_video', 10)
        
        # Additional publisher for the error value
        self.error_publisher = self.create_publisher(Int16, '/line_tracking_error', 10)


    def camera_callback(self, data):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(data, 'mono8')
        # Edge detection
        edged = cv2.Canny(frame, 80, 100)


        white_index = []
        mid_point_line = 0
        # Detect white pixels in the middle row to find edges
        for index, value in enumerate(edged[139]):
            if value == 255:
                white_index.append(index)

        # If two edges are detected, calculate their midpoint
        if len(white_index) == 2:
            cv2.circle(img=edged, center=(white_index[0], 139), radius=2, color=(255, 0, 0), thickness=1)
            cv2.circle(img=edged, center=(white_index[1], 139), radius=2, color=(255, 0, 0), thickness=1)
            mid_point_line = int((white_index[0] + white_index[1]) / 2)
            cv2.circle(img=edged, center=(mid_point_line, 139), radius=3, color=(255, 0, 0), thickness=2)

        # Calculate error based on the robot's midpoint and the line's midpoint
        mid_point_robot = [205, 139]  # Assumed robot's midpoint
        cv2.circle(img=edged, center=(mid_point_robot[0], mid_point_robot[1]), radius=5, color=(255, 0, 0), thickness=2)
        self.error = mid_point_robot[0] - mid_point_line
        
        # Publish the calculated error for monitoring
        error_msg = Int16()
        error_msg.data = self.error
        self.error_publisher.publish(error_msg)
        
        # Convert and publish the processed frame
        processed_frame_msg = self.bridge.cv2_to_imgmsg(edged, "mono8")
        self.image_publisher.publish(processed_frame_msg)

        # Calculate motor speeds based on the error
        left_speed, right_speed = self.calculate_motor_speeds(self.error)
        # Publish motor speeds as a comma-separated string
        speeds_msg = String()
        speeds_msg.data = json.dumps([left_speed, right_speed])  # Using JSON for easy parsing and consistency
        self.speed_publisher.publish(speeds_msg)

    def calculate_motor_speeds(self, error):
        """
        Calculates motor speeds based on error.
        Here, implement your logic to determine left and right motor speeds.
        Example: basic proportional control logic could be applied.
        """
        # When error is exactly 205, set motor speeds to zero
        if error == 205:
            return 0, 0
        
        # Placeholder logic for speed calculation based on error
        # This is where you'd implement the PID control or simple proportional control, etc.
        base_speed = 50  # Base speed for both motors
        error_factor = 2  # How much error affects motor speed
        
        left_speed = max(min(base_speed - (error * error_factor), 100), -100)
        right_speed = max(min(base_speed + (error * error_factor), 100), -100)
        
        return left_speed, right_speed

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber_v2 = CameraSubscriberV2()
    print("Video Subscriber V2 Node started")
    rclpy.spin(camera_subscriber_v2)
    camera_subscriber_v2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()