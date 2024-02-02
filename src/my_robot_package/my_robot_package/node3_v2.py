# node3_v2.py
# import serial  # Uncomment for actual serial communication
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Changed to match the new message type
import json  # Importing for parsing stringified list or dict

class DifferentialDriveControl(Node):
    """
    Subscribes to motor speeds published by another node and handles
    sending these values to a differential drive controller over a serial connection.
    """
    def __init__(self):
        super().__init__('differential_drive_control')
        self.subscriber = self.create_subscription(String, '/motor_speeds', self.speeds_callback, 10)

        # self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=2)  # Uncomment for actual serial communication

    def speeds_callback(self, msg):
        # Parse the speeds from the incoming message
        speeds = json.loads(msg.data)
        left_speed, right_speed = speeds
        
        # Prepare the message string to send over serial
        send_string = f"{left_speed},{right_speed}\n"

        # For now, print the would-be sent string to the terminal
        print(f"Sending to differential drive: {send_string}")
        # Uncomment below for actual serial communication
        # self.ser.write(send_string.encode('utf-8'))
        # receive_string = self.ser.readline().decode('utf-8').rstrip()
        # print(f"Received from serial: {receive_string}")

def main(args=None):
    rclpy.init(args=args)
    differential_drive_control = DifferentialDriveControl()
    rclpy.spin(differential_drive_control)
    differential_drive_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()