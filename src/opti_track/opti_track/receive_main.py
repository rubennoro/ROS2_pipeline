import rclpy
import time
from rclpy.node import Node
import struct
import serial
# For BEST_EFFORT QOS
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped

class OptitrackSubscriber(Node):

    def __init__(self):
        super().__init__('optitracksub')

        # Initialize USB connection with Arduino Giga 
        self.usb_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.ser = serial.Serial(self.usb_port, self.baud_rate, timeout=1)

        # Create subscriber with QOS best effort
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/opti_test/pose',
            self.callback,
            qos_profile_sensor_data
        )

        # Add for error bypassing 
        self.subscription

        self.last_time = self.get_clock().now()

    def callback(self, msg):
        pose = msg
        #print("Raw message received:", msg)

        # Write to the serial port for arduino
        self.write_serial(pose)

    # Arduino communication
    def write_serial(self, data):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Time difference in seconds
        self.last_time = current_time

        self.get_logger().info(
            f'I heard: Position ({data.pose.position.x}, {data.pose.position.y}, {data.pose.position.z}), Rate: {1/dt:.2f} Hz'
        )

        
        # Pack 11 floats of 44 bytes into a struct
        send_data = struct.pack(
            'fffffffffff', 
            data.pose.position.x, data.pose.position.y, data.pose.position.z,
            data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w,
            data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.orientation.x
        )

        # Write data to serial port
        if self.ser.is_open:
            self.ser.write(send_data)
            self.node.get_logger().info("Data sent")
        else:
            self.node.get_logger().error("Serial port not open")
        

    # Closing connection after completion
    def close_serial(self):
        if self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")


def main(args=None):
    rclpy.init(args=args)

    opti_sub = OptitrackSubscriber()

    # Run for 10 seconds
    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < 10:
        rclpy.spin_once(opti_sub)
    
    opti_sub.close_serial()
    rclpy.shutdown()

if __name__=='__main__':
    main()