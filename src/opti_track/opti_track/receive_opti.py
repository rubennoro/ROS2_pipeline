import rclpy
import time
import smbus
import serial
import csv
import os
from rclpy.node import Node
import struct
from geometry_msgs.msg import PoseStamped

class OptitrackSubscriber(Node):
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('optitrack_subscriber')
        
        self.node = rclpy.create_node('optitrack_subscriber')

        # Serial port setup
        self.usb_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.ser = serial.Serial(self.usb_port, self.baud_rate, timeout=1)

        # Global variable to store received PoseStamped data
        self.var = None

        # Create the subscription to PoseStamped
        self.node.create_subscription(PoseStamped, '/vrpn_client_node/opti_test/pose', self.pose_callback, 10)

        # File setup for Data Testing
        #self.file_path = '/home/husky/arduinointerface/plot_optitrack'
        #self.file_name = 'circle240hzwifieth2.csv'
        #self.path = os.path.join(self.file_path, self.file_name)

        # Open CSV file to write data
        #self.test_data = open(self.path, mode='w')
        #self.points_writer = csv.writer(self.test_data)
        #self.points_writer.writerow(['Time', 'Bytes', 'PosX', 'PosY', 'PosZ', 'OriX', 'OriY', 'OriZ', 'OriW'])
	#Part of Data Collection is arr_bytes
        self.arr_bytes = 0

    def pose_callback(self, msg):
        # This function gets called when a new PoseStamped message is received
        self.var = msg

    def write_serial_data(self):
        # Make sure that PoseStamped data is available
        if self.var is None:
            return
        
        if self.arr_bytes > 6720:
            self.arr_bytes = 0
        self.arr_bytes += 28
        #Only for data collection
        #timestamp = time.time()

        # Pack data into bytes
        data_bytes = struct.pack(
            'fffffffffff', 
            self.var.pose.position.x, self.var.pose.position.y, self.var.pose.position.z,
            self.var.pose.orientation.x, self.var.pose.orientation.y, self.var.pose.orientation.z, self.var.pose.orientation.w,
            self.var.pose.position.x, self.var.pose.position.y, self.var.pose.position.z, self.var.pose.orientation.x
        )

        # Write data to serial port
        if self.ser.is_open:
            self.ser.write(data_bytes)
            self.node.get_logger().info("Data sent")
        else:
            self.node.get_logger().error("Serial port not open")

        # Optionally, write to CSV (if needed for logging purposes)
        # self.points_writer.writerow([timestamp, self.arr_bytes, self.var.pose.position.x, self.var.pose.position.y, self.var.pose.position.z, 
        #                              self.var.pose.orientation.x, self.var.pose.orientation.y, self.var.pose.orientation.z, self.var.pose.orientation.w])

    def run(self):
        # Run the subscriber node
        rate = self.node.create_rate(240)

        while rclpy.ok():
            # Handle data writing to serial
            self.write_serial_data()

            # Sleep to maintain the desired rate
            rate.sleep()

        # Close CSV file when done
        self.test_data.close()

        # Shut down the node
        rclpy.shutdown()

def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Create the subscriber class instance
    subscriber = OptitrackSubscriber()

    # Run the subscriber node
    subscriber.run()

if __name__ == '__main__':
    main()

      
