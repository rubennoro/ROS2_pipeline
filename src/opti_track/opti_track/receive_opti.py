import rclpy
import time
from rclpy.node import Node
import struct
import serial
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped

class OptitrackSubscriber(Node):
    def __init__(self):
        super().__init__('optitracksub')
        
        # Serial Configuration
        self.usb_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.serial_timeout = 0.1  # Reduced timeout for responsiveness
        self.min_serial_interval = 0.01  # 100Hz max rate
        self.ser = None
        
        # Serial Initialization
        self.init_serial()
        
        # ROS Subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/opti_test/pose',
            self.callback,
            qos_profile_sensor_data
        )
        
        # Serial Read Timer (10ms interval)
        self.read_timer = self.create_timer(0.01, self.read_serial)
        
        # Statistics
        self.last_serial_time = time.time()
        self.packet_count = 0
        self.error_count = 0

    #Runs fine and makes connection
    def init_serial(self):
        try:
            if self.ser is not None:
                self.ser.close()
            self.ser = serial.Serial(
                port=self.usb_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout,
                write_timeout=0.1  # Explicit write timeout
            )
            #This all runs fine, makes the connection
            self.get_logger().info(f"Serial port {self.usb_port} opened!")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port: {str(e)}")
            self.ser = None

    def callback(self, msg):
        self.write_serial(msg)

    def write_serial(self, data):
        try:
            #self.get_logger().info(f"At write serial...")
            current_time = time.time()
            if current_time - self.last_serial_time < self.min_serial_interval:
                #self.get_logger().info(f"Not enough time passed")
                return

            send_data = struct.pack(
                '11f',  # Explicit 11 floats
                data.pose.position.x, data.pose.position.y, data.pose.position.z,
                data.pose.orientation.x, data.pose.orientation.y,
                data.pose.orientation.z, data.pose.orientation.w,
                0.0, 0.0, 0.0, 0.0  # Padding
            )

            if self.ser and self.ser.is_open:
                self.ser.write(send_data)
                self.ser.flush()
                self.last_serial_time = current_time
                self.packet_count += 1
            else:
                raise serial.SerialException("Port not open")
                
        except Exception as e:
            self.get_logger().error(f"Write error: {str(e)}", throttle_duration_sec=1.0)
            self.error_count += 1
            self.init_serial()  # Reconnect

    def read_serial(self):
        if not self.ser or not self.ser.is_open:
            return
            
        try:
            # Read all available lines
            while self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line:  # Only process non-empty lines
                    self.get_logger().info(f"Arduino: {line}", throttle_duration_sec=0.1)
                    
        except Exception as e:
            self.get_logger().error(f"Read error: {str(e)}", throttle_duration_sec=1.0)
            self.error_count += 1

    def close_serial(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info("Serial port closed")
            except Exception as e:
                self.get_logger().error(f"Close error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = OptitrackSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        #print(node.error_count)
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

      
