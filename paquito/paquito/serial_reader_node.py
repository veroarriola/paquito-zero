import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import serial
from time import sleep

#from collections import deque
#from queue import SimpleQueue



class SerialDataPublisher(Node):
    def __init__(self):
        super().__init__('serial_data_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.ser = serial.Serial ("/dev/ttyS0", 9600)    #Open port with baud rate
        #self.msg_queue = SimpleQueue()
        #self.ring_buffer = deque()
        self.read_bytes = b""

    def timer_callback(self):
        ser = self.ser
        self.read_bytes
        #ring_buffer = self.ring_buffer

        #received_data = ser.read()              #read serial port
        #sleep(0.03)
        data_left = ser.inWaiting()             #check for remaining byte
        received_data = ser.read(data_left)
        self.read_bytes += received_data
        #ring_buffer.append(received_data)

        index = self.read_bytes.find(b"\r\n")
        if index > -1:
            line = self.read_bytes[:index]
            self.read_bytes = self.read_bytes[index + 2:]

            msg = String()
            msg.data = 'Hello World: %s' % line
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    serial_data_publisher = SerialDataPublisher()

    rclpy.spin(serial_data_publisher)

    serial_data_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

