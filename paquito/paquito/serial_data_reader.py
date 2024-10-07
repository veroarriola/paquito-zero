import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import serial


class SerialDataPublisher(Node):
    def __init__(self):
        super.__init__('serial_data_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.03  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        ser = serial.Serial ("/dev/ttyS0", 9600)    #Open port with baud rate

    def timer_callback(self):
        received_data = ser.read()              #read serial port
        sleep(0.03)
        data_left = ser.inWaiting()             #check for remaining byte
        received_data += ser.read(data_left)

        msg = String()
        msg.data = 'Hello World: %s' % received_data
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
