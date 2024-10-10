import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from timeit import default_timer as timer
#from threading import Locki

class StringSubscriber(Node):

    def __init__(self):
        super().__init__('string_subscriber')
        self.subscription = self.create_subscription(
                String,
                'sensor_readings',
                self.listener_callback,
                10)
        self.subscription  # prevent unused variable warning

        timer_period = 1.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.num_messages = 0
        self.enc_messages = 0
        self.mic_messages = 0
        self.start_time = timer()

    def listener_callback(self, msg):
        #self.get_logger().info(f"I heard: {msg.data}")
        mess = msg.data#.decode("utf-8")
        print(mess)
        if (mess.endswith('[/ENC]')):
            self.enc_messages += 1
        elif (mess.endswith('[/MIC]')):
            self.mic_messages += 1
        self.num_messages += 1

    def timer_callback(self):
        #with lock:
        i = self.num_messages
        i_enc = self.enc_messages
        i_mic = self.mic_messages
        self.num_messages = self.enc_messages = self.mic_messages = 0
        new_start = timer()
        elapsed_time = new_start - self.start_time
        self.start_time = new_start

        self.get_logger().info(f"I head {i} messages: {i_enc} [ENC/] and {i_mic} [MIC/] in {elapsed_time} seconds.")
        

def main(args = None):
    print('Hi from paco_monitor.')

    rclpy.init(args = args)

    string_subscriber = StringSubscriber()

    rclpy.spin(string_subscriber)

    string_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
