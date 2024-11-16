# import rclpy
# from rclpy.node import Node
#
# from std_msgs.msg import String
#
# import serial
#
#
# class SerialDataPublisher(Node):
#     def __init__(self):
#         super.__init__('serial_data_publisher')
#         self.publisher_ = self.create_publisher(String, 'topic', 10)
#         timer_period = 0.03  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#
#         ser = serial.Serial ("/dev/ttyS0", 9600)    #Open port with baud rate
#
#     def timer_callback(self):
#         received_data = ser.read()              #read serial port
#         sleep(0.03)
#         data_left = ser.inWaiting()             #check for remaining byte
#         received_data += ser.read(data_left)
#
#         msg = String()
#         msg.data = 'Hello World: %s' % received_data
#         self.publisher_.publish(msg)
#         #self.get_logger().info('Publishing: "%s"' % msg.data)

#https://learn.sparkfun.com/tutorials/graph-sensor-data-with-python-and-matplotlib/update-a-graph-in-real-time
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
#import tmp102
import smbus

# Module variables
i2c_ch = 1
bus = None

# TMP102 address on the I2C bus
# https://learn.sparkfun.com/tutorials/graph-sensor-data-with-python-and-matplotlib/plot-sensor-data
# i2c_address = 0x48
#
# # Register addresses
# reg_temp = 0x00
# reg_config = 0x01
#
# # Calculate the 2's complement of a number
# def twos_comp(val, bits):
#     if (val & (1 << (bits - 1))) != 0:
#         val = val - (1 << bits)
#     return val
#
# # Read temperature registers and calculate Celsius
# def read_temp():
#
#     global bus
#
#     # Read temperature registers
#     val = bus.read_i2c_block_data(i2c_address, reg_temp, 2)
#     temp_c = (val[0] << 4) | (val[1] >> 4)
#
#     # Convert to 2s complement (temperatures can be negative)
#     temp_c = twos_comp(temp_c, 12)
#
#     # Convert registers value to temperature (C)
#     temp_c = temp_c * 0.0625
#
#     return temp_c
#
# # Initialize communications with the TMP102
# def init():
#
#     global bus
#
#     # Initialize I2C (SMBus)
#     bus = smbus.SMBus(i2c_ch)
#
#     # Read the CONFIG register (2 bytes)
#     val = bus.read_i2c_block_data(i2c_address, reg_config, 2)
#
#     # Set to 4 Hz sampling (CR1, CR0 = 0b10)
#     val[1] = val[1] & 0b00111111
#     val[1] = val[1] | (0b10 << 6)
#
#     # Write 4 Hz sampling back to CONFIG
#     bus.write_i2c_block_data(i2c_address, reg_config, val)
#
#     # Read CONFIG to verify that we changed it
#     val = bus.read_i2c_block_data(i2c_address, reg_config, 2)


# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

# Initialize communication with TMP102
tmp102.init()

# This function is called periodically from FuncAnimation
def animate(i, xs, ys):

    # Read temperature (Celsius) from TMP102
    temp_c = round(tmp102.read_temp(), 2)

    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys.append(temp_c)

    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('TMP102 Temperature over Time')
    plt.ylabel('Temperature (deg C)')

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
plt.show()
