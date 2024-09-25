#! /usr/bin/env python

# Simple string program. Writes and updates strings.
# Demo program for the I2C 16x2 Display from Ryanteck.uk
# Created by Matthew Timmons-Brown for The Raspberry Pi Guy YouTube channel

# Import necessary libraries for communication and display use
import drivers
from time import sleep

from subprocess import check_output
ip = check_output(["hostname", "-I"], encoding="utf8").split()[0]
str_ip = str(ip)

# Load the driver and set it to "display"
# If you use something from the driver library use the "display." prefix first
display = None

def wait_for_display():
    global display
    while display is None:
        try:
            display = drivers.Lcd(0x27)
            ip = check_output(["hostname", "-I"], encoding="utf8").split()[0]
            str_ip = str(ip)
        except OSError:
            pass

def display_host():
# Main body of code
    display.lcd_backlight(1)
    display.lcd_clear()
    while True:
        # Remember that your sentences can only be 16 characters long!
        display.lcd_display_string("Host:", 1)             # Write line of text to first line of display
        display.lcd_display_string(str_ip, 2)  # Write line of text to second line of display
        sleep(9)                                           # Give time for the message to be read

try:
    while True:
        try:
            wait_for_display()
            display_host()
        except OSError:
            continue
except KeyboardInterrupt:
    # If there is a KeyboardInterrupt (when you press ctrl+c), exit the program and cleanup
    print("Cleaning up!")
    display.lcd_clear()
    display.lcd_backlight(0)

