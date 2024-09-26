#!/usr/bin/python3

IN_RASPBERRY = True

import io
import logging
import socketserver
from http import server
from threading import Condition

if IN_RASPBERRY:
    from picamera2 import Picamera2
    from libcamera import controls
    from picamera2.encoders import JpegEncoder
    from picamera2.outputs import FileOutput
else:
    import cv2
    from imutils.video import VideoStream
    import threading

import time
from smbus2 import SMBus

PAGE = """\
<!DOCTYPE html>
<html>
<head>
 <title>Cámara</title>
</head>
<body>
<h1>Webcam Streaming Demo</h1>
<img src="stream.mjpg" />
</body>
</html>
"""

if IN_RASPBERRY:
    sm_slave_addr = 0x8    # dirección del bus en hexadecimal
    sleep_interval = 0.1

    # FL FR
    # RL RR
    # (Backward Forward)
    commands = {
        'stop':            0b00000000,
        'brake':           0b11001100,
        'accelerate':      0b00110011,
        'forward':         0b00001111,
        'NE':              0b00001010, #right turn
        'right':           0b01101001,
        'SE':              0b10100000, #right back
        'backward':        0b11110000,
        'SW':              0b01000001, #left back
        'left':            0b10010110,
        'NW':              0b00000101, #left turn
        'clockwise':       0b01011010, #clockwise
        'countclockwise':  0b10100101, #countclockwise
        'speak':           0b00010001,
    }


def send_command(command, args = []):
    """
    Envía el byte representando el comando indicado al
    arduino.
    command: cadena de texto para identificar el commando.
    args: lista con los argumentos opcionales, según cada comando.
    """
    with SMBus(1) as bus:    # indica /dev/ic2-1
        # El límite de SMBus son 32 bytes
        data = [commands[command]] + args
        bus.write_i2c_block_data(sm_slave_addr, 0, data)
        time.sleep(sleep_interval)


class StreamingOutput(io.BufferedIOBase):
    """
    Clase utilizada con picamera2
    """
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class StreamingHandler(server.BaseHTTPRequestHandler):

    def do_POST(self):
        print("Atiende post")
        if self.path == '/command/':
            content_length = int(self.headers['Content-Length'])
            body = self.rfile.read(content_length)
            command = body.decode("utf-8")
            print("POST body:", command)
            if IN_RASPBERRY:
                if command in commands:
                    send_command(command)
            content = ('{"command": "' + str(command) + '", "status": "ok"}').encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        else:
            print("Intento de contactar ", self.path)
            self.send_error(404)
            self.end_headers()

    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/command/':
            print("GET command")
            self.send_response(200)
            self.send_header('Location', '/command/')
            self.end_headers()
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                global vs
                global jpeg_quality
                while True:
                    if IN_RASPBERRY:
                        with output.condition:
                            output.condition.wait()
                            jpg_buffer = output.frame
                    else:
                        frame = vs.read()
                        #(h, w) = frame.shape[:2]
                        #frame = cv2.resize(frame, (w//2, h//2))
                        ret_code, jpg_buffer = cv2.imencode(
                            ".jpg",
                            frame,
                            [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
                        )
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(jpg_buffer))
                    self.end_headers()
                    self.wfile.write(jpg_buffer)
                    self.wfile.write(b'\r\n')
                    #cv2.imshow('frame', frame)
            except BrokenPipeError as e:
                logging.info('Broken pipe client $s: %s closed conection.',
                             self.client_address, str(e))
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
                raise e
        else:
            self.send_error(404)
            self.end_headers()


#class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
class StreamingServer(server.ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address, bind_and_activate):
        super().__init__(server_address, bind_and_activate)
        print("Dirección: ", server_address)


if __name__ == '__main__':
    #vs = VideoStream(usePiCamera=True).start()
    if IN_RASPBERRY:
        picam2 = Picamera2()
        conf = picam2.create_video_configuration(main={"size": (1280, 720)})
        #conf = picam2.create_video_configuration(main={"size": (1296//2, 972//2)})

        picam2.configure(conf)
        output = StreamingOutput()
        picam2.start_recording(JpegEncoder(), FileOutput(output))
        picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous, "AfSpeed": controls.AfSpeedEnum.Fast})
    else:
        vs = VideoStream(src=0).start()
        time.sleep(2.0)
        jpeg_quality = 95  # 0 to 100, higher is better quality, 95 is cv2 default

    try:
        address = ('', 8000)
        server = StreamingServer(address, StreamingHandler)
        server.serve_forever()
    finally:
        if IN_RASPBERRY:
            picam2.stop_recording()
        else:
            vs.stop()
            #cv2.destroyAllWindows()

