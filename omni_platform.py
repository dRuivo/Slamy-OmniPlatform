import serial
import socketio
from threading import Thread
import time

sio = socketio.Client()

class serialHandler:
    def __init__(self, port, baudrate=9600, timeout=1):
        self.stop = False
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

    def start(self):
        t = Thread(target=self.listen)
        t.start()

    def listen(self):
        print('listening')
        while not self.stop:
            line = self.serial.readline()   # read a '\n' terminated line
            print(line)

    def send(self, s):
        print(s)
        self.serial.write(s)

    def close(self):
        self.stop = True

class omniInterface:
    def __init__(self, s_handler):
        # FL, FR, RL, RR
        self.motor_speeds = [-1, -127, -255, 255]
        
        self.sending_timestamp = time.time()
        self.received_timestamp = time.time()
        self.sending_heart_period = 0.2
        self.received_heart_period = 0.2
        
        self.alive = False
        
        self.stop = False
        self.serial_handler = s_handler

    def run(self):
        t = Thread(target=self.check_heartbeat)
        t.start()
        self.serial_handler.start()

    def check_heartbeat(self):
        while not self.stop:
            if self.alive and (time.time() - self.received_timestamp) >= self.received_heart_period:
                self.motor_speeds = [0, 0, 0, 0]
                line = self.prepSpeedLine(self.motor_speeds)
                self.serial_handler.send(line)
                self.alive = False

    def set_velocity(self, x, y, theta):
        self.received_timestamp = time.time()
        self.alive = True
        self.motor_speeds = self.convert_speeds(x, y, theta)
        line = self.prepSpeedLine(self.motor_speeds)
        self.serial_handler.send(line)

    @staticmethod
    def prepSpeedLine(speeds):
        line = b's'
        for i in speeds:
            line += int(i).to_bytes(2, byteorder='little', signed=True)
        line += b'\r\n'
        return line

    @staticmethod
    def convert_speeds(x, y, theta):
        speeds = [0, 0, 0, 0]
        speeds[0] = x + y + theta # fl
        speeds[1] = x - y + theta # rl
        speeds[2] = x - y - theta # fr
        speeds[3] = x + y - theta # rr
        return speeds


class omniClient(socketio.ClientNamespace):
    def __init__(self, device, namespace=None):
        super(socketio.ClientNamespace, self).__init__(namespace)
        self.device = device


    def on_connect(self):
        print('connection established')

    def on_velocity_cmd(self, data):
        print(data)
        self.device.set_velocity(data['x'], data['y'], data['theta'])
        print(data)

    def on_disconnect(self):
        print('disconnected from server')



try:
    ser = serialHandler('/dev/ttyACM0')
    omni = omniInterface(ser)
    sio.register_namespace(omniClient(omni))
    omni.run()
    sio.connect('http://localhost:5000')
    sio.wait()
    
except KeyboardInterrupt:
    print("disconnecting")
    ser.close()