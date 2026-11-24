from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.control.controller.threads import threadcontroller



from src.utils.messages.allMessages import (
    TOFDistance
)

import RPi.GPIO as GPIO
import serial 
import time
import chardet
import sys

TOF_length = 16
TOF_header=(87,0,255)
TOF_system_time = 0
TOF_distance = 0
TOF_status = 0
TOF_signal = 0
TOF_check = 0

ser = serial.Serial('/dev/ttyAMA0',115200)
ser.flushInput()


  

class threadtof_sensor(ThreadWithStop):
    """This thread handles tof_sensor.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        super(threadtof_sensor, self).__init__(pause=0.001)

        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.subscribe()
        self.tof_sensor_sender = messageHandlerSender(self.queuesList, TOFDistance)
        self.tof_readings = {}  # {sensor_id: distance_mm} για όλους τους sensors
        print(f"\033[1;97m[ TOF sensor Thread ] :\033[0m \033[1;92mINFO\033[0m - TOF sensor initialized successfully (nai sigoura)")
        
    def verifyCheckSum(self,data, len):
        #print(data)
        TOF_check = 0
        for  k in range(0,len-1):
            TOF_check += data[k]
        TOF_check=TOF_check%256
        
        if(TOF_check == data[len-1]):
            #print("TOF data is ok!")
            return 1    
        else:
            print("TOF data is error!")
            return 0  

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass

    def state_change_handler(self):
        pass

    def thread_work(self):

        time.sleep(0.05)

        # Read enough bytes to cover packets from all 4 sensors (4 * 16 = 64 bytes minimum)
        if ser.inWaiting() >= 64:
            TOF_data = ()
            for i in range(0, 32):
                TOF_data = TOF_data + (ord(ser.read(1)), ord(ser.read(1)))

            # Scan the buffer looking for valid packet headers
            for j in range(0, 48):  # 64 bytes - 16 byte packet = 48 max start positions
                if (TOF_data[j] == TOF_header[0] and
                        TOF_data[j+1] == TOF_header[1] and
                        TOF_data[j+2] == TOF_header[2] and
                        self.verifyCheckSum(TOF_data[j:j+TOF_length], TOF_length)):

                    if ((TOF_data[j+12]) | (TOF_data[j+13] << 8)) == 0:
                        continue  # signal = 0, invalid reading

                    sensor_id = TOF_data[j+3]
                    TOF_distance = (TOF_data[j+8]) | (TOF_data[j+9] << 8) | (TOF_data[j+10] << 16)
                    distance = int(TOF_distance)

                    self.tof_readings[sensor_id] = distance

                    # Send the full dict with latest readings from all sensors seen so far
                    self.tof_sensor_sender.send(dict(self.tof_readings))

