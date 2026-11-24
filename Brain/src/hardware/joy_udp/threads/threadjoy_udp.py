import time, socket, threading
from struct import unpack

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

from src.utils.messages.allMessages import (
    JoyAxes,
    JoyButton
)

# class JoyAxes(Enum):
#     Queue = "General"
#     Owner = "threadjoy_udp"
#     msgID = 1
#     msgType = "dict"
# class JoyButton(Enum):
#     Queue = "General"
#     Owner = "threadjoy_udp"
#     msgID = 2
#     msgType = "int"

# SERVER CONFIG

UDP_IP = "0.0.0.0"
UDP_PORT = 5017

class threadjoy_udp(ThreadWithStop):
    """This thread handles joy_udp.file:///run/user/1000/gvfs/sftp:host=10.42.0.32,user=pi/home/pi/Documents/Brain/src/hardware/camera/threads/threadCamera.py
    
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        super(threadjoy_udp, self).__init__(pause=0.001)
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        self.joyAxesSender = messageHandlerSender(self.queuesList, JoyAxes)
        self.joyButtonSender = messageHandlerSender(self.queuesList, JoyButton)

        # Setup UDP Server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        self.sock.bind((UDP_IP, UDP_PORT))
        
    def state_change_handler(self):
        pass

    def thread_work(self):
        try:
            data, addr = self.sock.recvfrom(1024) # buffer size is 1024 bytes
            #print("received message: ", data.hex())
            
            data_size = len(data)
            #print(data_size)
            if data_size == 13: # Probably Axes message
                if((data[0] == 170) and (data[1] == 187) and (data[2] == 4)):
                    
                    decoded_axes = unpack('>'+'H'*(len(data[3 :])//2),data[3 :])
                                 
                    steering_v = self.constrain_num(float(decoded_axes[1]), 0.0, float(decoded_axes[0]))
                    throttle_v = self.constrain_num(float(decoded_axes[4]), 0.0, float(decoded_axes[0]))
                    
                    msg_tmp = {
                        "steering": str(int(steering_v)),
                        "throttle": str(int(throttle_v)),
                    }
                    self.joyAxesSender.send(msg_tmp)

            elif data_size == 5: # Probably Button message
                if((data[0] == 170) and (data[1] == 187) and (data[2] == 204)):
                    decoded_button = unpack('>'+'H'*(len(data[3 :])//2),data[3 :])[0]
                    if (decoded_button >= 0):
                        #print("Button:", decoded_button)

                        self.joyButtonSender.send(decoded_button)
                        
            elif data_size == 4: # Probably Hat message
                if((data[0] == 170) and (data[1] == 187) and (data[2] == 221)):
                    decoded_hat = int(data[3])
                    #print(decoded_hat)
        except:
            pass

    def stop(self):
        self.sock.close()
    
    def constrain_num(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))
    
    def map_range(self, x, in_min, in_max, out_min, out_max):
        tmp_x = min(in_max, max(in_min, x))
        return (tmp_x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    
