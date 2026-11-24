from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

from src.utils.messages.allMessages import (
    DWM_pos
)

import time
import serial
import threading

PORT ="/dev/ttyACM1"
MIN_QUALITY=20

class threadtag_reader(ThreadWithStop):
    """This thread handles tag_reader.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.port = PORT
        self.min_quality = MIN_QUALITY
        # self._running = False
        self.subscribe()
        super(threadtag_reader, self).__init__()
        self._lock = threading.Lock()
        self._pos = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "quality": 0
        }

        
        self.tagSender = messageHandlerSender(self.queuesList, DWM_pos)



    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass

    def state_change_handler(self):
        pass

    def thread_work(self):      

        ser = None

        try:

            ser = serial.Serial(
                self.port,
                115200,
                timeout=1.0,
                exclusive=True
            )

            print(f"[DWM1001] Connected on {self.port}")

            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # enter shell
            ser.write(b"\r\r")
            time.sleep(1)

            # start location engine
            ser.write(b"lec\r")
            time.sleep(0.5)

            print("[DWM1001] Streaming POS data")
            while True:
            # while self._running:

                raw = ser.read_until(b'\n')

                if not raw:
                    continue

                line = raw.decode(errors="ignore").strip()

                # print("RAW:", line)

                parts = line.split(",")

                if "POS" not in parts:
                    continue

                i = parts.index("POS")

                try:
                    x = float(parts[i + 1])
                    y = float(parts[i + 2])
                    z = float(parts[i + 3])
                    q = int(parts[i + 4])
                except (ValueError, IndexError):
                    continue

                if q < self.min_quality:
                    continue

                # update shared position safely
                with self._lock:
                    self._pos["x"] = x
                    self._pos["y"] = y
                    self._pos["z"] = z
                    self._pos["quality"] = q
                
                self.tagSender.send({
                    "x": x,
                    "y": y,
                    "z": z,
                    "quality": q
                })

                # print(f"[DWM1001] UPDATED → x={x:.3f} y={y:.3f} z={z:.3f} q={q}")

        except Exception as e:

            print(f"[DWM1001] ERROR: {e} — reconnecting...")

        finally:

            if ser:
                try:
                    ser.close()
                except:
                    pass

            time.sleep(5)
                

