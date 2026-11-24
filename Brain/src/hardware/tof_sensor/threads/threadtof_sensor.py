import sys
import time
import serial
import RPi.GPIO as GPIO
import chardet

from src.templates.threadwithstop import ThreadWithStop
from src.control.controller.threads import threadcontroller
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import (
    mainCamera,
    TOFForward,
    TOFLeft,
    TOFRear,
    TOFRight,
)

# ═════════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═════════════════════════════════════════════════════════════════════════════
TOF_length = 16
TOF_header = (87, 0, 255)


# ═════════════════════════════════════════════════════════════════════════════
class threadtof_sensor(ThreadWithStop):
    """This thread reads distance data from 4 ToF sensors over serial.
    Packets arrive as a raw byte stream (16 bytes each, header [87, 0, 255]).
    Sensor IDs: 0 = rear, 3 = left, 1 = right, 2 = forward.

    Args:
        queueList (dict): Dictionary of multiprocessing queues keyed by message type.
        logging: Logging object for debugging output.
        debugging (bool): Enable extra debug prints. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        super(threadtof_sensor, self).__init__(pause=0.001)

        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.subscribe()

        # ── SERIAL PORT INITIALIZATION (LOCAL SCOPE) ─────────────────────────
        try:
            self.ser = serial.Serial("/dev/ttyAMA0", 115200)
            self.ser.flushInput()
        except serial.SerialException as e:
            print(
                f"\033[1;91m[ TOF sensor Thread ] ERROR -\033[0m "
                f"Failed to open serial port /dev/ttyAMA0: {e}"
            )
            sys.exit(1)

        # ── SENDERS ──────────────────────────────────────────────────────────
        self.tof_rear_sender = messageHandlerSender(self.queuesList, TOFRear)
        self.tof_left_sender = messageHandlerSender(self.queuesList, TOFLeft)
        self.tof_right_sender = messageHandlerSender(self.queuesList, TOFRight)
        self.tof_forward_sender = messageHandlerSender(self.queuesList, TOFForward)

        # ── BUFFERS & ACCUMULATORS ───────────────────────────────────────────
        self._serial_buf = bytearray()

        # Frozen-reading detection logic
        self._last_dist = {}  # sensor_id → last distance sent
        self._frozen_cnt = {}  # sensor_id → consecutive identical count
        self._FROZEN_LIMIT = 100

        print(
            f"\033[1;97m[ TOF sensor Thread ] :\033[0m \033[1;92mINFO\033[0m "
            f"- TOF sensor initialized successfully"
        )

    def verifyCheckSum(self, data, length):
        """Returns True if the packet checksum is valid."""
        total = 0
        for k in range(length - 1):
            total += data[k]
        total = total % 256

        if total == data[length - 1]:
            return True
        return False

    def subscribe(self):
        """Subscribes to the messages you are interested in."""
        pass

    def state_change_handler(self):
        pass

    def thread_work(self):
        try:
            # --- Step 1: Read all available bytes into the accumulator ---
            waiting = self.ser.inWaiting()
            if waiting > 0:
                self._serial_buf.extend(self.ser.read(waiting))

            # --- Step 2: Parse complete packets from the accumulator ---
            while len(self._serial_buf) >= TOF_length:

                # Search for the packet header [87, 0, 255] anywhere in the buffer.
                header_pos = -1
                for i in range(len(self._serial_buf) - 2):
                    if (
                        self._serial_buf[i] == TOF_header[0]
                        and self._serial_buf[i + 1] == TOF_header[1]
                        and self._serial_buf[i + 2] == TOF_header[2]
                    ):
                        header_pos = i
                        break

                if header_pos == -1:
                    # No header found anywhere — discard the entire garbage buffer.
                    self._serial_buf.clear()
                    break

                if header_pos > 0:
                    # Drop stale bytes accumulated before the valid header.
                    del self._serial_buf[:header_pos]

                # Re-verify length after slicing out leading garbage bytes
                if len(self._serial_buf) < TOF_length:
                    break

                # Extract the potential 16-byte packet
                packet = self._serial_buf[:TOF_length]

                # Validate checksum
                if not self.verifyCheckSum(packet, TOF_length):
                    # Bad packet — drop the first byte and look for the next header
                    del self._serial_buf[0]
                    continue

                # Valid packet confirmed — extract sensor telemetry data
                signal = packet[12] | (packet[13] << 8)
                distance = packet[8] | (packet[9] << 8) | (packet[10] << 16)
                sensor_id = packet[3]

                # Out-of-range flag evaluation
                if signal == 0:
                    distance = 9999

                # Anti-freeze hardware logic
                if self._last_dist.get(sensor_id) == distance:
                    self._frozen_cnt[sensor_id] = (
                        self._frozen_cnt.get(sensor_id, 0) + 1
                    )
                else:
                    self._frozen_cnt[sensor_id] = 0
                    self._last_dist[sensor_id] = distance

                if self._frozen_cnt.get(sensor_id, 0) >= self._FROZEN_LIMIT:
                    distance = 9999

                # Route data to the correct multiprocessing queue sender
                if sensor_id == 0:
                    self.tof_right_sender.send(distance)
                elif sensor_id == 3:
                    self.tof_forward_sender.send(distance)
                    if self.debugging:
                        print(
                            f"\033[1;97m[ TOF sensor Thread ] :\033[0m \033[1;92mINFO\033[0m - Sent left distance: {distance} mm"
                        )
                elif sensor_id == 1:
                    self.tof_rear_sender.send(distance)
                    if self.debugging:
                        print(
                            f"\033[1;97m[ TOF sensor Thread ] :\033[0m \033[1;92mINFO\033[0m - Sent right distance: {distance} mm"
                        )
                elif sensor_id == 2:
                    self.tof_left_sender.send(distance)
                else:
                    pass

                # Remove the safely processed packet from the buffer
                del self._serial_buf[:TOF_length]

        except Exception as e:
            print(f"\033[1;91m[ TOF sensor Thread ] RUNTIME ERROR: {e}\033[0m")

