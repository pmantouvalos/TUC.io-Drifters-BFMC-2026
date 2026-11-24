# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.
# (license header unchanged)

from enum import Enum

####################################### processCamera #######################################
class mainCamera(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 1
    msgType = "str"

class serialCamera(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 2
    msgType = "str"

class Recording(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 3
    msgType = "bool"

class Signal(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 4
    msgType = "str"

class LaneKeeping(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 5
    msgType = "int"

####################################### OAK-D #######################################
class IntersectionMask(Enum):
    Queue   = "Critical"
    Owner   = "processOakD"
    msgID   = 6
    msgType = "int"

class ProcessingCameraPrimary(Enum):
    Queue = "General"
    Owner = "processOakD"
    msgID = 1
    msgType = "str"

class ProcessingCameraSecondary(Enum):
    Queue = "General"
    Owner = "processOakD"
    msgID = 2
    msgType = "str"

class VisionManeuver(Enum):
    Queue = "Critical"
    Owner = "processOakD"
    msgID = 3
    msgType = "dict"

class ObjectDetect(Enum):
    Queue = "Critical"
    Owner = "processOakD"
    msgID = 4
    msgType = "dict"

# ── NEW: gate that enables/disables parking bay scan in processOakD ──────────
class ParkingScanGate(Enum):
    Queue   = "Critical"
    Owner   = "processOakD"
    msgID   = 5          # next free ID in Critical after ObjectDetect(4)
    msgType = "bool"
# ─────────────────────────────────────────────────────────────────────────────

class LeftLineClassifier(Enum):
    Queue = "General"
    Owner = "processOakD"
    msgID = 8
    msgType = "str"

class RightLineClassifier(Enum):
    Queue = "General"
    Owner = "processOakD"
    msgID = 9
    msgType = "str"
####################################### TOF sensor ###################################

class TOFRear(Enum):
    Queue = "Critical"
    Owner = "threadtof_sensor"
    msgID = 1
    msgType = "int"

class TOFLeft(Enum):
    Queue = "Critical"
    Owner = "threadtof_sensor"
    msgID = 2
    msgType = "int"

class TOFRight(Enum):
    Queue = "Critical"
    Owner = "threadtof_sensor"
    msgID = 3
    msgType = "int"

class TOFForward(Enum):
    Queue = "Critical"
    Owner = "threadtof_sensor"
    msgID = 4
    msgType = "int"

####################################### JoyUDP #######################################
class JoyAxes(Enum):
    Queue = "General"
    Owner = "threadjoy_udp"
    msgID = 1
    msgType = "dict"

class JoyButton(Enum):
    Queue = "Critical"
    Owner = "threadjoy_udp"
    msgID = 2
    msgType = "int"

################################# processCarsAndSemaphores ##################################
class Cars(Enum):
    Queue = "General"
    Owner = "threadCarsAndSemaphores"
    msgID = 1
    msgType = "dict"

class Semaphores(Enum):
    Queue = "General"
    Owner = "threadCarsAndSemaphores"
    msgID = 2
    msgType = "dict"

################################# From Dashboard ##################################
class SpeedMotor(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 1
    msgType = "str"

class SteerMotor(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 2
    msgType = "str"

class Control(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 3
    msgType = "dict"

class Brake(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 4
    msgType = "str"

class Record(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 5
    msgType = "str"

class Config(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 6
    msgType = "dict"

class Klem(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 7
    msgType = "str"

class DrivingMode(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 8
    msgType = "str"

class ToggleInstant(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 9
    msgType = "str"

class ToggleBatteryLvl(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 10
    msgType = "str"

class ToggleImuData(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 11
    msgType = "str"

class ToggleResourceMonitor(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 12
    msgType = "str"

class State(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 13
    msgType = "str"

class Brightness(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 14
    msgType = "str"

class Contrast(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 15
    msgType = "str"

class DropdownChannelExample(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 16
    msgType = "str"

class SliderChannelExample(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 17
    msgType = "str"

class ControlCalib(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 18
    msgType = "dict"

class IsAlive(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 19
    msgType = "bool"

class RequestSteerLimits(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 20
    msgType = "bool"

class ThresholdMin(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 21
    msgType = "str"

class ThresholdMax(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 22
    msgType = "str"

class StanleyGain(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 23
    msgType = "str"

class UpSlice(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 24
    msgType = "str"

class DownSlice(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 25
    msgType = "str"

class NavigateTo(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 26
    msgType = "dict"

################################# From Nucleo ##################################
class BatteryLvl(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 1
    msgType = "int"

class ImuData(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 2
    msgType = "str"

class InstantConsumption(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 3
    msgType = "float"

class ResourceMonitor(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 4
    msgType = "dict"

class CurrentSpeed(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 5
    msgType = "float"

class CurrentSteer(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 6
    msgType = "float"

class ImuAck(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 7
    msgType = "str"

class ShutDownSignal(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 8
    msgType = "str"

class CalibPWMData(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 9
    msgType = "dict"

class CalibRunDone(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 10
    msgType = "bool"

class AliveSignal(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 11
    msgType = "bool"

class SteeringLimits(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 12
    msgType = "dict"

################################# From Controller ##################################
class CurrentNode(Enum):
    Queue = "General"
    Owner = "threadcontroller"
    msgID = 1
    msgType = "str"

################################# From Locsys ##################################
class Location(Enum):
    Queue = "General"
    Owner = "threadTrafficCommunication"
    msgID = 1
    msgType = "dict"

class DWM_pos(Enum):
    Queue = "General"
    Owner = "dwm1001_reader"
    msgID = 2
    msgType = "dict"

######################    From processSerialHandler  ###########################
class EnableButton(Enum):
    Queue = "General"
    Owner = "threadWrite"
    msgID = 1
    msgType = "bool"

class WarningSignal(Enum):
    Queue = "General"
    Owner = "brain"
    msgID = 2
    msgType = "str"

class SerialConnectionState(Enum):
    Queue = "General"
    Owner = "processSerialHandler"
    msgID = 3
    msgType = "bool"

################################# From StateMachine ##################################
class StateChange(Enum):
    Queue = "Critical"
    Owner = "stateMachine"
    msgID = 1
    msgType = "str"

################################# From Localizer ##################################
class LiveAnimation(Enum):
    Queue = "General"
    Owner = "threadLocalizer"
    msgID = 1
    msgType = "dict"

class LocalizerPose(Enum):
    Queue = "General"
    Owner = "threadLocalizer"
    msgID = 2
    msgType = "dict"

################################# From Navigator ##################################
class NavCommand(Enum):
    Queue = "General"
    Owner = "threadNavigator"
    msgID = 1
    msgType = "dict"

### It will have this format: {"WarningName":"name1", "WarningID": 1}
