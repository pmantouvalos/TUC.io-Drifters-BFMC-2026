from enum import Enum

class ControlMode(Enum):
    ERROR                            = 0
    INIT                             = 1
    IDLE                             = 2
    MANUAL                           = 3
    AUTO                             = 4
    MOVE                             = 5
    STOP_SIGN                        = 6
    FIND_STOP_LINE                   = 7
    SIMPLE_PEDESTRIAN                = 8
    
    # Highway states
    ENTER_HIGHWAY                    = 10
    EXIT_HIGHWAY                     = 11
    
    # Crosswalk & Pedestrian
    CROSSWALK                        = 13
    PEDESTRIAN_AT_CROSSWALK          = 14
    MOVE_FOR_PEDESTRIAN_IN_CROSSWALK = 15
    
    # Parking states
    ENTER_PARKING                    = 29 # THE CAR GOES FORWARD UNTIL IT STOPS SEEING THE PARKING SIGNS THEN IT ENTERS THE FIND_PARKING_SPOT STATE :) 
    FIND_PARKING_SPOT                = 16
    PARKING_ALIGN                    = 17
    PARKING_REVERSE                  = 18
    PARKING_CORRECT                  = 19
    PARKING_FORWARD                  = 20
    PARKED                           = 21
    UNPARKING                        = 22
    PARKING_POSITION                 = 28 # Parking positioning (paper §4.4.6)

    # Intersection states
    INTERSECTION_FWD                 = 23
    INTERSECTION_LEFT                = 24
    INTERSECTION_RIGHT               = 25   
    PRIORITY                         = 26   
    ENTER_INTERSECTION               = 27   

    # Overtake & Traffic Light
    OVERTAKE                         = 12
    VEER_RIGHT                       = 30
    VEER_LEFT                        = 31
    CONTINUE_AHEAD                   = 32
    TRAFFIC_LIGHT                    = 33
    OVERTAKE_LEFT                    = 34
    OVERTAKE_RIGHT                   = 35
    
    # Roundabout states (ΝΕΑ)
    ROUNDABOUT_APPROACH              = 36
    ROUNDABOUT_YIELD                 = 37
    ROUNDABOUT_CRUISE                = 38
    ROUNDABOUT_EXIT                  = 39
    
    START                            = 40
    ENTER_HIGHWAY_2                  = 41
    EXIT_PARKING                     = 42
    EXIT_START                       = 43

    DIRECT_INTERSECTION              = 44
    AVOID_CAR                        = 45