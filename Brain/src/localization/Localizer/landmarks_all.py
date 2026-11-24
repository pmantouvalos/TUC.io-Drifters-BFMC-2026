import json
import os
import numpy as np

H = 13797.950

DIRS = {
    "EAST": 0.0,
    "NORTH": np.pi / 2,
    "WEST": np.pi,
    "SOUTH": -0.5 * np.pi ,  
}

raw = [
    {
        "id": "start",
        "type": "start",
        "points": [
            {"raw_x": 12066.0, "raw_y": 11591.0},
            {"raw_x": 12365.0, "raw_y": 11591.0},
            {"raw_x": 12365.0, "raw_y": 11938.0},
            {"raw_x": 12066.0, "raw_y": 11938.0},
        ],
        "facing": [DIRS["WEST"]]
    },
    
    # {
    #     "id": "start_line",
    #     "raw_x": 14105.0,
    #     "raw_y": 12697.0,
    #     "type": "STOP_LINE",
    #     "facing": [DIRS["NORTH"]]
    # },

    {"id": "STOP_LINE_1", "raw_x": 14757.0, "raw_y": 12880.0, "type": "STOP_LINE", "facing": [DIRS["WEST"]]},
    {"id": "STOP_LINE_2", "raw_x": 17067.0, "raw_y": 11451.0, "type": "STOP_LINE", "facing": [DIRS["SOUTH"]]},
    {"id": "STOP_LINE_3", "raw_x": 16167.0, "raw_y": 9789.0, "type": "STOP_LINE", "facing": [DIRS["EAST"]]},
    {"id": "STOP_LINE_4", "raw_x": 15273.0, "raw_y": 10854.0, "type": "STOP_LINE", "facing": [DIRS["NORTH"]]},
    {"id": "STOP_LINE_5", "raw_x": 15300.0, "raw_y": 12362.0, "type": "STOP_LINE", "facing": [DIRS["NORTH"]]},
    {"id": "STOP_LINE_6", "raw_x": 1985.0,  "raw_y": 13452.0, "type": "STOP_LINE", "facing": [DIRS["WEST"]]},
    {"id": "STOP_LINE_7", "raw_x": 4228.0,  "raw_y": 13452.0, "type": "STOP_LINE", "facing": [DIRS["WEST"]]},
    {"id": "STOP_LINE_8", "raw_x": 8193.0,  "raw_y": 12867.0, "type": "STOP_LINE", "facing": [DIRS["WEST"]]},
    {"id": "STOP_LINE_9", "raw_x": 3390.0,  "raw_y": 13081.0, "type": "STOP_LINE", "facing": [DIRS["EAST"]]},
    {"id": "STOP_LINE_10", "raw_x": 2511.0,  "raw_y": 12566.0, "type": "STOP_LINE", "facing": [DIRS["NORTH"]]},
    {"id": "STOP_LINE_11", "raw_x": 4763.0,  "raw_y": 12567.0, "type": "STOP_LINE", "facing": [DIRS["NORTH"]]},
    {"id": "STOP_LINE_12", "raw_x":  713.0,  "raw_y": 10694.0, "type": "STOP_LINE", "facing": [DIRS["SOUTH"]]},
    {"id": "STOP_LINE_13", "raw_x": 2896.0,  "raw_y": 10695.0, "type": "STOP_LINE", "facing": [DIRS["SOUTH"]]},
    {"id": "STOP_LINE_14", "raw_x": 5129.0,  "raw_y": 10695.0, "type": "STOP_LINE", "facing": [DIRS["SOUTH"]]},
    {"id": "STOP_LINE_15", "raw_x": 6573.0,  "raw_y": 10326.0, "type": "STOP_LINE", "facing": [DIRS["SOUTH"]]},
    {"id": "STOP_LINE_17", "raw_x": 1993.0,  "raw_y": 10178.0, "type": "STOP_LINE", "facing": [DIRS["WEST"]]},
    {"id": "STOP_LINE_18", "raw_x": 4230.0,  "raw_y": 10148.0, "type": "STOP_LINE", "facing": [DIRS["WEST"]]},
    {"id": "STOP_LINE_19", "raw_x": 1236.0,  "raw_y": 9776.0, "type": "STOP_LINE", "facing": [DIRS["EAST"]]},
    {"id": "STOP_LINE_20", "raw_x": 3393.0,  "raw_y": 9757.0, "type": "STOP_LINE", "facing": [DIRS["EAST"]]},
    {"id": "STOP_LINE_21", "raw_x": 5641.0,  "raw_y": 9787.0, "type": "STOP_LINE", "facing": [DIRS["EAST"]]},
    {"id": "STOP_LINE_22", "raw_x": 7500.0,  "raw_y": 9797.0, "type": "STOP_LINE", "facing": [DIRS["EAST"]]},
    {"id": "STOP_LINE_23", "raw_x": 9443.0, "raw_y": 9815.0, "type": "STOP_LINE", "facing": [DIRS["EAST"]]},
    {"id": "STOP_LINE_24", "raw_x":   346.0, "raw_y": 9284.0, "type": "STOP_LINE", "facing": [DIRS["NORTH"]]},
    {"id": "STOP_LINE_25", "raw_x": 2524.0,  "raw_y": 9287.0, "type": "STOP_LINE", "facing": [DIRS["NORTH"]]},
    {"id": "STOP_LINE_26", "raw_x": 4743.0,  "raw_y": 9283.0, "type": "STOP_LINE", "facing": [DIRS["NORTH"]]},
    {"id": "STOP_LINE_27", "raw_x": 15274.0, "raw_y": 9285.0, "type": "STOP_LINE", "facing": [DIRS["NORTH"]]},
    {"id": "STOP_LINE_28", "raw_x":  710.0,  "raw_y": 7810.0, "type": "STOP_LINE", "facing": [DIRS["SOUTH"]]},
    {"id": "STOP_LINE_29", "raw_x": 1985.0,  "raw_y": 7275.0, "type": "STOP_LINE", "facing": [DIRS["WEST"]]},
    {"id": "STOP_LINE_30", "raw_x": 2911.0,  "raw_y": 7808.0, "type": "STOP_LINE", "facing": [DIRS["SOUTH"]]},
    {"id": "STOP_LINE_31", "raw_x": 4228.0,  "raw_y": 7280.0, "type": "STOP_LINE", "facing": [DIRS["WEST"]]},
    {"id": "STOP_LINE_32", "raw_x": 5122.0,  "raw_y": 7806.0, "type": "STOP_LINE", "facing": [DIRS["SOUTH"]]},
    {"id": "STOP_LINE_33", "raw_x": 5637.0,  "raw_y": 6900.0, "type": "STOP_LINE", "facing": [DIRS["EAST"]]},
    {"id": "STOP_LINE_34", "raw_x": 1233.0,  "raw_y": 6912.0, "type": "STOP_LINE", "facing": [DIRS["EAST"]]},
    {"id": "STOP_LINE_35", "raw_x": 3389.0,  "raw_y": 6909.0, "type": "STOP_LINE", "facing": [DIRS["EAST"]]},
    {"id": "STOP_LINE_36", "raw_x": 356.0,   "raw_y": 6398.0, "type": "STOP_LINE", "facing": [DIRS["NORTH"]]},
    {"id": "STOP_LINE_37", "raw_x": 5151.0,  "raw_y": 6399.0, "type": "STOP_LINE", "facing": [DIRS["SOUTH"]]}, 
    {"id": "STOP_LINE_38", "raw_x": 4760.0,  "raw_y": 5936.0, "type": "STOP_LINE", "facing": [DIRS["NORTH"]]},
    {"id": "STOP_LINE_39", "raw_x":  722.0,  "raw_y": 4704.0, "type": "STOP_LINE", "facing": [DIRS["SOUTH"]]},
    {"id": "STOP_LINE_40", "raw_x": 1235.0,  "raw_y": 4196.0, "type": "STOP_LINE", "facing": [DIRS["WEST"]]}, 
    {"id": "STOP_LINE_41", "raw_x": 1698.0,  "raw_y": 3836.0, "type": "STOP_LINE", "facing": [DIRS["EAST"]]},
    {"id": "STOP_LINE_42", "raw_x":  347.0,  "raw_y": 3291.0, "type": "STOP_LINE", "facing": [DIRS["NORTH"]]},
    {"id": "STOP_LINE_43", "raw_x": 14995.0, "raw_y": 3420.0, "type": "STOP_LINE", "facing": [DIRS["WEST"]]},
    {"id": "STOP_LINE_44", "raw_x": 16232.0, "raw_y": 4276.0, "type": "STOP_LINE", "facing": [DIRS["SOUTH"]]},
    {"id": "STOP_LINE_45", "raw_x": 17073.0, "raw_y": 3047.0, "type": "STOP_LINE", "facing": [DIRS["EAST"]]},
    {"id": "STOP_LINE_46", "raw_x": 15863.0, "raw_y": 2195.0, "type": "STOP_LINE", "facing": [DIRS["NORTH"]]},

    # ==============================================================
    # RENAMED: Zebra Polygons (so they don't clash with the signs)
    # ==============================================================
    {
        "id": "crosswalk_1",
        "type": "Zebra",
        "points": [
            {"raw_x": 16918.0, "raw_y": 11390.0},
            {"raw_x": 16918.0, "raw_y": 11044.0},
            {"raw_x": 17229.0, "raw_y": 11044.0},
            {"raw_x": 17229.0, "raw_y": 11390.0},
        ],
        "facing": [DIRS["SOUTH"]]
    },
    {
        "id": "crosswalk_2",
        "type": "Zebra",
        "points": [
            {"raw_x": 15437.0, "raw_y": 10917.0},
            {"raw_x": 15437.0, "raw_y": 11262.0},
            {"raw_x": 15124.0, "raw_y": 11262.0},
            {"raw_x": 15124.0, "raw_y": 10917.0},
        ],
        "facing": [DIRS["NORTH"]]
    },
    {
        "id": "crosswalk_3",
        "type": "Zebra",
        "points": [
            {"raw_x": 8254.0, "raw_y": 12726.0},
            {"raw_x": 8254.0, "raw_y": 13039.0},
            {"raw_x": 8601.0, "raw_y": 13039.0},
            {"raw_x": 8601.0, "raw_y": 12726.0},
        ],
        "facing": [DIRS["WEST"]]
    },
    {
        "id": "crosswalk_4",
        "type": "Zebra",
        "points": [
            {"raw_x": 9038.0, "raw_y": 9655.0},
            {"raw_x": 9038.0, "raw_y": 10317.0},
            {"raw_x": 9386.0, "raw_y": 10317.0},
            {"raw_x": 9386.0, "raw_y": 9655.0},
        ],
        "facing": [DIRS["EAST"]]
    },
    {
        "id": "crosswalk_5",
        "type": "Zebra",
        "points": [
            {"raw_x": 4604.0, "raw_y": 5993.0},
            {"raw_x": 4604.0, "raw_y": 6342.0},
            {"raw_x": 4946.0, "raw_y": 6342.0},
            {"raw_x": 4946.0, "raw_y": 5993.0},
        ],
        "facing": [DIRS["NORTH"]]
    },
    {
        "id": "crosswalk_6",
        "type": "Zebra",
        "points": [
            {"raw_x": 1291.0, "raw_y": 3666.0},
            {"raw_x": 1291.0, "raw_y": 4007.0},
            {"raw_x": 1638.0, "raw_y": 4007.0},
            {"raw_x": 1638.0, "raw_y": 3666.0},
        ],
        "facing": [DIRS["EAST"]]
    },
    {
        "id": "crosswalk_7",
        "type": "Zebra",
        "points": [
            {"raw_x": 4947.0, "raw_y": 5993.0},
            {"raw_x": 4947.0, "raw_y": 6342.0},
            {"raw_x": 4946.0, "raw_y": 6342.0},
            {"raw_x": 4946.0, "raw_y": 5993.0},
        ],
        "facing": [DIRS["SOUTH"]]
    },
    {
        "id": "crosswalk_8",
        "type": "Zebra",
        "points": [
            {"raw_x": 1291.0, "raw_y": 4008.0},
            {"raw_x": 1291.0, "raw_y": 4353.0},
            {"raw_x": 1638.0, "raw_y": 4353.0},
            {"raw_x": 1638.0, "raw_y": 4008.0},
        ],
        "facing": [DIRS["WEST"]]
    },

    # ==============================================================
    # RENAMED: All point landmarks now match NN labels exactly
    # ==============================================================

    # Crosswalk Sign (NN outputs "Crosswalk")
    {"id": "crosswalk_sign_1", "raw_x": 17323.0, "raw_y": 11506.0, "type": "Crosswalk", "facing": [DIRS["SOUTH"]]},
    {"id": "crosswalk_sign_2", "raw_x": 15050.0, "raw_y": 10797.0, "type": "Crosswalk", "facing": [DIRS["NORTH"]]},
    {"id": "crosswalk_sign_3", "raw_x": 8145.0,  "raw_y": 13098.0, "type": "Crosswalk", "facing": [DIRS["WEST"]]},
    {"id": "crosswalk_sign_4", "raw_x": 9516.0,  "raw_y": 9553.0, "type": "Crosswalk", "facing": [DIRS["EAST"]]},
    {"id": "crosswalk_sign_5", "raw_x": 8895.0,  "raw_y": 10399.0, "type": "Crosswalk", "facing": [DIRS["WEST"]]}, 
    {"id": "crosswalk_sign_6", "raw_x": 5397.0,  "raw_y": 6454.0, "type": "Crosswalk", "facing": [DIRS["SOUTH"]]}, 
    {"id": "crosswalk_sign_7", "raw_x": 4508.0,  "raw_y": 5858.0, "type": "Crosswalk", "facing": [DIRS["NORTH"]]},
    {"id": "crosswalk_sign_8", "raw_x": 1173.0,  "raw_y": 4483.0, "type": "Crosswalk", "facing": [DIRS["WEST"]]},
    {"id": "crosswalk_sign_9", "raw_x": 1817.0,  "raw_y": 3558.0, "type": "Crosswalk", "facing": [DIRS["EAST"]]},

    # Highway Signs
    {"id": "highway_entry_1", "raw_x": 14537.0,  "raw_y": 2740.0, "type": "HighwayEntry", "facing": [DIRS["EAST"]]},
    {"id": "highway_exit_2", "raw_x": 14439.0,  "raw_y": 3724.0, "type": "HighwayExit", "facing": [DIRS["WEST"]]},
    {"id": "highway_entry_2", "raw_x": 6182.0,  "raw_y": 2327.0, "type": "HighwayEntry", "facing": [DIRS["WEST"]]},
    {"id": "highway_exit_1", "raw_x": 6116.0,  "raw_y": 1373.0, "type": "HighwayExit", "facing": [DIRS["EAST"]]},

    # Roundabout
    {"id": "roundabout_1", "raw_x": 16500.0,  "raw_y": 4372.0, "type": "Roundabout", "facing": [DIRS["SOUTH"]]},
    {"id": "roundabout_2", "raw_x": 17220.0,  "raw_y": 2776.0, "type": "Roundabout", "facing": [DIRS["EAST"]]},
    {"id": "roundabout_3", "raw_x": 15576.0,  "raw_y": 2084.0, "type": "Roundabout", "facing": [DIRS["NORTH"]]},
    {"id": "roundabout_4", "raw_x": 14908.0,  "raw_y": 3665.0, "type": "Roundabout", "facing": [DIRS["WEST"]]},

    # Parking
    {"id": "parking_1", "raw_x": 12987.0,  "raw_y": 13119.0, "type": "Parking", "facing": [DIRS["WEST"]]},
    {"id": "parking_2", "raw_x": 12992.0,  "raw_y": 12650.0, "type": "Parking", "facing": [DIRS["WEST"]]},
    {"id": "parking_3", "raw_x": 8935.0,  "raw_y": 13124.0, "type": "Parking", "facing": [DIRS["WEST"]]},
    {"id": "parking_4", "raw_x": 8940.0,  "raw_y": 12617.0, "type": "Parking", "facing": [DIRS["WEST"]]},

    # Priority
    # {"id": "priority_1", "raw_x": 4161.0,  "raw_y": 13680.0, "type": "Priority", "facing": [DIRS["WEST"]]},
    # {"id": "priority_2", "raw_x": 5364.0,  "raw_y": 10768.0, "type": "Priority", "facing": [DIRS["SOUTH"]]},
    # {"id": "priority_3", "raw_x": 4504.0,  "raw_y": 9180.0, "type": "Priority", "facing": [DIRS["NORTH"]]},
    # {"id": "priority_4", "raw_x": 115.0,  "raw_y": 9179.0, "type": "Priority", "facing": [DIRS["NORTH"]]},
    # {"id": "priority_5", "raw_x": 981.0,  "raw_y": 7934.0, "type": "Priority", "facing": [DIRS["SOUTH"]]},
    # {"id": "priority_6", "raw_x": 110.0,  "raw_y": 6298.0, "type": "Priority", "facing": [DIRS["NORTH"]]},

    # # Stop Sign
    # {"id": "stop_sign_1", "raw_x": 2269.0,  "raw_y": 12475.0, "type": "Stop", "facing": [DIRS["NORTH"]]},
    # {"id": "stop_sign_2", "raw_x": 4470.0,  "raw_y": 12457.0, "type": "Stop", "facing": [DIRS["NORTH"]]},
    # {"id": "stop_sign_3", "raw_x": 1376.0,  "raw_y": 9538.0, "type": "Stop", "facing": [DIRS["EAST"]]},
    # {"id": "stop_sign_4", "raw_x": 3155.0,  "raw_y": 7930.0, "type": "Stop", "facing": [DIRS["SOUTH"]]},
    # {"id": "stop_sign_5", "raw_x": 3450.0,  "raw_y": 6656.0, "type": "Stop", "facing": [DIRS["EAST"]]},
    # {"id": "stop_sign_6", "raw_x": 1373.0,  "raw_y": 6650.0, "type": "Stop", "facing": [DIRS["EAST"]]},
    # {"id": "stop_sign_7", "raw_x": 990.0,  "raw_y": 4843.0, "type": "Stop", "facing": [DIRS["SOUTH"]]},
    # {"id": "stop_sign_8", "raw_x": 110.0,  "raw_y": 3203.0, "type": "Stop", "facing": [DIRS["NORTH"]]},
    # {"id": "stop_sign_9", "raw_x": 16302.0,  "raw_y": 9542.0, "type": "Stop", "facing": [DIRS["EAST"]]},
    
    # Semaphores
    {"id": "semaphores_1", "raw_x": 12012.0,  "raw_y": 12040.0, "type": "TrafficLight", "facing": [DIRS["WEST"]]},
    {"id": "semaphores_2", "raw_x": 1838.0,  "raw_y": 10461.0, "type": "TrafficLight", "facing": [DIRS["WEST"]]},
    {"id": "semaphores_3", "raw_x": 2235.0,  "raw_y": 9167.0, "type": "TrafficLight", "facing": [DIRS["NORTH"]]},
    {"id": "semaphores_4", "raw_x": 3510.0,  "raw_y": 9545.0, "type": "TrafficLight", "facing": [DIRS["EAST"]]},
    {"id": "semaphores_5", "raw_x": 3125.0,  "raw_y": 10828,  "type": "TrafficLight", "facing": [DIRS["SOUTH"]]},

    # OneWay
    {"id": "oneway_1", "raw_x": 5322.0,  "raw_y": 13700.0, "type": "OneWay", "facing": [DIRS["WEST"]]}, 
    {"id": "oneway_2", "raw_x": 15600.0,  "raw_y": 13161.0, "type": "OneWay", "facing": [DIRS["WEST"]]},
    {"id": "oneway_3", "raw_x": 14560.0,  "raw_y": 9500.0, "type": "OneWay", "facing": [DIRS["EAST"]]},
    {"id": "oneway_4", "raw_x": 15000.0,  "raw_y": 10500.0, "type": "OneWay", "facing": [DIRS["NORTH"]]},
    
    # NoEntry
    {"id": "no_entry_1", "raw_x": 6940.0,  "raw_y": 10500.0, "type": "NoEntry", "facing": [DIRS["WEST"]]}, 
    {"id": "no_entry_2", "raw_x": 15650.0,  "raw_y": 12400.0, "type": "NoEntry", "facing": [DIRS["SOUTH"]]},
    {"id": "no_entry_3", "raw_x": 14880.0,  "raw_y": 12600.0, "type": "NoEntry", "facing": [DIRS["EAST"]]},
]

def transform_point(raw_x, raw_y):
    return round(raw_x / 1000, 3), round((H - raw_y) / 1000, 3)

for lm in raw:
    if "points" in lm:
        # 1. Transform all polygon corners first
        for p in lm["points"]:
            p["x"], p["y"] = transform_point(p["raw_x"], p["raw_y"])
            del p["raw_x"], p["raw_y"]
        
        # 2. Calculate the midpoint between the first two points (p0 and p1)
        p0 = lm["points"][0]
        p1 = lm["points"][3]
        
        lm["x"] = round((p0["x"] + p1["x"]) / 2, 3)
        lm["y"] = round((p0["y"] + p1["y"]) / 2, 3)

        # Clean up stray raw keys if they exist on parent
        lm.pop("raw_x", None)
        lm.pop("raw_y", None)
    else:
        lm["x"], lm["y"] = transform_point(lm["raw_x"], lm["raw_y"])
        del lm["raw_x"], lm["raw_y"]

output_path = os.path.join(os.path.dirname(__file__), "landmarks.json")

with open(output_path, "w") as f:
    json.dump({"landmarks": raw}, f, indent=2, default=lambda x: float(x) if isinstance(x, np.floating) else x)