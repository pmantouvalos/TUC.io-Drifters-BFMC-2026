import json


def load_landmarks(json_path):
    with open(json_path, "r") as f:
        data = json.load(f)

    return data["landmarks"]


def load_point_landmarks(data):
    pts = []
    for lm in data:
        if "x" in lm and "y" in lm:
            pt = {"id": lm["id"], "type": lm["type"], "x": lm["x"], "y": lm["y"]}
            if "facing" in lm:
                pt["facing"] = lm["facing"]
            pts.append(pt)
    return pts


def filter_by_type(landmarks, lm_type):
    return [lm for lm in landmarks if lm["type"] == lm_type]
