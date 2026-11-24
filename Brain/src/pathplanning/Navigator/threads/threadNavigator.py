import time
import networkx as nx
import numpy as np
import traceback
import os
from pathlib import Path

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import LocalizerPose, NavCommand
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender


class threadNavigator(ThreadWithStop):
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        self.current_path = []

        # ==========================================
        # WAYPOINTS SEQUENCE
        # ==========================================
        self.target_nodes = ["97", "198", "6", "91", "75", "54", "372"]

        # Distance (meters) before the intersection to send the turn command
        self.lookahead_distance = 0.50
        self.last_command_time = 0

        # ==========================================
        # CONFIGURATION: Nodes & Overrides
        # ==========================================
        self.INTERSECTION_NODES = [
            "94",
            "21",
            "9",
            "103",
            "33",
            "45",
            "76",
            # "200",
            "85",
            "57",
            "207",
            # "78",
            "210",
            # "59",
            "22",
            "48",
            "35",
            "95",
            "105",
            "96",
            # "11",
            "104",
            "36",
            "46",
            # "47",
            # "290",
            "77",
            # "60",
            "23",
            # "333", "335", "338", "341",  # ROUNDABOUT NODES
        ]

        # self.ROUNDABOUT_NODES = ["333", "335", "338", "341"]

        self.SPECIAL_TURNS = {
            ("186", "207", "211"): "TURN_LEFT",
            ("186", "207", "187"): "GO_STRAIGHT",
            ("206", "207", "187"): "TURN_LEFT",
            ("75", "78", "70"): "GO_STRAIGHT",
            ("75", "78", "72"): "TURN_LEFT",
            ("186", "210", "187"): "GO_STRAIGHT",
            ("186", "210", "211"): "TURN_LEFT",
            ("54", "59", "55"): "TURN_RIGHT",
            ("54", "59", "49"): "GO_STRAIGHT",
            ("54", "57", "55"): "TURN_RIGHT",
            ("54", "57", "49"): "GO_STRAIGHT",
            ("334", "335", "342"): "TURN_RIGHT",
            ("337", "338", "398"): "TURN_RIGHT",
            ("340", "341", "318"): "TURN_RIGHT",
        }

        # --- DYNAMIC ROUTING & LANE CONFIG ---
        self.NODES_TO_AVOID = []  # Add node IDs here to blacklist them

        # Nodes where the car MUST hug a specific lane
        self.FORCE_LEFT_LANE_NODES = []
        self.FORCE_RIGHT_LANE_NODES = []

        self.current_lane_pref = "center"

        self._init_graph()
        self.subscribe()
        super(threadNavigator, self).__init__()

    def _init_graph(self):
        try:
            # Note: Verify your username is 'pi'
            graph_path = "/home/pi/Documents/Brain/src/utils/Competition_track_graph.graphml"
            if not os.path.exists(graph_path):
                print(f"\033[1;97m[ Navigator ] :\033[0m \033[1;91mERROR\033[0m - Map not found at {graph_path}!")

                self.G = None
                return

            self.G = nx.read_graphml(graph_path)
            print("\033[1;97m[ Navigator ] :\033[0m \033[1;92mINFO\033[0m - Track graph loaded successfully!")

        except Exception as e:
            print(f"\033[1;97m[ Navigator ] :\033[0m \033[1;91mERROR\033[0m - Failed to load graph: {e}")
            self.G = None

    def subscribe(self):
        self.poseSubscriber = messageHandlerSubscriber(
            self.queuesList, LocalizerPose, "lastOnly", True
        )
        self.commandSender = messageHandlerSender(self.queuesList, NavCommand)

    def state_change_handler(self):
        pass

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def get_node_pos(self, node):
        return float(self.G.nodes[node]["x"]), float(self.G.nodes[node]["y"])

    def find_closest_node(self, x, y):
        best_node = None
        best_dist = float("inf")
        for node in self.G.nodes():
            nx_x, nx_y = self.get_node_pos(node)
            dist = np.hypot(nx_x - x, nx_y - y)
            if dist < best_dist:
                best_dist = dist
                best_node = node
        return best_node

    def get_turn_command(self, nodeA, nodeB, nodeC):
        # --- Priority 1. THE MANUAL OVERRIDE (Exceptions) ---
        route_sequence = (nodeA, nodeB, nodeC)
        if route_sequence in self.SPECIAL_TURNS:
            return self.SPECIAL_TURNS[route_sequence]

        # --- Priority 2. THE FILTER ---
        if nodeB not in self.INTERSECTION_NODES:
            return "GO_STRAIGHT"

        # --- Priority 3. STANDARD VECTOR GEOMETRY ---
        xA, yA = self.get_node_pos(nodeA)
        xB, yB = self.get_node_pos(nodeB)
        xC, yC = self.get_node_pos(nodeC)

        theta_in = np.arctan2(yB - yA, xB - xA)
        theta_out = np.arctan2(yC - yB, xC - xB)

        delta_theta = self.normalize_angle(theta_out - theta_in)
        threshold = np.deg2rad(30)

        if delta_theta > threshold:
            command = "TURN_LEFT"
        elif delta_theta < -threshold:
            command = "TURN_RIGHT"
        else:
            command = "GO_STRAIGHT"

        # --- Priority 4. THE ROUNDABOUT CLAMP ---
        # if nodeB in self.ROUNDABOUT_NODES and command == "TURN_LEFT":
        #    command = "GO_STRAIGHT"

        return command

    def print_remaining_mission_itinerary(self, start_node):
        """
        Calculates and prints the master itinerary traversing all remaining waypoints.
        """
        if not self.target_nodes:
            return

        print("\n\033[1;95m" + "=" * 60)
        print("GLOBAL MISSION ITINERARY (Remaining Waypoints)")
        print("=" * 60 + "\033[0m")

        safe_nodes = [n for n in self.G.nodes if n not in self.NODES_TO_AVOID]
        safe_G = self.G.subgraph(safe_nodes)

        current_loc = start_node
        step_counter = 1

        for wp in self.target_nodes:
            try:
                # Try to find a safe path to the next waypoint
                sub_path = nx.shortest_path(safe_G, source=current_loc, target=wp)
            except nx.NetworkXNoPath:
                try:
                    # Fallback to full map if blocked
                    sub_path = nx.shortest_path(self.G, source=current_loc, target=wp)
                except nx.NetworkXNoPath:
                    print(
                        f"   \033[1;91mERROR: Cannot plan any path to waypoint {wp}\033[0m"
                    )
                    break

            # Extract maneuvers for this specific leg
            for i in range(len(sub_path) - 2):
                nodeA = sub_path[i]
                nodeB = sub_path[i + 1]
                nodeC = sub_path[i + 2]

                if nodeB in self.INTERSECTION_NODES:
                    cmd = self.get_turn_command(nodeA, nodeB, nodeC)
                    print(f"   {step_counter}. Intersection {nodeB:<4} ➔  {cmd}")
                    step_counter += 1

            print(f"   \033[1;92mARRIVE AT WAYPOINT: {wp}\033[0m")
            current_loc = wp  # Set start point for next leg to this waypoint

        print("\033[1;95m" + "=" * 60 + "\033[0m\n")

    # --- MAIN LOOP ---
    def thread_work(self):
        try:
            if self.G is None:
                time.sleep(1)
                return

            if self.poseSubscriber.is_data_in_pipe():
                pose_data = self.poseSubscriber.receive()
                if not pose_data:
                    return

                x, y, theta = pose_data.get("pose", [0, 0, 0])
                mode = pose_data.get("mode", "GLOBAL")

                # 1. LOST (Waiting for Localization)
                if mode in ["GLOBAL", "RECOVER"]:
                    self.current_path = []
                    if time.time() - self.last_command_time > 1.0:
                        print(
                            "\033[1;93m[ NAVIGATOR ] Car is Lost. Waiting for localization...\033[0m"
                        )
                        # No longer sending "WANDER" command. Just wait.
                        self.last_command_time = time.time()
                    return

                # 2. LOCAL (Confident)
                if mode == "LOCAL":
                    # --- 1. SEND MASTER QUEUE ONCE ---
                    if not getattr(self, "_master_queue_sent", False):
                        if len(self.target_nodes) == 0:
                            return

                        start_node = self.find_closest_node(x, y)
                        master_sequence = []
                        current_loc = start_node

                        safe_nodes = [
                            n for n in self.G.nodes if n not in self.NODES_TO_AVOID
                        ]
                        safe_G = self.G.subgraph(safe_nodes)

                        print(
                            "\n\033[1;95m[ NAVIGATOR ] Calculating Global Mission Itinerary...\033[0m"
                        )

                        for wp in self.target_nodes:
                            try:
                                sub_path = nx.shortest_path(
                                    safe_G, source=current_loc, target=wp
                                )
                            except nx.NetworkXNoPath:
                                try:
                                    sub_path = nx.shortest_path(
                                        self.G, source=current_loc, target=wp
                                    )
                                except nx.NetworkXNoPath:
                                    print(
                                        f"\033[1;91m[ NAVIGATOR ] ERROR - No path found to {wp}!\033[0m"
                                    )
                                    return

                            # Extract maneuvers for this leg
                            for i in range(len(sub_path) - 2):
                                nodeA, nodeB, nodeC = (
                                    sub_path[i],
                                    sub_path[i + 1],
                                    sub_path[i + 2],
                                )
                                if nodeB in self.INTERSECTION_NODES:
                                    master_sequence.append(
                                        self.get_turn_command(nodeA, nodeB, nodeC)
                                    )

                            current_loc = wp  # Chain to the next waypoint

                        print(
                            f"\033[1;96m[ NAVIGATOR ] MASTER SEQUENCE SENT: {master_sequence}\033[0m"
                        )

                        # Send the entire list ONCE
                        self.commandSender.send({"maneuver_sequence": master_sequence})
                        self._master_queue_sent = True

                        # Initialize tracking to the first target
                        self.current_path = nx.shortest_path(
                            self.G, source=start_node, target=self.target_nodes[0]
                        )

                    # --- 2. DYNAMIC LANE ASSIGNMENT ---
                    if len(self.current_path) >= 2:
                        upcoming_node = self.current_path[1]
                        desired_lane = "center"
                        if upcoming_node in self.FORCE_LEFT_LANE_NODES:
                            desired_lane = "left"
                        elif upcoming_node in self.FORCE_RIGHT_LANE_NODES:
                            desired_lane = "right"

                        if desired_lane != self.current_lane_pref:
                            self.current_lane_pref = desired_lane
                            self.commandSender.send({"lane_preference": desired_lane})

                    # --- 3. TRACK PROGRESS TO CURRENT WAYPOINT ---
                    if len(self.current_path) >= 3:
                        nodeB = self.current_path[1]
                        xB, yB = self.get_node_pos(nodeB)

                        # Pop the physical node as we pass it
                        if np.hypot(xB - x, yB - y) < self.lookahead_distance:
                            self.current_path.pop(0)

                    # --- 4. WAYPOINT REACHED ---
                    elif len(self.current_path) == 2:
                        nodeTarget = self.current_path[1]
                        xT, yT = self.get_node_pos(nodeTarget)

                        # Widened radius to 0.8m so it reliably triggers
                        if np.hypot(xT - x, yT - y) < 0.8:
                            if time.time() - self.last_command_time > 1.0:
                                reached_node = self.target_nodes.pop(0)
                                print(f"\033[1;97m[ Navigator ] :\033[0m \033[1;92mINFO\033[0m - Waypoint {reached_node} reached!")
                                self.last_command_time = time.time()

                                if len(self.target_nodes) == 0:
                                    self.commandSender.send(
                                        {"command": "STOP_AT_DESTINATION"}
                                    )
                                else:
                                    # Target the next waypoint just for lane tracking.
                                    # Does NOT send a new queue because _master_queue_sent is True.
                                    start_node = self.find_closest_node(x, y)
                                    self.current_path = nx.shortest_path(
                                        self.G,
                                        source=start_node,
                                        target=self.target_nodes[0],
                                    )

        except Exception as e:
            print(f"\033[1;97m[ Navigator ] :\033[0m \033[1;91mERROR\033[0m - Thread crashed: {e}")
            traceback.print_exc()

        time.sleep(0.05)
