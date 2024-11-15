#!/usr/bin/env python3
from typing import Optional, Tuple, List, Dict
from argparse import ArgumentParser
from math import inf, sqrt, atan2, pi
from time import sleep, time
import queue
import json

import numpy as np
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist, Point32, PoseStamped, Pose, Vector3, Quaternion, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import euler_from_quaternion

# AABB format: (x_min, x_max, y_min, y_max)
OBS_TYPE = Tuple[float, float, float, float]
# Position format: {"x": x, "y": y, "theta": theta}
POSITION_TYPE = Dict[str, float]

# don't change this
GOAL_THRESHOLD = 0.1


def angle_to_0_to_2pi(angle: float) -> float:
    while angle < 0:
        angle += 2 * pi
    while angle > 2 * pi:
        angle -= 2 * pi
    return angle


class PIDController:
    """
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    """

    def __init__(self, kP, kI, kD, kS, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kS = kS
        self.err_int = 0
        self.err_dif = 0
        self.err_prev = 0
        self.err_hist = []
        self.t_prev = 0
        self.u_min = u_min
        self.u_max = u_max

    def control(self, err, t):
        ######### Your code starts here #########
        dt = t - self.t_prev
        self.err_hist.append(err)
        self.err_int += err
        if len(self.err_hist) > self.kS:
            self.err_int -= self.err_hist.pop(0)
        self.err_dif = err - self.err_prev
        u = (self.kP * err) + (self.kI * self.err_int * dt) + (self.kD * self.err_dif / dt)
        self.err_prev = err
        self.t_prev = t
        return max(self.u_min, min(u, self.u_max))
        ######### Your code ends here #########


class Node:
    def __init__(self, position: POSITION_TYPE, parent: "Node"):
        self.position = position
        self.neighbors = []
        self.parent = parent

    def distance_to(self, other_node: "Node") -> float:
        return np.linalg.norm(self.position - other_node.position)

    def to_dict(self) -> Dict:
        return {"x": self.position[0], "y": self.position[1]}

    def __str__(self) -> str:
        return (
            f"Node<pos: {round(self.position[0], 4)}, {round(self.position[1], 4)}, #neighbors: {len(self.neighbors)}>"
        )


class RrtPlanner:

    def __init__(self, obstacles: List[OBS_TYPE], map_aabb: Tuple):
        self.obstacles = obstacles
        self.map_aabb = map_aabb
        self.graph_publisher = rospy.Publisher("/rrt_graph", MarkerArray, queue_size=10)
        self.plan_visualization_pub = rospy.Publisher("/waypoints", MarkerArray, queue_size=10)
        self.delta = 0.1
        self.obstacle_padding = 0.15
        self.goal_threshold = GOAL_THRESHOLD

    def visualize_plan(self, path: List[Dict]):
        marker_array = MarkerArray()
        for i, waypoint in enumerate(path):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position = Point(waypoint["x"], waypoint["y"], 0.0)
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker.scale = Vector3(0.075, 0.075, 0.1)
            marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.5)
            marker_array.markers.append(marker)
        self.plan_visualization_pub.publish(marker_array)

    def visualize_graph(self, graph: List[Node]):
        marker_array = MarkerArray()
        for i, node in enumerate(graph):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale = Vector3(0.05, 0.05, 0.05)
            marker.pose.position = Point(node.position[0], node.position[1], 0.01)
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.5)
            marker_array.markers.append(marker)
        self.graph_publisher.publish(marker_array)

    def _randomly_sample_q(self) -> Node:

        ######### Your code starts here #########
        x_min, x_max, y_min, y_max = self.map_aabb
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        return Node(np.array([x, y]), None)
        ######### Your code ends here #########

    def _nearest_vertex(self, graph: List[Node], q: Node) -> Node:
        ######### Your code starts here #########
        min_dist = inf
        nearest_vertex = None
        for v in graph:
            dist = v.distance_to(q)
            if dist < min_dist:
                nearest_vertex = v
                min_dist = dist
        return nearest_vertex
        ######### Your code ends here #########

    def _is_in_collision(self, q_rand: Node):
        x = q_rand.position[0]
        y = q_rand.position[1]
        for obs in self.obstacles:
            x_min, x_max, y_min, y_max = obs
            x_min -= self.obstacle_padding
            y_min -= self.obstacle_padding
            x_max += self.obstacle_padding
            y_max += self.obstacle_padding
            if (x_min < x and x < x_max) and (y_min < y and y < y_max):
                return True
        return False

    def _extend(self, graph: List[Node], q_rand: Node):
        ######### Your code starts here #########
        if self._is_in_collision(q_rand):
            return None

        q_near = self._nearest_vertex(graph, q_rand)
        dist = q_near.distance_to(q_rand)
        if dist < self.delta:
            x_new = Node(q_rand.position.copy(), q_near)
        else:
            x_new_pos = q_near.position + ((q_rand.position - q_near.position) / dist * self.delta)
            x_new = Node(x_new_pos, q_near)

        if self._is_in_collision(x_new):
            return None

        q_near.neighbors.append(x_new)
        graph.append(x_new)
        return x_new
        ######### Your code ends here #########

    def generate_plan(self, start: POSITION_TYPE, goal: POSITION_TYPE) -> Tuple[List[POSITION_TYPE], List[Node]]:
        """Public facing API for generating a plan. Returns the plan and the graph.

        Return format:
            plan:
            [
                {"x": start["x"], "y": start["y"]},
                {"x": ...,      "y": ...},
                            ...
                {"x": goal["x"],  "y": goal["y"]},
            ]
            graph:
                [
                    Node<pos: x1, y1, #neighbors: n_1>,
                    ...
                    Node<pos: x_n, y_n, #neighbors: z>,
                ]
        """
        graph = [Node(np.array([start["x"], start["y"]]), None)]
        goal_node = Node(np.array([goal["x"], goal["y"]]), None)
        plan = []
        ######### Your code starts here #########

        while True:
            q_rand = self._randomly_sample_q()
            x_new = self._extend(graph, q_rand)
            if x_new is not None:
                if x_new.distance_to(goal_node) < self.goal_threshold:
                    print("success - found a path")
                    goal_node.parent = x_new
                    break

        curr_node = goal_node
        while curr_node.parent is not None:
            plan.append(curr_node.parent.to_dict())
            curr_node = curr_node.parent
        plan.reverse()

        ######### Your code ends here #########
        return plan, graph


# Protip: copy the ObstacleFreeWaypointController class from lab5.py here


######### Your code starts here #########
class ObstacleFreeWaypointController:
    def __init__(self, waypoints: List[Dict]):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.current_position = None
        self.goal_position = goal_position
        self.waypoints = waypoints

        self.linear_controller = PIDController(0.35, 0.01, 0.2, 5, -0.22, 0.22)
        self.angular_controller = PIDController(0.5, 0.0, 0.2, 10, -2.84, 2.84)
        self.goal_threshold = GOAL_THRESHOLD

    def odom_callback(self, msg):
        # Extracting current position from Odometry message
        pose = msg.pose.pose
        orientation = pose.orientation
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_position = {"x": pose.position.x, "y": pose.position.y, "theta": theta}

    def calculate_error(self, goal_position) -> Optional[Tuple]:
        if self.current_position is None:
            return None

        # Calculating error in position and orientation
        dx = goal_position["x"] - self.current_position["x"]
        dy = goal_position["y"] - self.current_position["y"]
        distance_error = sqrt(dx**2 + dy**2)

        desired_theta = atan2(dy, dx)
        angle_error = desired_theta - self.current_position["theta"]

        # Ensure angle error is within -pi to pi range
        if angle_error > pi:
            angle_error -= 2 * pi
        elif angle_error < -pi:
            angle_error += 2 * pi

        return distance_error, angle_error

    def control_robot(self):

        while (self.current_position is None) and (not rospy.is_shutdown()):
            rospy.loginfo("Waiting for current position...")
            rospy.sleep(0.1)

        rate = rospy.Rate(20)  # 20 Hz
        current_waypoint_idx = 0

        while not rospy.is_shutdown():

            current_waypoint = self.waypoints[current_waypoint_idx]

            error = self.calculate_error(current_waypoint)
            if error is None:
                continue
            distance_error, angle_error = error

            # Check if close enough to the goal
            if distance_error < self.goal_threshold:
                if current_waypoint_idx < len(self.waypoints) - 1:
                    rospy.loginfo("waypoint reached")
                    current_waypoint_idx += 1
                    continue
                rospy.loginfo("final waypoint reached")
                break

            # Calculate control commands using PID controllers
            cmd_linear_vel = self.linear_controller.control(distance_error, rospy.get_time())
            cmd_angular_vel = self.angular_controller.control(angle_error, rospy.get_time())

            # Publish control commands to /cmd_vel topic
            ctrl_msg = Twist()
            ctrl_msg.linear.x = cmd_linear_vel
            ctrl_msg.angular.z = cmd_angular_vel
            self.vel_pub.publish(ctrl_msg)

            # Print for debugging purposes
            rospy.loginfo(
                f"distance to target: {distance_error:.2f}\tangle error: {angle_error:.2f}\tcommanded linear vel: {cmd_linear_vel:.2f}\tcommanded angular vel: {cmd_angular_vel:.2f}"
            )
            rate.sleep()


######### Your code ends here #########


""" Example usage

rosrun development lab10.py --map_filepath src/csci445l/scripts/lab10_map.json
"""


if __name__ == "__main__":

    parser = ArgumentParser()
    parser.add_argument("--map_filepath", type=str, required=True)
    args = parser.parse_args()
    with open(args.map_filepath, "r") as f:
        map_ = json.load(f)
        goal_position = map_["goal_position"]
        obstacles = map_["obstacles"]
        map_aabb = map_["map_aabb"]
        start_position = {"x": 0.0, "y": 0.0}

    rospy.init_node("rrt_planner")
    planner = RrtPlanner(obstacles, map_aabb)
    plan, graph = planner.generate_plan(start_position, goal_position)
    planner.visualize_plan(plan)
    planner.visualize_graph(graph)
    controller = ObstacleFreeWaypointController(plan)

    try:
        while not rospy.is_shutdown():
            controller.control_robot()
    except rospy.ROSInterruptException:
        print("Shutting down...")
