#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

from .capture import detect_objects
import cv2
from .camera_view import Camera
from geometry_msgs.msg import Twist, Vector3
import math
import numpy as np
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleStatus
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool
import sys
import threading


MAX_SPEED = 1.0
MIN_SPEED = 0.0
MAX_YAW_SPEED = 0.5

MISSION_TOLERANCE = 0.25
ANGLE_TOLERANCE = 5

HALF_PI = 1.570796325

N = 0
E = 90
S = 180
W = 270


class OffboardControl(Node):

    def __init__(self, mission_mode, mission_steps):
        super().__init__('minimal_publisher')

        self.uav_direction = 'N'
        self.mission_mode = mission_mode
        self.mission_steps = mission_steps
        self.mission_index = 0
        self.armed = False
        self.cmdloop_control = 0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # create subscriptions
        self.status_subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        self.offboard_velocity_subscription = self.create_subscription(
            Twist,
            '/offboard_velocity_cmd',
            self.offboard_velocity_callback,
            qos_profile
        )

        self.attitude_subscription = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile
        )

        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )

        self.my_bool_subscription = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,
            qos_profile
        )

        # create publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_velocity = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.arm_publisher = self.create_publisher(Bool, '/arm_message', qos_profile)

        # create callback function for the arm timer (period is arbitrary, just should be more than 2Hz)
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # create callback function for the command loop (period is arbitrary, just should be more than 2Hz)
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        self.yaw = 0.0 # yaw value we send as command
        self.true_yaw = 0.0 # current yaw value of drone
        self.offboard_mode = False
        self.flight_check = False
        self.my_control = 0
        self.arm_message = False
        self.failsafe = False

        # states with corresponding callback functions that run once when state switches
        self.states = {
            "IDLE": self.state_init,
            "ARMING": self.state_arming,
            "TAKEOFF": self.state_takeoff,
            "LOITER": self.state_loiter,
            "OFFBOARD": self.state_offboard
        }

        self.current_state = "IDLE"
        self.last_state = self.current_state

    # callback function that arms, takes off, and switches to offboard mode (implements a finite state machine)
    def arm_timer_callback(self):
        match self.current_state:
            case "IDLE":
                if(self.flight_check and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.flight_check)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.my_control > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")

                self.arm() # send arm command

            case "TAKEOFF":
                if(not(self.flight_check)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")

                self.arm() # send arm command
                self.take_off() # send takeoff command

            # wait in this state while taking off, and the moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER":
                if(not(self.flight_check)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")

                self.arm()

            case "OFFBOARD":
                if(not(self.flight_check) or self.arm_state == VehicleStatus.ARMING_STATE_DISARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")

                self.state_offboard()

        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.my_control += 1


    def state_init(self):
        self.my_control = 0


    def state_arming(self):
        self.my_control = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")


    def state_takeoff(self):
        self.my_control = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
        self.get_logger().info("Takeoff command send")


    def state_loiter(self):
        self.my_control = 0
        self.get_logger().info("Loiter Status")


    def state_offboard(self):
        self.my_control = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboard_mode = True

    # arms the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
        self.get_logger().info("Takeoff command send")

    # publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7 # altitude value in takeoff command
        msg.command = command # command ID
        msg.target_system = 1 # system which should execute the command
        msg.target_component = 1 # component which should execute the command, 0 for all components
        msg.source_system = 1 # system sending the command
        msg.source_component = 1 # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.publisher_vehicle_command.publish(msg)

    # receives and sets vehicle status values
    def vehicle_status_callback(self, msg):
        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")

        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")

        if (msg.pre_flight_checks_pass != self.flight_check):
            self.get_logger().info(f"Flight check: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flight_check = msg.pre_flight_checks_pass

    # receives Twist commands from Teleop and converts FRD -> FLU
    def offboard_velocity_callback(self, msg):
        self.velocity.x = -msg.linear.y
        self.velocity.y = msg.linear.x

        self.velocity.z = -msg.linear.z # Z (FLU) is -Z (FRD)

        self.yaw = msg.angular.z # conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.true_yaw)

    # receives current trajectory values from drone and grabs the yaw value of the orientation
    def attitude_callback(self, msg):
        orientation_q = msg.q

        # true_yaw is the drones current yaw value
        self.true_yaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]),
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        
    # receives odometry values for each UAV
    def odometry_callback(self, msg):
        """
        Odometry message fields:
            timestamp: int
            timestamp_sample: int
            pose_frame: int
            position: float[3]
            q: float[4]
            velocity_frame: int
            velocity: float[3]
            angular_velocity: float[3]
            position_variance: float[3]
            orientation_variance: float[3]
            velocity_variance: float[3]
        """

        self.odometry = msg

    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")

    # publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if not self.armed and self.mission_mode:
            arm_message = Bool()
            arm_message.data = True

            self.arm_publisher.publish(arm_message)
            self.armed = True

        if(self.offboard_mode == True):
            self.cmdloop_control += 1

            # publish offboard control modes
            offboard_message = OffboardControlMode()
            offboard_message.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_message.position = False
            offboard_message.velocity = True
            offboard_message.acceleration = False
            self.publisher_offboard_mode.publish(offboard_message)

            # compute velocity in the direction of the current mission step
            if self.mission_mode:
                arrived = self.velocity_to_destiny()

                if arrived:
                    self.mission_index += 1
                    arrived = False

                if self.mission_index >= len(self.mission_steps): # end of mission
                    # self.mission_mode = False
                    # self.velocity.x = MIN_SPEED
                    # self.velocity.y = MIN_SPEED
                    # self.velocity.z = MIN_SPEED
                    # self.yaw = MIN_SPEED

                    self.mission_index -= 1

            # compute velocity in the world frame
            cos_yaw = np.cos(self.true_yaw)
            sin_yaw = np.sin(self.true_yaw)
            velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
            velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

            # create and publish TrajectorySetpoint message with NaN values for position and acceleration
            trajectory_message = TrajectorySetpoint()
            trajectory_message.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_message.velocity[0] = velocity_world_x
            trajectory_message.velocity[1] = velocity_world_y
            trajectory_message.velocity[2] = self.velocity.z            
            trajectory_message.position[0] = float('nan')
            trajectory_message.position[1] = float('nan')
            trajectory_message.position[2] = float('nan')
            trajectory_message.acceleration[0] = float('nan')
            trajectory_message.acceleration[1] = float('nan')
            trajectory_message.acceleration[2] = float('nan')
            trajectory_message.yaw = float('nan')
            trajectory_message.yawspeed = self.yaw
            trajectory_message.yawspeed = self.yaw

            self.publisher_trajectory.publish(trajectory_message)

    def velocity_to_destiny(self):
        # error_x = self.mission_steps[self.mission_index][0] - self.uav_positions[self.namespace][0]
        # error_y = self.mission_steps[self.mission_index][1] - self.uav_positions[self.namespace][1]

        # destiny_direction = get_destiny_direction(error_x, error_y) if (abs(error_x) > MISSION_TOLERANCE or abs(error_y) > MISSION_TOLERANCE) else self.uav_direction
        # self.set_uav_direction()

        # if destiny_direction != self.uav_direction:
        #     self.velocity.x = MIN_SPEED
        #     self.velocity.y = MIN_SPEED
        #     self.compute_yaw(destiny_direction)
        # else:
        #     self.yaw = MIN_SPEED
        #     self.compute_velocity(error_x, error_y)

        # error_z = self.mission_steps[self.mission_index][2] - self.uav_positions[self.namespace][2]
        # if abs(error_z) > MISSION_TOLERANCE:
        #     self.velocity.z = MAX_SPEED if error_z > 0 else -MAX_SPEED
        # else:
        #     self.velocity.z = MIN_SPEED

        # return abs(error_x) < MISSION_TOLERANCE and abs(error_y) < MISSION_TOLERANCE and abs(error_z) < MISSION_TOLERANCE
    
        error_x = self.mission_steps[self.mission_index][0] - self.odometry.position[0]
        if abs(error_x) > MISSION_TOLERANCE:
            self.velocity.y = MAX_SPEED if error_x > 0 else -MAX_SPEED
        else:
            self.velocity.y = MIN_SPEED

        error_y = self.mission_steps[self.mission_index][1] - self.odometry.position[1]
        if abs(error_y) > MISSION_TOLERANCE:
            self.velocity.x = MAX_SPEED if error_y < 0 else -MAX_SPEED
        else:
            self.velocity.x = MIN_SPEED

        error_z = self.mission_steps[self.mission_index][2] - self.odometry.position[2]
        if abs(error_z) > MISSION_TOLERANCE:
            self.velocity.z = MAX_SPEED if error_z > 0 else -MAX_SPEED
        else:
            self.velocity.z = MIN_SPEED

        return abs(error_x) < MISSION_TOLERANCE and abs(error_y) < MISSION_TOLERANCE and abs(error_z) < MISSION_TOLERANCE
        
    def set_uav_direction(self):
        angle = math.degrees(self.true_yaw + HALF_PI)

        if abs(N - angle) < ANGLE_TOLERANCE:
            self.uav_direction = 'N'
        elif abs(E - angle) < ANGLE_TOLERANCE:
            self.uav_direction = 'E'
        elif abs(S - angle) < ANGLE_TOLERANCE:
            self.uav_direction = 'S'
        elif abs(W - angle) < ANGLE_TOLERANCE:
            self.uav_direction = 'W'
        
    def compute_yaw(self, destiny_direction):
        if((self.uav_direction == 'N' and (destiny_direction == 'E' or destiny_direction == 'S')) or 
           (self.uav_direction == 'E' and (destiny_direction == 'S' or destiny_direction == 'W')) or
           (self.uav_direction == 'S' and (destiny_direction == 'W' or destiny_direction == 'N')) or
           (self.uav_direction == 'W' and (destiny_direction == 'N' or destiny_direction == 'E'))):
            
            self.yaw = MAX_YAW_SPEED
        else:
            self.yaw = -MAX_YAW_SPEED

    def compute_velocity(self, error_x, error_y):
        if self.uav_direction == 'N':
            self.velocity.y = get_velocity(error_x, MAX_SPEED, -MAX_SPEED)
            self.velocity.x = get_velocity(error_y, -MAX_SPEED, MAX_SPEED)
        elif self.uav_direction == 'E':
            self.velocity.x = get_velocity(error_x, MAX_SPEED, -MAX_SPEED)
            self.velocity.y = get_velocity(error_y, MAX_SPEED, -MAX_SPEED)
        elif self.uav_direction == 'S':
            self.velocity.y = get_velocity(error_x, -MAX_SPEED, MAX_SPEED)
            self.velocity.x = get_velocity(error_y, MAX_SPEED, -MAX_SPEED)
        elif self.uav_direction == 'W':
            self.velocity.x = get_velocity(error_x, -MAX_SPEED, MAX_SPEED)
            self.velocity.y = get_velocity(error_y, -MAX_SPEED, MAX_SPEED)


# auxiliar functions


def get_destiny_direction(error_x, error_y):
    if error_y >= abs(error_x) and error_y > 0:
        return 'N'
        
    if abs(error_x) >= abs(error_y) and error_x < 0:
        return 'E'
        
    if abs(error_y) >= abs(error_x) and error_y < 0:
        return 'S'
        
    if error_x >= abs(error_y) and error_x > 0:
        return 'W'


def get_velocity(error, positive_speed, negative_speed):
    if abs(error) > MISSION_TOLERANCE:
        return positive_speed if error > 0 else negative_speed
    else:
        return MIN_SPEED


def correct_velocity(velocity):
    if velocity > MAX_SPEED:
        return MAX_SPEED
    
    if velocity < -MAX_SPEED:
        return -MAX_SPEED
    
    return velocity


def get_steps(text):
    mission_steps = []
    steps = text.split(";")

    for s in steps:
        coordinates = s.split(",")
        coordinates = [float(c) for c in coordinates]

        mission_steps.append(coordinates)

    return mission_steps


def launch_cam_receiver():
  cam = Camera("/camera", (1920,1080))

  while True:
    captured_image = cam.get_next_image()
    cv2.imshow('Unprocessed Image', captured_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    detect_objects(captured_image)


def main(args=None):
    cam_thread = threading.Thread(target=launch_cam_receiver)
    cam_thread.start()

    rclpy.init(args=args)

    mission_mode = True if sys.argv[1] == 't' else False
    mission_steps = get_steps(sys.argv[2])

    offboard_control = OffboardControl(mission_mode, mission_steps)

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()