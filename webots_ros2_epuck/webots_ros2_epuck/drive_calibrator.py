# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# This node helps you to calibrate wheel radius and distance between the wheels
# by moving the robot forward and correcting wheel radius, and rotating robot and
# correcting distance between the wheels.
# ros2 run webots_ros2_epuck drive_calibrator --ros-args -p type:=linear -p wheel_radius:=0.021

from enum import Enum
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg._parameter import Parameter
from rclpy.parameter import ParameterType, ParameterValue
from geometry_msgs.msg import Twist
from webots_ros2_core.math_utils import quaternion_to_euler


# Target distance for robot to pass in meters
DEFAULT_DISTANCE = 0.1335
# Default separation between two wheels (from e-puck website)
DEFAULT_WHEEL_DISTANCE = 0.0552
# Default wheel radius (from e-puck website)
DEFAULT_WHEEL_RADIUS = 0.021


RANGE_N_MEASUREMENTS = 15
RADIUS_STEP_SIZE = 0.02
LINEAR_VELOCITY = 0.02


class State(Enum):
    LINEAR_MOVE = 1
    ANGULAR_MOVE = 2
    MEASURE_RANGE = 3
    ANGULAR_FIND_FEATURE = 4


class EPuckDriveCalibrator(Node):
    def __init__(self, name, args=None):
        super().__init__(name)

        self.test_done = False
        self.rotation_count = 0
        self.last_yaw = 0

        # Parameters
        self.type_param = self.declare_parameter('type', 'rotation')
        self.distance = self.declare_parameter('distance', DEFAULT_DISTANCE)
        self.wheel_distance_param = self.declare_parameter('wheel_distance', DEFAULT_WHEEL_DISTANCE)
        self.wheel_radius_param = self.declare_parameter('wheel_radius', DEFAULT_WHEEL_RADIUS)

        # Topics
        self.create_subscription(Odometry, '/odom', self.odometry_callback, 1)
        self.create_subscription(Range, '/tof', self.distance_callback, 1)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameter service
        self.cli = self.create_client(SetParameters, 'epuck/set_parameters')
        self.cli.wait_for_service(timeout_sec=1.0)
        self.set_velocity(0, 0)
        self.set_param('wheel_distance', self.wheel_distance_param.value)
        self.set_param('wheel_radius', self.wheel_radius_param.value)
        self.get_logger().info('Setting wheel distance to: {}m'.format(self.wheel_distance_param.value))
        self.get_logger().info('Setting wheel radius to: {}m'.format(self.wheel_radius_param.value))

        self.state = State.MEASURE_RANGE
        self.range_sum = 0
        self.range_avg = 0
        self.range_n_measurements = 0
        self.range_start = 0
        self.range_initial = True
        self.range_prev = 0
        self.move_distance = 0
        self.move_direction = 1
        self.odom_last_linear = 0
        self.odom_last_angular = 0
        self.odom_prev_linear = 0
        self.wheel_radius = self.wheel_radius_param.value

    def set_param(self, name, value):
        req = SetParameters.Request()
        param_value = ParameterValue(double_value=value, type=ParameterType.PARAMETER_DOUBLE)
        param = Parameter(name=name, value=param_value)
        req.parameters.append(param)
        self.cli.call_async(req)

    def set_velocity(self, linear, angular):
        msg = Twist()
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular)
        msg.linear.x = float(linear)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        self.pub.publish(msg)

    def distance_callback(self, msg: Range):
        # MEASURE_RANGE: Precise distance measurement
        if self.state == State.MEASURE_RANGE:
            if self.range_n_measurements < RANGE_N_MEASUREMENTS:
                self.range_sum += msg.range
                self.range_n_measurements += 1
            else:
                self.range_avg = self.range_sum / RANGE_N_MEASUREMENTS

                # Update wheel radius
                if not self.range_initial:
                    range_diff = abs(self.range_start - self.range_avg)
                    estimated_error = abs(self.odom_last_linear - self.odom_prev_linear) - range_diff
                    new_wheel_radius = self.wheel_radius + estimated_error * RADIUS_STEP_SIZE
                    self.get_logger().info('Updating wheel radius from {} to {}'.format(
                        self.wheel_radius,
                        new_wheel_radius
                    ))
                    self.set_param('wheel_radius', new_wheel_radius)
                    self.wheel_radius = new_wheel_radius
                self.range_initial = False
                self.odom_prev_linear = self.odom_last_linear

                # Go to next state
                self.move_direction = 1 if self.range_avg > self.distance.value else -1
                self.range_start = self.range_avg
                self.state = State.LINEAR_MOVE

        # LINEAR_MOVE: Move linearly
        if self.state == State.LINEAR_MOVE:
            if abs(msg.range - self.range_start) < self.distance.value:
                self.set_velocity(LINEAR_VELOCITY * self.move_direction, 0)
            else:
                self.set_velocity(0, 0)
                self.range_sum = 0
                self.range_n_measurements = 0
                self.state = State.MEASURE_RANGE


    def odometry_callback(self, msg: Odometry):
        yaw, _, _ = quaternion_to_euler(msg.pose.pose.orientation)
        self.odom_last_linear = msg.pose.pose.position.x
        self.odom_last_angular = yaw


def main(args=None):
    rclpy.init(args=args)
    epuck_controller = EPuckDriveCalibrator('epuck_drive_calibrator', args=args)
    rclpy.spin(epuck_controller)
    epuck_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
