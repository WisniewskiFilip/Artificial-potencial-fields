# -*- coding: utf-8 -*-

# !/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from evry_project_plugins.srv import DistanceToFlag

import math


class Robot:
    def __init__(self, robot_name):
        """Constructor of the class Robot
        The required publishers / subscribers are created.
        The attributes of the class are initialized

        Args:
            robot_name (str): Name of the robot, like robot_1, robot_2 etc. To be used for your subscriber and publisher with the robot itself
        """
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0  # Sonar distance
        self.x, self.y = 0.0, 0.0  # coordinates of the robot
        self.yaw = 0.0  # yaw angle of the robot
        self.robot_name = robot_name

        '''Listener and publisher'''

        rospy.Subscriber(self.robot_name + "/sensor/sonar_front",
                         Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom",
                         Odometry, self.callbackPose)
        self.cmd_vel_pub = rospy.Publisher(
            self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Range): Message that contains the distance separating the US sensor from a potential obstacle
        """
        self.sonar = msg.range

    def get_sonar(self):
        """Method that returns the distance separating the ultrasonic sensor from a potential obstacle
        """
        return self.sonar

    def callbackPose(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Odometry): Message that contains the coordinates of the agent
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x,
                           quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion_list)
        self.yaw = yaw

    def get_robot_pose(self):
        """Method that returns the position and orientation of the robot"""
        return self.x, self.y, self.yaw

    def constraint(self, val, min=-2.0, max=2.0):
        """Method that limits the linear and angular velocities sent to the robot

        Args:
            val (float): [Desired velocity to send
            min (float, optional): Minimum velocity accepted. Defaults to -2.0.
            max (float, optional): Maximum velocity accepted. Defaults to 2.0.

        Returns:
            float: Limited velocity whose value is within the range [min; max]
        """
        # DO NOT TOUCH
        if val < min:
            return min
        if val > max:
            return max
        return val

    def set_speed_angle(self, linear, angular):
        """Method that publishes the proper linear and angular velocities commands on the related topic to move the robot

        Args:
            linear (float): desired linear velocity
            angular (float): desired angular velocity
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear)
        cmd_vel.angular.z = self.constraint(angular, min=-1, max=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        """Get the distance separating the agent from a flag. The service 'distanceToFlag' is called for this purpose.
        The current position of the robot and its id should be specified. The id of the robot corresponds to the id of the flag it should reach


        Returns:
            float: the distance separating the robot from the flag
        """
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            # int(robot_name[-1]) corresponds to the id of the robot. It is also the id of the related flag
            result = service(pose, int(self.robot_name[-1]))
            return result.distance
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def calculate_pid(self, error, integral, derivative, kp, ki, kd):
        return kp * error + ki * integral + kd * derivative

        # Attractive potential field strategy functions


def calculate_apf(self, goal_pose, obstacle_positions):
    attractive_force = self.calculate_attractive_force(goal_pose)
    repulsive_force = self.calculate_repulsive_force(obstacle_positions)

    total_force = attractive_force + repulsive_force
    return total_force

 def calculate_attractive_force(self, goal_pose):
        k_att = 1.0  # Attractive force constant

        delta_x = goal_pose.x - self.x
        delta_y = goal_pose.y - self.y
        distance = math.sqrt(delta_x**2 + delta_y**2)

        # Check if the distance is zero to avoid division by zero
        if distance == 0:
            attractive_force_x = 0.0
            attractive_force_y = 0.0
        else:
            attractive_force_x = k_att * delta_x / distance
            attractive_force_y = k_att * delta_y / distance

        return Pose2D(attractive_force_x, attractive_force_y, 0.0)


def calculate_repulsive_force(self, obstacle_positions):
    k_rep = 1.0  # Repulsive force constant
    min_distance = 1.0  # Minimum distance to consider obstacles

    repulsive_force_x = 0.0
    repulsive_force_y = 0.0

    for obstacle_pose in obstacle_positions:
        delta_x = self.x - obstacle_pose.x
        delta_y = self.y - obstacle_pose.y
        distance = math.sqrt(delta_x ** 2 + delta_y ** 2)

        # Check if the distance is zero to avoid division by zero
        if distance != 0:
        repulsive_force_x += k_rep * (1.0 / distance - 1.0 / min_distance) * delta_x / distance ** 2
        repulsive_force_y += k_rep * (1.0 / distance - 1.0 / min_distance) * delta_y / distance ** 2

    return Pose2D(repulsive_force_x, repulsive_force_y, 0.0)


def run_demo():
    """Main loop"""
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print(f"Robot : {robot_name} is starting..")

    # Timing
    if robot_name == "robot_1":
        rospy.sleep(rospy.Duration(0))  # Robot 1 starts immediately
    elif robot_name == "robot_2":
        rospy.sleep(rospy.Duration(5))  # Robot 2 starts after 5 seconds
    elif robot_name == "robot_3":
        rospy.sleep(rospy.Duration(10))  # Robot 3 starts after 10 seconds

    # rate = rospy.Rate(1)  # 1 Hz

    # PID controller parameters
    kp = 0.5  # Proportional gain
    ki = 0.1  # Integral gain
    kd = 0.01  # Derivative gain
    integral = 0.0
    prev_error = 0.0

    flag_positions = {
        1: Pose2D(x=-21.213203, y=21.213203, theta=0.0),
        2: Pose2D(x=21.213203, y=21.213203, theta=0.0),
        3: Pose2D(x=0, y=-30, theta=0.0),
        # Add more entries for additional flags as needed
    }

    while not rospy.is_shutdown():
        robot_id = int(robot.robot_name[-1])

        # Get the flag's position as the goal
        # Get the position of the assigned flag using the robot's ID
        goal_pose = flag_positions.get(robot_id, Pose2D())  # Default to an empty Pose2D if ID not found

        # Get obstacle positions (modify as needed)
        obstacle_positions = [Pose2D(2.0, 2.0, 0.0), Pose2D(-1.0, 0.0, 0.0)]  # Example obstacle positions

        # APF strategy
        apf_force = robot.calculate_apf(goal_pose, obstacle_positions)

        velocity = robot.constraint(apf_force.x, min=0.0, max=2.0)
        angle = math.atan2(apf_force.y, apf_force.x)
        # Strategy
        # velocity = 1
        # angle = 0.0

        target_distance = 10  # ideal distance to maintain from the flag

        distance = float(robot.getDistanceToFlag())
        print(f"Robot {robot.robot_name} distance to flag = {distance}")

        # PID controller calculations
        error = distance - target_distance
        integral = integral + error
        derivative = error - prev_error

        # Calculate PID controller output
        pid_output = robot.calculate_pid(error, integral, derivative, kp, ki, kd)

        # Update previous error for the next iteration
        prev_error = error

        # saturation
        if distance > 10:
            velocity = robot.constraint(pid_output, min=0.0, max=2.0)
        elif distance > 1:
            # Gradually slow down as the distance decreases
            velocity = robot.constraint(pid_output, min=0.0, max=2.0) * (distance - 1) / 9.0
        else:
            # Stop when the distance is less than or equal to 1
            velocity = 0

        # Finishing by publishing the desired speed
        robot.set_speed_angle(velocity, angle)

        print('velocity: %f' % velocity)

        # Write here your strategy..

        if distance > 1.0
        velocity = 1
        else:
        velocity = 0

        # Finishing by publishing the desired speed.
        # DO NOT TOUCH.
        robot.set_speed_angle(velocity, angle)
        print(f"Robot {robot.robot_name} velocity: {velocity}, angle: {angle}")
        rospy.sleep(0.5)


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous=True)
    run_demo()