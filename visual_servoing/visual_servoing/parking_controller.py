#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("Kp_steer", 1.0)
        self.declare_parameter("Kd_steer", 1.0)
        self.declare_parameter("Kd_speed", 1.0)
        self.declare_parameter("Kp_speed", 1.0)
        self.declare_parameter("cone_dt", 0.02)
        self.declare_parameter("parking_distance", 0.75)

        self.DRIVE_TOPIC = self.get_parameter("drive_topic").get_parameter_value().string_value # set in launch file; different for simulator vs racecar
        self.DRIVE_TOPIC = "/drive"    
    
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)
        
        self.Kp_steering = self.get_parameter("Kp_steer").value
        self.Kd_steering = self.get_parameter("Kd_steer").value
        self.Kp_velocity = self.get_parameter("Kp_speed").value
        self.Kd_velocity = self.get_parameter("Kd_speed").value
        self.timestep = self.get_parameter("cone_dt").value
        
        self.parking_distance = self.get_parameter("parking_distance").value # default 0.75 meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.get_logger().info("Parking Controller Initialized")
        
    def angle_2(self, x, y):
        return np.arctan2(y, x) # TODO confirm sign of angle is correct
    
    def distance_2(self, x, y):
        return np.sqrt(x**2 + y**2)

    def parking_PD_control(self, angle, d_angle, distance, d_distance):
        # Steering
        angle_control = self.Kp_steering*angle + self.Kd_steering*d_angle
        
        # Velocity
        dist_error = distance - self.parking_distance
        speed_control = self.Kp_velocity*dist_error + self.Kd_velocity*d_distance
        
        return angle_control, speed_control
    

    def relative_cone_callback(self, msg):
        print("cone callback called")
        prev_angl = self.angle_2(self.relative_x, self.relative_y)
        prev_dist = self.distance_2(self.relative_x, self.relative_y)
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd
        
        angl2cone = self.angle_2(self.relative_x, self.relative_y)
        angl_delta = angl2cone - prev_angl
        dist2cone = self.distance_2(self.relative_x, self.relative_y)
        dist_delta = dist2cone - prev_dist
        print(angl2cone, dist2cone)
        
        steering_angle, forward_speed = self.parking_PD_control(angl2cone, angl_delta/self.timestep,
                                                                dist2cone, dist_delta/self.timestep)
        
        print(steering_angle, forward_speed)
        forward_speed = min(forward_speed, 1.0)
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = "/map" # TODO set correct frame id
        drive_cmd.drive.steering_angle = steering_angle
        drive_cmd.drive.speed = -1*max(0.0, forward_speed)

        #################################
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        x, y = self.relative_x, self.relative_y
        error_msg.x_error = x
        error_msg.y_error = y
        error_msg.distance_error = self.distance_2(x, y)

        #################################
        
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()