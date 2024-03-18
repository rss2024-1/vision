#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import String
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
        self.declare_parameter("cone_dt", 0.02) # timestep between published cone locations; value found from lab3 data - TODO confirm for lab4
        self.declare_parameter("pub_ctrl_log", True) # parameter to enable/disable data dump publisher

        self.DRIVE_TOPIC = self.get_parameter("drive_topic").get_parameter_value().string_value # set in launch file; different for simulator vs racecar
        # self.DRIVE_TOPIC = "/drive"    
        
        # DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar
        self.timestep = self.get_parameter("cone_dt").value
        self.pub_ctrl_log = self.get_parameter("pub_ctrl_log").value
        
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)
        self.log_pub = self.create_publisher(String, "/ctrl_data", 10)
        self.param_pub = self.create_publisher(String, "/ctrl_params", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        # PD gains
        self.declare_parameter("Kp_steer", 1.0) # TODO pick better PD gains, confirm signs
        self.declare_parameter("Kd_steer", 1.0)
        self.declare_parameter("Kp_speed", 1.0)
        self.declare_parameter("Kd_speed", 1.0)
        self.Kp_steering = self.get_parameter("Kp_steer").value
        self.Kd_steering = self.get_parameter("Kd_steer").value
        self.Kp_velocity = self.get_parameter("Kp_speed").value
        self.Kd_velocity = self.get_parameter("Kd_speed").value
        
        # Max/min control values
        self.declare_parameter("angle_max", 2.0) # maximum steering angle (left? right?)
        self.declare_parameter("angle_min", -2.0) # minimum steering angle
        self.declare_parameter("speed_max", 1.0) # maximum forward speed
        self.declare_parameter("speed_min", -0.1) # minimum forward speed
        self.max_steer = self.get_parameter("angle_max").value
        self.min_steer = self.get_parameter("angle_min").value
        self.max_speed = self.get_parameter("speed_max").value
        self.min_speed = self.get_parameter("speed_min").value
        
        # Acceptable error values
        self.declare_parameter("dist_error", 0.01) # acceptable distance error threshold
        self.declare_parameter("angle_error", 0.0) # acceptable angle error threshold
        self.dist_error = self.get_parameter("dist_error").value
        self.angle_error = self.get_parameter("angle_error").value
        
        # Reversing/reparking
        self.reverse_state = False # track whether car is currently backing up
        self.declare_parameter("steer_rev", -0.5) # multiplier for steering commands while reversing
        self.rev_steer = self.get_parameter("steer_rev").value
        self.parking_retry = False # track special state in which car is backing up to try parking again
        self.park_fail_count = 0
        self.declare_parameter("fail_count", 50) # number of "failed" parking values to trigger a retry
        self.declare_parameter("retry_dist", 0.5) # amount to back up when doing a parking retry
        self.fail_count_max = self.get_parameter("fail_count").value
        self.park_retry_dist = self.get_parameter("retry_dist").value
        
        # Goal parking distance, current coordinates
        self.declare_parameter("parking_distance", 0.75) # goal distance away from the cone to park 
        self.parking_distance = self.get_parameter("parking_distance").value # default 0.75 meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        
        self.last_log = 0
        self.get_logger().info("Parking Controller Initialized")
        
    def angle_2(self, x, y):
        return np.arctan2(y, x) # TODO confirm sign of angle is correct
    
    def distance_2(self, x, y):
        return np.sqrt(x**2 + y**2)

    def speed_PD(self, distance, d_distance):
        return self.Kp_velocity*distance + self.Kd_velocity*d_distance

    def steer_PD(self, angle, d_angle):
        if self.reverse_state: return self.rev_steer * (self.Kp_steering*angle + self.Kd_velocity*d_angle)
        else: return self.Kp_steering*angle + self.Kd_steering*d_angle

    def relative_cone_callback(self, msg):

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        prev_angl = self.angle_2(self.relative_x, self.relative_y)
        prev_dist = self.distance_2(self.relative_x, self.relative_y) - self.parking_distance
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        
        # Calculate error        
        angl2cone = self.angle_2(self.relative_x, self.relative_y)
        angl_delta = angl2cone - prev_angl
        dist2cone = self.distance_2(self.relative_x, self.relative_y) - self.parking_distance
        # dist2cone = self.distance_2(self.relative_x, self.relative_x) - self.parking_distance # actually distance to target point, not exactly cone
        dist_delta = dist2cone - prev_dist
        # print(angl2cone, dist2cone)
        
        # Check if parking retry conditions met repeatedly
        if abs(dist2cone) < self.dist_error and abs(angl2cone) > self.angle_error:
            self.park_fail_count += 1
            if self.park_fail_count > self.fail_count_max:
                self.parking_retry = True
                self.park_fail_count = 0
        
        elif self.park_fail_count > 0 and not self.parking_retry:
            self.park_fail_count -= 1
        
        # Calculate appropriate speed
        speed = np.clip(self.speed_PD(dist2cone, dist_delta/self.timestep), self.min_speed, self.max_speed)
        if abs(speed) < self.dist_error: speed = 0.0
        
        # Logic for parking retry state
        if self.parking_retry == True:
            speed = self.min_speed
            if dist2cone >= self.park_retry_dist:
                self.parking_retry = False
        
        # Check if car is backing up
        self.reverse_state = (speed < 0.0)
        
        # Calculate appropriate steering angle
        steer = np.clip(self.steer_PD(angl2cone, angl_delta/self.timestep), self.min_steer, self.max_steer)
        
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = "/map" # TODO set correct frame id
        drive_cmd.drive.steering_angle = steer
        drive_cmd.drive.speed = float(speed)
        
        if self.pub_ctrl_log:
            ctrl_log = String() # desired paramaters for loging: Angle to cone, 
            time_sec = drive_cmd.header.stamp.sec
            time_nanosec = drive_cmd.header.stamp.nanosec
            ctrl_log.data = (str(time_sec)+"."+str(time_nanosec) # seconds with decimal appended
                            + "," + str(round(angl2cone, 5))
                            + "," + str(round(angl_delta, 5))
                            + "," + str(round(dist2cone, 5))
                            + "," + str(round(dist_delta, 5))
                            + "," + str(round(steer, 5))
                            + "," + str(round(speed, 5))
                            + "," + str(self.reverse_state)
                            + "," + str(self.parking_retry)
                            + "," + str(self.park_fail_count)
                            )
            self.log_pub.publish(ctrl_log)
            if (time_sec - self.last_log) > 5: # publish parameters every ~3 seconds
                self.last_log = time_sec
                param_log = String()
                param_log.data = ("tStep:" + str(self.timestep)
                                  + "," + "tgtDist:" + str(self.parking_distance)
                                  + "," + "stKp:" + str(self.Kp_steering)
                                  + "," + "stKd:" + str(self.Kd_steering)
                                  + "," + "spKp:" + str(self.Kp_velocity)
                                  + "," + "spKd:" + str(self.Kd_velocity)
                                  + "," + "maxAng:" + str(self.max_steer)
                                  + "," + "minAng:" + str(self.min_steer)
                                  + "," + "maxSp:" + str(self.max_speed)
                                  + "," + "minSp:" + str(self.min_speed)
                                  + "," + "distErr:" + str(self.dist_error)
                                  + "," + "anglErr:" + str(self.angle_error)
                                  + "," + "rvSteer:" + str(self.rev_steer)
                                  + "," + "failCnt:" + str(self.fail_count_max)
                                  + "," + "tryDist:" + str(self.park_retry_dist)
                                  )
                self.param_pub.publish(param_log)
        
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