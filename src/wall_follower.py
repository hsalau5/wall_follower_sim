#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    MAX_STEERING = rospy.get_param("wall_follower/max_steering")
    K_P = rospy.get_param("wall_follower/k_p")
    K_I = rospy.get_param("wall_follower/k_i")
    K_D = rospy.get_param("wall_follower/k_d")

    def __init__(self):
        # TODO:
        self.error_history = list()
        # Initialize your publishers and
        # subscribers here

        # Publihser 
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped) 


        # Subscriber # TODO: Set up subscriber + callback function
        rospy.init_node('simple_subscriber', anonymous=True)
        rospy.Subscriber("my_random_float", self.SCAN_TOPIC, self.drive)
    
    def pid(self, error, k_p, k_i, k_d):
        response = k_p*error + k_i*sum(self.error_history) + k_d*(error - self.error_history[-1]) 
        self.error_history.append(error)

        return response

    def find_position(self, scan, theta_start, theta_end):
        '''
        Determine y(t)
        Finds the position signal in either distance or angle
        '''
        # know where to start and end range
        start_index = int(theta_start - scan.angle_min /scan.angle_increment) # TODO: implement theta_start - scan.angle_min
        end_index = int(theta_end - scan.angle_min /scan.angle_increment) # TODO: implement theta_end - scan.angle_min

        # slice the scan data to get the range at theta_start and theta_end
        ranges = scan.ranges[start_index:end_index]

        #get (x,y) coordinates for each position
        x_y_points = list()
        for i in range(len(ranges)):
            x_y_points.append( (ranges[i]*np.cos(i*scan.angle_increment + scan.angle_min), ranges[i]*np.sin(i*scan.angle_increment + scan.angle_min)) )

        #generate a line from the (x,y) coordinates using least squares
        x = [x_y_points[i][0] for i in range(len(x_y_points))]
        y = [x_y_points[i][1] for i in range(len(x_y_points))]
        A = np.vstack([x, np.ones(len(x))]).T
        m, b = np.linalg.lstsq(A, y)[0]

        # Generate error term
        # V1: find angle of line
        # angle = np.arctan(m)
        # position = angle

        # V2: find distance between line and origin
        distance = np.sqrt(m**2 + 1)
        position = distance

        return position
    
    def find_error(self, output_signal):
        '''
        Determines e(t)
        Finds the error between the output and reference signals
        '''
        
        # V1: Errror w/ angle
        # reference_signal = 1/2 * np.pi

        # V2: Error w/ distance
        reference_singal = self.DESIRED_DISTANCE

        error = reference_singal - output_signal
        return error

    def calculate_control_signal(self, error, k_p, k_i, k_d):
        '''
        Determines u(t)
        Calculates the control signal for the wall follower
        '''
        response = self.pid(error, k_p, k_i, k_d)

        # TODO: Determine if my sides are correct
        return response * self.SIDE


    def drive(self, scan):
        '''
        Publishes the calculated drive command
        '''
        # TODO: Set KP, KI, KD, error pull

        if self.SIDE < 0: # right
            theta_start = (scan.angle_min + scan.angle_max)/2
            thete_end = scan.angle_max
        else: # left
            theta_start = scan.angle_min
            theta_end = (scan.angle_min + scan.angle_max)/2

        position = self.find_position(scan, theta_start, theta_end)
        error = self.find_error(position)
        control_signal = self.calculate_control_signal(error, self.K_P, self.K_I, self.K_D)
        
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = "base_link" # TODO: determine if this is correct
        ack_msg.drive.steering_angle = control_signal * self.MAX_STEERING
        ack_msg.drive.speed = self.VELOCITY

        
        self.pub.publish(ack_msg)







        

    
    # TODO:
    # Write your callback functions here.

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
