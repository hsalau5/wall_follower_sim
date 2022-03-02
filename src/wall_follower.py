#!/usr/bin/env python2

import numpy as np
from scipy import stats

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    MAX_STEERING_ANGLE = rospy.get_param("wall_follower/max_steering_angle")
    WALL_TOPIC = "/wall"
    K_P = SIDE * 2.5
    K_I = rospy.get_param("wall_follower/k_i")
    K_D = rospy.get_param("wall_follower/k_d")

    def __init__(self):
        self.error_history = [0] # Initialize error history to 0

        # Publihser 
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=60)

        # Subscriber # TODO: Set up subscriber + callback function
        rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.drive)

        # Line visualizer
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)

    
    def pid(self, error, k_p, k_i, k_d):
        response = k_p*error + k_i*sum(self.error_history) + k_d*(error - self.error_history[-1]) 
        self.error_history.append(error)

        return response

    def find_angle(self,ranges, angles):
        '''
        Finds the angle of the wall
        '''
        x = np.multiply(ranges, np.cos(angles))
        y = np.multiply(ranges, np.sin(angles))

        m, b, r_value, p_value, std_err = stats.linregress(x,y)

        return m

    def find_position(self, ranges, angles):
        '''
        Determine y(t)
        Finds the position signal in either distance or angle
        '''
        # x = np.multiply(ranges, np.cos(angles))
        # y = np.multiply(ranges, np.sin(angles))

        
        # y_ = np.linspace(0., 5., num=20)
        # x_ = np.multiply(0, y_)

        



        # rospy.loginfo("Side: " + str(self.SIDE))
        # rospy.loginfo("Scan Start Index: " + str(start_index))
        # rospy.loginfo("Scan End Index: " + str(end_index))
        # rospy.loginfo("Scan length: " + str(len(scan.ranges)))
        # rospy.loginfo("Number of measurments" + str((scan.angle_max -  scan.angle_min)/scan.angle_increment))
        # rospy.loginfo("Max range: " + str(scan.range_max))


        #generate a line from the (x,y) coordinates using least squares
        # Approach 1
        # A = np.vstack([x, np.ones(len(x))]).T
        # m, b = np.linalg.lstsq(A, y)[0]

        # Approach 2
        # A = np.concatenate((x, np.ones_like(x)), axis=1)
        # m, b = np.linalg.lstsq(A, y.T[0], rcond = -1)[0]

        # x_ = np.linspace(0., 5., num=20)
        # y_ = m*x_ + b   

        # VisualizationTools.plot_line(x_, y_, self.line_pub, frame="/laser")

        # b = np.abs(b)




        # rospy.loginfo("points: " + str(x_y_points))
        # rospy.loginfo("m: " + str(m))
        # rospy.loginfo("Generating a line with this number of points: " + str(len(x)))

        # Generate error term
        # V1: find angle of line
        # angle = np.arctan(m)
        # debug_angle = angle * 180 / np.pi # convert to degrees
        # position = angle

        # rospy.loginfo("Angle: " + str(debug_angle))


        # V2: find distance between line and origin
        # distance = np.sqrt(m**2 + 1)
        # distance = np.sqrt((b/((m**2) + 1))**2 + (b/(m + (1/m)))**2)
        # position = distance

        # V3: Position is y intercept
        # position = b

        # V4: Position is minimum distance
        if ranges[self.SIDE] <= 2:
            position = ranges[self.SIDE]/2
        else:
            position = min(ranges)        

        position = np.min(ranges)

        # rospy.loginfo("Position: " + str(position))
        

        return position
    
    def find_error(self, output_signal):
        '''
        Determines e(t)
        Finds the error between the output and reference signals
        '''
        
        # V1: Errror w/ angle
        # reference_signal = 1/2 * np.pi

        # V2: Error w/ distance (subtractin)
        reference_signal = self.DESIRED_DISTANCE
        error = reference_signal - output_signal

        # V3: Error w/ distance (ratio)
        # error = reference_singal/output_signal

        #experiment w/ clipping error
        # error = min(1, (reference_singal - output_signal)/0.5)

        return error


    def drive(self, scan):
        '''
        Publishes the calculated drive command
        '''
        # TODO: Dynamically set KP, KI, KD
        self.K_P = rospy.get_param("wall_follower/k_p")
        self.K_I = rospy.get_param("wall_follower/k_i")
        self.K_D = rospy.get_param("wall_follower/k_d")

        rate = rospy.Rate(100) # 20hz #TODO: determine if this is correct

        angle_array = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        ranges = np.array(scan.ranges)



        side_ranges = ranges[np.argwhere( (self.SIDE * angle_array > 0) & (self.SIDE * angle_array < 0.6 * np.pi))]
        side_angles = angle_array[np.argwhere( (self.SIDE * angle_array > 0) & (self.SIDE * angle_array < 0.6 * np.pi))]

        # rospy.loginfo("min angle in angle array: " + str(side_angles[0]))
        # rospy.loginfo("max angle in angle array: " + str(side_angles[-1]))
        # rospy.loginfo("Angle increments " + str(scan.angle_increment))
        # rospy.loginfo(scan.angle_min)
        # rospy.loginfo("Side: " + str(self.SIDE))
        # rospy.loginfo("Scan Range: " + str(scan.angle_min) + " to " + str(scan.angle_max))
        # rospy.loginfo("Theta Start: " + str(theta_start))
        # rospy.loginfo("Theta End: " + str(theta_end))

        position = self.find_position(side_ranges, side_angles)
        error = self.find_error(position)

        steering_offset = self.find_angle(side_ranges, side_angles)
        control_signal = steering_offset + self.pid(error, self.K_P, self.K_I, self.K_D) 

        # rospy.loginfo("Error: " + str(error))
        # rospy.loginfo("Position: " + str(position))


        
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.drive.speed = self.VELOCITY
        ack_msg.drive.acceleration = 0
        ack_msg.drive.jerk = 0
        ack_msg.drive.steering_angle_velocity = 0
        ack_msg.drive.steering_angle = control_signal  
        self.pub.publish(ack_msg)

            

        rospy.loginfo("Steering angle: " + str(ack_msg.drive.steering_angle))
        rospy.loginfo("------------------------------------")

        
        
        

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()


'''
TA Help
1 - How do I dynamically update parameters (KP, KI, KD)
rosparam set k_p 2.0

2 - How do I see debug statements?
* Launch w/ roslaunch wall_follower wall_follower.launch

3 - How do I move the car's starting location?
* Use 2D pose to pull robot to location

4 - Ensure everything is running
Takeaways: Negative angles are on the RIGHT!!!

5 - Use rostopic echo to check messages
* roscore
* roslaunch racecar_simulator simulate.launch
* roslaunch wall_follower wall_follower.launch
'''