#!/usr/bin/env python2
import numpy as np
import rospy
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class ParticleFilter:
    def __init__(self):
        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")

        # Initialize publishers/subscribers
        #
        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from thecd 
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")

        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.lidar_callback, queue_size=1)

        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          self.pose_callback, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        # Change this because in the readme, it says to publish to base_link_pf
        # self.odom_pub  = rospy.Publisher("/base_link_pf", Odometry, queue_size = 1)

        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.
 
        ########## DINURI'S CODE STARTS HERE ############
    def calc_avg(self, particles):
        '''
        Take average of the calculated particles returned by the evaluate function of motion or sensor model
        '''
        N = np.array(particles).shape[0]
        for i in range(N):
            x_pos += particles[i,1]
            y_pos += particles[i,2]
            cos += np.cos(particles[i,3])
            sin += np.sin(particles[i,3])
        avg_x = x_pos/N
        avg_y = y_pos/N
        avg_cos = cos/N
        avg_sin = sin/N
        theta_pos = np.arctan(avg_sin/avg_cos)
        avg_pose = [avg_x, avg_y, theta_pos]
        return avg_pose
    
    # Callback functions
    def pose_callback(self,data):
        '''
        Gets initial pose from rviz data
        Remember to add posewithcovariance topic on rviz
        '''
        initial_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.orientation.z])
    
    def odom_callback(self,data):
        '''
        Uses motion model
        '''
        # get odometry data
        odom = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.angular.z])
        # update particle positions from initial pose
        updated_particles = self.motion_model.evaluate(np.array(self.pose_sub), odom)
        avg = self.calc_avg(updated_particles)
        self.odom_pub.publish(avg)

    def lidar_callback(self, data):
        '''
        Uses sensor model
        '''
        # calculate probabilities given initial pose and lidar data
        probs = self.sensor_model.evaluate(np.array(self.pose_sub), data.ranges)
        # do not use motion model here, use the current particle positions
        # odom = np.array(self.odom_sub.twist.twist.linear.x, self.odom_sub.self.odom_sub.twist.twist.linear.y, self.odom_sub.self.odom_sub.twist.twist.angular.z)
        # particles = self.motion_model.evaluate(self.pose_sub, odom)
        particles = np.array(self.pose_sub)
        # resample the particles based on these probabilities - use np.random.choice
        N = probs.shape[0]
        resam_choices = np.array()
        resam_choices_probs = np.array()
        for i in probs:
            # only include particles that have over 50% chance of being there
            if i > 0.5:
                np.append(resam_choices, particles[i,:])
                np.append(resam_choices_probs, probs[i,:])
        # given choices of high probability particles, compute random sample of new particles
        particle_resample = np.random.choice(resam_choices, p=resam_choices_probs)
        avg = self.calc_avg(particle_resample)
        self.odom_pub.publish(avg)


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()