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
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan,
                                          lidar_callback(), # TODO: Fill this in
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry, odom_callback(), # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          self.odom_sub.pose, # TODO: Fill this in
                                          queue_size=1)
        # need to check with rviz? no callback function bc self.odom_sub is a geometry message and we just want the pose

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        
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
        # Find a way to initialize the particles on rviz

        # Callback functions
        def odom_callback(odometry):
            '''
            Uses motion model
            '''
            odom = np.array(self.odom_sub.twist.twist.linear.x, self.odom_sub.self.odom_sub.twist.twist.linear.y, self.odom_sub.self.odom_sub.twist.twist.angular.z)
            particles = self.motion_model.evaluate(self.pose_sub, odom)
            # take average of the calculated particles returned by the evaluate function
            N = particles.shape[0]
            for i in range(N):
                x_pos += particles[i,1]
                y_pos += particles[i,2]
                cos,sin += np.cos(particles[i,3]), np.sin(particles[i,3])
            avg_x = x_pos/N
            avg_y = y_pos/N
            avg_cos,avg_sin = cos*1/N,sin*1/N
            theta_pos = np.arctan(avg_sin/avg_cos)
            self.pose_sub = [avg_x, avg_y, theta_pos]
            self.odom_pub.publish(self.pose_sub)

        def lidar_callback():

            probs = self.sensor_model.evaluate(self.pose_sub, self.laser_sub)
            # resample the particles based on these probabilities
            # Use np.random.choice?? Check on this
            N = probs.shape[0]
            np.random.choice(1,N)


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
