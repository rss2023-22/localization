#!/usr/bin/env python2
import numpy as np
import rospy
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped,PoseArray,Pose
import tf.transformations as trans

from threading import Lock

import tf2_ros

class ParticleFilter:
    def __init__(self):
        self.rate = 20
        rospy.Rate(self.rate)


        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")
        self.num_particles = rospy.get_param("~num_particles")

        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")

        self.last_odom = None
        self.last_odom_time = rospy.get_time()
        # Establish thread locking for the two callbacks updating the particle list
        self.particle_lock = Lock()
    
        # Get parameters
    

        # Initialize publishers/subscribers
        #
        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from thecd 
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        

        self.initial_pose = np.array([0,0,0])
        self.particles = None #np.array([0,0,0])

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.pose_callback, queue_size=1)
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        self.points_pub = rospy.Publisher("/pf/points", PoseArray, queue_size=1)
        self.transform_pub = tf2_ros.TransformBroadcaster()
        # Change this because in the readme, it says to publish to base_link_pf
        # self.odom_pub  = rospy.Publisher("/base_link_pf", Odometry, queue_size = 1)

        # Initialize the models
        

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
        x_pos,y_pos,cos,sin = 0,0,0,0
        for i in range(N):
            x_pos += particles[i,0]
            y_pos += particles[i,1]
            cos += np.cos(particles[i,2])
            sin += np.sin(particles[i,2])
        avg_x = x_pos/N
        avg_y = y_pos/N
        avg_cos = cos/N
        avg_sin = sin/N
        theta_pos = np.arctan2(avg_sin,avg_cos)
        avg_pose = [avg_x, avg_y, theta_pos]
        #rospy.loginfo(avg_pose)
        return avg_pose
    
    # Callback functions
    def pose_callback(self,data):
        '''
        Gets initial pose from rviz data
        Remember to add posewithcovariance topic on rviz
        '''
        with self.particle_lock:
            self.initial_pose = np.array([data.pose.pose.position.x,
                                          data.pose.pose.position.y,
                                          2*np.arctan2(data.pose.pose.orientation.z,data.pose.pose.orientation.w)])
            self.initial_cov = np.array([[data.pose.covariance[0],data.pose.covariance[1],data.pose.covariance[5]],
                                         [data.pose.covariance[6],data.pose.covariance[7],data.pose.covariance[11]],
                                         [data.pose.covariance[30],data.pose.covariance[31],data.pose.covariance[35]]])
            self.particles = np.random.multivariate_normal(self.initial_pose,self.initial_cov, size = 512)
    
    def publish_pose(self,pose):
        avg = pose
        z,w = np.sin(avg[2]/2),np.cos(avg[2]/2)
        quat = trans.quaternion_about_axis(avg[2],(0,0,1))
        
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time()
        odom_msg.header.frame_id = 'map'
        odom_msg.pose.pose.position.x = avg[0]
        odom_msg.pose.pose.position.y = avg[1]
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = z
        odom_msg.pose.pose.orientation.w = w
        
        self.odom_pub.publish(odom_msg)

        pose_transform = TransformStamped()
        pose_transform.header.stamp = rospy.Time.now()
        pose_transform.header.frame_id = 'map'
        pose_transform.child_frame_id = 'base_link_pf'
        pose_transform.transform.translation.x = avg[0]
        pose_transform.transform.translation.y = avg[1]
        pose_transform.transform.translation.z = 0
        pose_transform.transform.rotation.x = quat[0] #0
        pose_transform.transform.rotation.y = quat[1] #0
        pose_transform.transform.rotation.z = z #z
        pose_transform.transform.rotation.w = w #w
        self.transform_pub.sendTransform(pose_transform)
        #tf2_ros.TransformBroadcaster().sendTransform(pose_transform)
    
    def odom_callback(self,data):
        '''
        Uses motion model
        '''
        if self.particles is None: return
        
        with self.particle_lock:
            #rospy.loginfo('odom callback')
            # get odometry data
            now = rospy.get_time()
            now_odom = np.array([data.twist.twist.linear.x,data.twist.twist.linear.y,data.twist.twist.angular.z])
            
            if self.last_odom is None: odom = now_odom*(now-self.last_odom_time)
            else: odom = (now_odom+self.last_odom)*(now-self.last_odom_time)/2
            
            self.last_odom = now_odom
            self.last_odom_time = now
            
            # update particle positions from initial pose
            # rospy.loginfo(self.initial_pose)
            self.updated_particles = self.motion_model.evaluate(self.particles, odom)
            avg = self.calc_avg(self.updated_particles)
            
            self.publish_pose(avg)
            
            self.particles = self.updated_particles.copy()

    def lidar_callback(self, data):
        '''
        Uses sensor model
        '''
        if self.particles is None: return
        if np.random.rand() > 0.3: return
        with self.particle_lock:
            #rospy.loginfo('lidar callback')
            #odom = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.angular.z])
            # particles = self.motion_model.evaluate(self.updated_particles, odom)
            # calculate probabilities given initial pose and lidar data
            probs = self.sensor_model.evaluate(self.particles, np.array(data.ranges),5)
            probs /= sum(probs)
            #rospy.loginfo(probs)
            # do not use motion model here, use the current particle positions

            #particles = np.array(self.initial_pose)
            # resample the particles based on these probabilities - use np.random.choice

            # compute random sample of new particles
            particle_resample = np.zeros(self.particles.shape)
            sample_indices = np.random.choice(self.particles.shape[0], size=self.particles.shape[0], p=probs)
            for i in range(self.particles.shape[0]):
                particle_resample[i,:] = self.particles[sample_indices[i],:]
            avg = self.calc_avg(particle_resample)
            
            self.publish_pose(avg)
            
            self.particles = particle_resample.copy()

if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
