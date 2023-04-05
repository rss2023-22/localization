#!/usr/bin/env python2
import numpy as np
import rospy
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped
import tf.transformations as trans

from threading import Lock

import tf2_ros

class ParticleFilter:
    def __init__(self):
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()
        self.last_odom = None
        self.last_odom_time = rospy.get_time()
        # Establish thread locking for the two callbacks updating the particle list
        self.particle_lock = Lock()
    
        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")


        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")

        self.initial_pose = np.array([0,0,0])
        self.particles = np.zeros((200,3)) #np.array([0,0,0])


        self.transform_pub = tf2_ros.TransformBroadcaster()
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.pose_callback, queue_size=1)
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)

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
        odom_msg.header.stamp = rospy.Time.now()
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
        pose_transform.header.frame_id = '/map'
        pose_transform.child_frame_id = self.particle_filter_frame # 'base_link_pf' for the simulator, 'base_link' for the car
        pose_transform.transform.translation.x = avg[0]
        pose_transform.transform.translation.y = avg[1]
        pose_transform.transform.translation.z = 0
        pose_transform.transform.rotation.x = quat[0] #0
        pose_transform.transform.rotation.y = quat[1] #0
        pose_transform.transform.rotation.z = z #z
        pose_transform.transform.rotation.w = w #w
        self.transform_pub.sendTransform(pose_transform)
    
    def odom_callback(self,data):
        '''
        Uses motion model
        '''
        if self.particles is None: return
        
        with self.particle_lock:

            now = rospy.get_time()
            now_odom = np.array([data.twist.twist.linear.x,
                                 data.twist.twist.linear.y,
                                 data.twist.twist.angular.z])
            
            if self.last_odom is None: odom = now_odom*(now-self.last_odom_time)
            else: odom = (now_odom+self.last_odom)*(now-self.last_odom_time)/2
            
            self.last_odom = now_odom
            self.last_odom_time = now
            
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

            probs = self.sensor_model.evaluate(self.particles, np.array(data.ranges),1)
            probs /= sum(probs)

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