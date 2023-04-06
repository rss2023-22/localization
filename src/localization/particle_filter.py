#!/usr/bin/env python2
import numpy as np
import rospy
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped,Pose,PoseArray
import tf.transformations as trans

from threading import Lock

import tf2_ros

class ParticleFilter:
    def __init__(self):

        self.measure_convergence_rate = True
        self.measuring = False
        self.measure_time = None

        self.num_particles = 200
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

        self.probabilities = None

        self.initial_pose = np.array([0,0,0])
        self.particles = np.zeros((200,3)) #np.array([0,0,0])


        self.transform_pub = tf2_ros.TransformBroadcaster()
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.pose_callback, queue_size=1)
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        self.particle_pub = rospy.Publisher("particles", PoseArray, queue_size = 1)

    def calc_avg(self, particles):
        '''
        Take average of the calculated particles returned by the evaluate function of motion or sensor model
        '''

        avg_x = np.average(particles[:,0], weights=self.probabilities, axis = 0)
        avg_y = np.average(particles[:,1], weights=self.probabilities, axis = 0)
        avg_cos = np.average(np.cos(particles[:,2]), weights=self.probabilities, axis=0)
        avg_sin = np.average(np.sin(particles[:,2]), weights=self.probabilities, axis=0)
        theta_pos = np.arctan2(avg_sin,avg_cos)

        return [avg_x, avg_y, theta_pos]
    
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
            self.particles = np.random.multivariate_normal(self.initial_pose,self.initial_cov, size = self.num_particles)

    
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

            # ADDING STUFF TO MEASURE CONVERGENC RATE
            if self.measure_convergence_rate:
                dev1,dev2,dev3 = self.particles.std(axis=0)
                #print([dev1,dev2,dev3])
                # NOTE: up thresholds determined by giving false initialization and examinind stds
                up_threshold_1 = 0.13
                up_threshold_2 = 0.13
                up_threshold_3 = 0.13
                # NOTE: low threshold determined by giving true initilization and examining STDS
                low_threshold = 0.07
                if dev1 > up_threshold_1 or dev2 > up_threshold_2 or dev3 > up_threshold_3:
                    print('began measuring')
                    self.measuring = True
                    self.measure_time = rospy.get_time()
                if dev1 <= low_threshold and dev2 <= low_threshold and dev3 <= low_threshold and self.measuring:
                    print('end measuring')
                    curr = rospy.get_time()
                    self.measuring = False
                    diff = curr - self.measure_time
                    print('Convergence Time = ' + str(diff))
            # END OF CHANGES!



            
            if False: #Publish particles for debugging purposes
                particle_msg = PoseArray()
                particle_msg.header.stamp = rospy.Time.now()
                particle_msg.header.frame_id = '/map'
                
                for p in self.particles:
                    pose = Pose()
                    z,w = np.sin(p[2]/2),np.cos(p[2]/2)
                    pose.position.x = p[0]
                    pose.position.y = p[1]
                    pose.orientation.z = z
                    pose.orientation.w = w
                    
                    particle_msg.poses.append(pose)
                
                self.particle_pub.publish(particle_msg)


    def lidar_callback(self, data):
        '''
        Uses sensor model
        '''
        if self.particles is None: return
        if np.random.rand() > 0.3: return
        with self.particle_lock:

            probs = self.sensor_model.evaluate(self.particles, np.array(data.ranges),11)
            probs /= sum(probs)
            self.probabilities = probs

            particle_resample = self.particles[np.random.choice(self.particles.shape[0], size=self.particles.shape[0], p=probs), :]

            avg = self.calc_avg(particle_resample)
            
            self.publish_pose(avg)
            
            self.particles = particle_resample.copy()

if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
