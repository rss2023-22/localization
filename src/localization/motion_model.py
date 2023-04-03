import numpy as np
import rospy

class MotionModel:

    def __init__(self):
    
        self.DETERMINISTIC = rospy.get_param(rospy.search_param('deterministic'))

        #Constants for uncertainty - 1,2 are rotational, 3,4 are translational
        self.alpha = {1: 0.1,
                      2: 0.1,
                      3: 0.05,
                      4: 0.05} #Arbitrary values, no idea if they make sense

    def eps_b(self,b,n=2):
        '''
        Sample a single value from a distribution with mean zero and variance b
        
        args:
            b: A float, the overall variance of the distribution
            n: An integer for how many uniform samples to average
                Defaults to 2 (triangular)
        
        returns:
            A randomly-generated float
        '''
        c = np.sqrt(abs(3*b/n))
        return sum(np.random.rand(n))*2*c-c*n

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """

        N = particles.shape[0]
        for i in range(N):
            # Rotate the position components of the odometry to the world frame
            cos,sin = np.cos(particles[i,2]),np.sin(particles[i,2])
            rotated_displacement = (cos*odometry[0]-sin*odometry[1],
                                    sin*odometry[0]+cos*odometry[1])
            
            if self.DETERMINISTIC:
                # Add the displacements to the pose
                particles[i,0] += rotated_displacement[0]
                particles[i,1] += rotated_displacement[1]
                particles[i,2] += odometry[2]
            else:
                # Adapted from Probabilistic Robotics, sample_motion_model_odometry
                
                # Decompose motion into three components
                delta_rot_1 = np.arctan2(rotated_displacement[1],
                                         rotated_displacement[0])-particles[i,2]
                delta_trans = (odometry[0]**2+odometry[1]**2)**0.5
                delta_rot_2 = odometry[2]-delta_rot_1
                
                # Add noise
                delta_rot_1 -= self.eps_b(self.alpha[1]*delta_rot_1+\
                                          self.alpha[2]*delta_trans)
                delta_trans -= self.eps_b(self.alpha[3]*delta_trans+\
                                          self.alpha[4]*(delta_rot_1+delta_rot_2))
                delta_rot_2 -= self.eps_b(self.alpha[1]*delta_rot_2+\
                                          self.alpha[2]*delta_trans)
                
                # Update the pose
                particles[i,0] += delta_trans*np.cos(particles[i,2]+delta_rot_1)
                particles[i,1] += delta_trans*np.sin(particles[i,2]+delta_rot_1)
                particles[i,2] += delta_rot_1+delta_rot_2
        
        return particles

