import numpy as np

class MotionModel:

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        pass

        ####################################

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
        
        ####################################
        # TODO: add noise

        N = particles.shape[0]
        for i in range(N):
            #Rotate the position components of the odometry to the world frame
            cos,sin = np.cos(particles[i,2]),np.sin(particles[i,2])
            rotated_displacement = (cos*odometry[0]-sin*odometry[1],
                                    sin*odometry[0]+cos*odometry[1])
                            
            #Add the displacements to the pose
            particles[i,0] += rotated_displacement[0]
            particles[i,1] += rotated_displacement[1]
            particles[i,2] += odometry[2]
        
        return particles
            
        ####################################
