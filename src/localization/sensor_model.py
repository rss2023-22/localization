import numpy as np
from localization.scan_simulator_2d import PyScanSimulator2D
import matplotlib.pyplot as plt
# Try to change to just `from scan_simulator_2d import PyScanSimulator2D` 
# if any error re: scan_simulator_2d occurs

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class SensorModel:

    def __init__(self):
        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")
        self.lidar_scale_to_map_scale = rospy.get_param("~lidar_scale_to_map_scale", 1.0)


        ####################################
        # TODO
        # Adjust these parameters
        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12
        self.sigma_hit = 8.0

        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        self.z_max = self.table_width-1
        ####################################

        # Precompute the sensor model table
        self.sensor_model_table = None
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                self.num_beams_per_particle,
                self.scan_field_of_view,
                0, # This is not the simulator, don't add noise
                0.01, # This is used as an epsilon
                self.scan_theta_discretization) 

        # Subscribe to the map
        self.map = None
        self.map_set = False
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)

    def precompute_sensor_model(self):

        """
        Generate and store a table which represents the sensor model.
        
        For each discrete computed range value, this provides the probability of 
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.

        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.

        args:
            N/A
        
        returns:
            No return type. Directly modify `self.sensor_model_table`.
            columns are d values!
            200x200
        """

        zmax = self.table_width-1
        sigma = self.sigma_hit 
        alphaHit = self.alpha_hit
        alphaShort = self.alpha_short
        alphaMax = self.alpha_max
        alphaRand = self.alpha_rand
        table_width = self.table_width

        plot = False # if want to plot the probability distribution. Requires matplotlib.pyplot imported as plt
        checksum = False # if want to print sum's by column (to make sure ~1.0 for proper normalization)

        def phit(zk,d):
            if 0<=zk<=zmax:
                return 1.0/((2.0*np.pi*sigma**2.0)**(0.5))*np.exp(-1.0*(((zk-d)**2.0)/(2.0*sigma**2.0)))
            return 0.0

        def pshort(zk,d):
            if zk >= 0 and zk<=d and d != 0:
                return 2.0/d*(1-(float(zk)/d))
            return 0.0

        def pmax(zk,d): 
            if zk == zmax:
                return 1.0
            return 0.0

        def prand(zk,d):
            if zk>=0 and zk <= zmax:
                return 1.0/zmax
            return 0.0

        def getP(zk,d): # everything except hit
            return alphaShort*pshort(zk,d)+alphaMax*pmax(zk,d)+alphaRand*prand(zk,d)
                
        #compute PHIT prior to others, columns are d
        out = []
        out = np.zeros((table_width,table_width))
        for i in range(table_width): # z
            for j in range(table_width): # d
                out[i][j] = phit(i,j)

        out = out/out.sum(axis=0)
        out *= alphaHit
        
        #compute other part of distribution
        for i in range(table_width):
            for j in range(table_width):
                out[i][j] += getP(i,j)

        # normalize out
        out = out/out.sum(axis=0)
        self.sensor_model_table = out

    def downsample(self, arr, spacing, mode = 'direct'):
        if mode == 'avg':
            end = spacing * (len(arr) // spacing)
            return np.mean(arr[:end].reshape(-1, spacing), axis=1)
        else:
            return  arr[0::spacing]



    def evaluate(self, particles, observation, spacing=1):
        
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar.

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        observation = self.downsample(observation,spacing)

        # UNCOMMENT THE NEXT 3 LINES IF CODE ON ACTUAL CAR!!!
        # scans = self.scan_sim.scan(particles)
        # for i in range(len(scans)):
        #     scans[i] = self.downsample(scans[i],spacing)

        z_k = np.clip(np.array(observation)/ (self.map_resolution*self.lidar_scale_to_map_scale), a_min=0, a_max = self.z_max) # clip observations
        d = np.clip(scans / (self.map_resolution*self.lidar_scale_to_map_scale), a_min = 0, a_max = self.z_max) # clip scans
        probs = np.prod(self.sensor_model_table[np.rint(z_k).astype(np.int32), np.rint(d).astype(np.int32)], axis=1)**(1/2.2) # get probabilities

        return probs

            

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map_resolution = map_msg.info.resolution

        self.map = np.array(map_msg.data, np.double)/100.
        self.map = np.clip(self.map, 0, 1)

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
                self.map,
                map_msg.info.height,
                map_msg.info.width,
                map_msg.info.resolution,
                origin,
                0.5) # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")
