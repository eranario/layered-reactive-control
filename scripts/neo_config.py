# This is a refactoring of the NEO Tutorial by @author Jesse Haviland

import time
import random
import datetime
from pprint import pprint

# NEO TUTORIAL IMPORTS
import swift
import spatialgeometry as sg
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp

class Experimentor():

    '''
    This is an Experimentor class object.
    This is used to conduct experiments with an explicit set of conditions.
    '''

    def __init__(self):

        pass

    def setEnvironment(self, realtime = False, headless = False, browser = 'safari'):

        '''
        This is a method to set the environment.
        '''

        self.environment = swift.Swift()
        self.environment.launch(realtime = realtime, headless = headless, browser = browser)

    def setRobot(self, num_joints: int):

        '''
        This is a method to set the robot object.

        Parameters
        --------------------
        num_joints : int
            This is the number of joints in the robot to activate.

        Returns
        --------------------
        None
        '''

        self.robot = {
            'robot'         : rtb.models.Panda(),  # Create robot object
            'joints'        : num_joints, 
            'crash'         : False
        }  
       
        self.robot['robot'].q = self.robot['robot'].qr            # Set joint angles to ready configuration

        self.setEnvironmentObject(self.robot['robot'])

    def setObstables(self, obstacles: list):

        '''
        This is a method to set the scene obstacles.

        Parameters
        --------------------
        obstacles : list-like
            List containing the obstacles within the scene.
            Obstacles created using the spatialgeometry library.
            E.g., [(Sphere, radius, pose, velocity), (Cuboid, [length, width, height], pose, velocity)]

        Returns
        --------------------
        None
        '''

        if not hasattr(self, 'obstacles'):
            self.obstacles = {}
        
        # Check is obstacles already in dictionary.
        # If return the count of obstacles in the dictionary.
        count_of_existing_obstacles = len(self.obstacles)
        for i, obstacle in enumerate(obstacles):
            method = getattr(sg, obstacle[0])
            if obstacle[0] == 'Cylinder':
                object = method(obstacle[1][0], obstacle[1][1], pose = sm.SE3(obstacle[2]), collision = True)
            else:
                object = method(obstacle[1], pose = sm.SE3(obstacle[2]), collision = True)
            object.color = genRandomColor(1.0)
            self.obstacles[(i+1) + count_of_existing_obstacles] = {'object' : object}
            self.obstacles[(i+1) + count_of_existing_obstacles]['object'].v = obstacle[3]
            self.obstacles[(i+1) + count_of_existing_obstacles]['detected'] = False #TODO: need to change how detected is applied.
            self.obstacles[(i+1) + count_of_existing_obstacles]['name'] = obstacle[4]
            self.setEnvironmentObject(object)

    def setTarget(self, target: tuple):

        '''
        This is a method to set the scene target.

        Parameters
        --------------------
        obstacles : tuple
            Tuple containing the target within the scene.
            Target created using the spatialgeometry library.
            E.g., (Sphere, radius, pose).

        Returns
        --------------------
        None
        '''

        if not hasattr(self, 'target'):
            self.target = {}

        method = getattr(sg, target[0])
        object = method(target[1], pose = sm.SE3(target[2]), collision = True)
        object.color = [1.0, 0.0, 1.0, 1.0]
        self.target = object
        self.setEnvironmentObject(object)

    def setSensingRegion(self, regions: list):

        '''
        Parameters
        --------------------
        region : list-like
            List containing the sensing regions within the scene.
            Regions created using the spatialgeometry library.
            E.g., [
                (
                    Sphere, radius, pose, velocity, working_distance, fixed, color), (Cuboid, [length, width, height], pose, velocity, working_distance, fixed, color)]

        Returns
        --------------------
        None
        '''
        
        if not hasattr(self, 'sensing_regions'):
            self.sensing_regions = {}

        count_of_existing_regions = len(self.sensing_regions)
        for i, region in enumerate(regions):
            method = getattr(sg, region[0])
            object = method(region[1], pose = sm.SE3(region[2]), collision = True)
            object.color = region[6]
            self.sensing_regions[(i+1) + count_of_existing_regions] = {'object' : object}
            self.sensing_regions[(i+1) + count_of_existing_regions]['object'].v = region[3]
            self.sensing_regions[(i+1) + count_of_existing_regions]['working_distance'] = np.array(region[4])
            self.sensing_regions[(i+1) + count_of_existing_regions]['fixed'] = region[5]
            self.setEnvironmentObject(object)

    def createSensor(self, args):

        '''
        This is a method to setup a camera sensor.

        ### (Cuboid, [length, width, height], pose, velocity, working_distance, fixed, color)
        ### ('Cuboid', [0.5, 0.5, 0.5], [0,0,1], [0, 0, 0, 0, 0, 0], [0,0,0.2], True, [0,1,0,0.5])

        Parameters
        --------------------
        fov : tuple of floats
            (horizontal, vertical) angular viewing span.

        fov_resolution: tuple of ints
            (horizontal, vertical) sensing resolution.

        depth : tuple of floats
            (minimum, maximum) effective viewing distances.

        depth_resolution : int
            Number of discrete sensing volumes to make over effective viewing range.

        depth_limit : float
            Simulation limit for sensing range. 
            Limited by the diameter of the robots reach. 

        Returns
        --------------------

        '''
        
        # extract arguments
        fov, fov_resolution, depth, depth_resolution, fixed, depth_limit = args

        color = genRandomColor(0.1)

        if depth_limit:
            if depth[1] > depth_limit:
                print(f"The sensing range is greater than the working volume.\nLimiting the sensing range to {depth_limit} meters.")
                depths = np.linspace(depth[0], depth_limit, depth_resolution+1)
            else:
                depths = np.linspace(depth[0], depth[1], depth_resolution+1)
        else:
            depths = np.linspace(depth[0], depth[1], depth_resolution+1)
        working_distances = depths + (0.5 * (depths[1] - depths[0]))
        h_bounds, v_bounds = [depths * -(np.tan(np.radians(0.5*fov[0]))), depths * +(np.tan(np.radians(0.5*fov[0])))], [depths * -(np.tan(np.radians(0.5*fov[1]))), depths * +(np.tan(np.radians(0.5*fov[1])))]
        sensing_regions = []
        for interval in range(len(depths)-1):
            if fixed:
                pose = fixed.copy()
                pose[2] -= working_distances[interval]
                region = ('Cuboid', [h_bounds[1][interval]*2, v_bounds[1][interval]*2, depths[interval+1]-depths[interval]], pose, [0,0,0,0,0,0], [0,0,working_distances[interval]], fixed, color)
            else:
                region = ('Cuboid', [h_bounds[1][interval]*2, v_bounds[1][interval]*2, depths[interval+1]-depths[interval]], [0,0,1], [0,0,0,0,0,0], [0,0,working_distances[interval]], fixed, color)
            sensing_regions.append(region)
        
        return sensing_regions
            
    def transformObject(self, object, htm):

        object.T = htm

    def setEnvironmentObject(self, object):

        '''
        This is a method to add created objects to the scene.

        Parameters
        --------------------
        object : object-like
            These are objects (e.g., robot, obstacles, target)

        Returns
        --------------------
        None
        '''

        self.environment.add(object)

    def setEndPose(self):

        '''
        This is a method to set the desired end-effector pose to the location of target
        '''

        self.end_pose = self.robot['robot'].fkine(self.robot['robot'].q)
        self.end_pose.A[:3, 3] = self.target.T[:3, -1]

    def getPose(self):

        '''
        This is a method to get the current pose of the robot's end effector.
        '''

        self.current_pose = self.robot['robot'].fkine(self.robot['robot'].q)

        return self.current_pose.__dict__['data'][0]
    
    def setTranformPose(self):

        '''
        This is a method to transform from the end-effector to desired pose.
        '''

        self.transform = self.current_pose.inv() * self.end_pose

    def getSpatialError(self):

        '''
        This is a method to set the spatial error.
        '''

        # Spatial error
        self.spatial_error = np.sum(np.abs(np.r_[self.transform.t, self.transform.rpy() * np.pi / 180]))

    def getVelocity(self, controller_gain: float, error_threshold: float):

        '''
        This is a method to get the spatial velocity of the robot.

        :param gain: The gain for the controller. Can be vector corresponding to each
        axis, or scalar corresponding to all axes.
        :type gain: float, or array-like
        :param threshold: The threshold or tolerance of the final error between
            the robot's pose and desired pose
        '''

        self.robot['velocity'], self.robot['arrived'] = rtb.p_servo(self.current_pose, self.end_pose, controller_gain, error_threshold)

    def setControlMinimization(self, control_minimization: float):

        '''
        This is method to set gain term (lambda) for control minimization.
        '''

        self.gain_control_minimization = control_minimization

    def setQuadraticComponent(self):

        '''
        This is a method to set the quadratic component of the objective function.
        '''

        self.quadratic_component_Q = np.eye(self.robot['joints']+6)

    def setJointVelocityQ(self):

        '''
        This is a  method to set the joint velocity component of the quadratic component of the objective function.
        '''

        self.quadratic_component_Q[:self.robot['joints'], :self.robot['joints']] *= self.gain_control_minimization

    def setSlackComponentQ(self):

        '''
        This is a method to set the slack component of the quadratic component of the objective function.
        '''

        self.quadratic_component_Q[self.robot['joints']:, self.robot['joints']:] = (1 / self.spatial_error) * np.eye(6)

    def getEqualityConstaints(self):

        '''
        This is a method to get the equality contraints.
        '''

        self.Aeq = np.c_[self.robot['robot'].jacobe(self.robot['robot'].q), np.eye(6)]
        self.beq = self.robot['velocity'].reshape((6,)) 

    def getInequalityConstraints(self):

        '''
        This is a method to get the inequailty constraints.
        '''

        self.Ain = np.zeros((self.robot['joints'] + 6, self.robot['joints'] + 6))
        self.bin = np.zeros(self.robot['joints'] + 6)
        
    def setJointApproachLimit(self, approach_limit):

        '''
        The minimum angle (in radians) in which the joint is allowed to approach.

        Parameters
        --------------------
        approach_limit : float
            The minimum angle (in radians) in which the joint is allowed to approach.

        Returns
        --------------------
        '''

        self.joint_approach_limit = approach_limit
    
    def setJointApproachDamperThreshold(self, damper_threshold):

        '''
        This angle (in radians) in which the the velocity damper becomes active.
        '''

        self.joint_velocity_damper_threshold = damper_threshold

    def setJointParameters(self, approach_limit, damper_threshold):

        '''
        This is a method to set the joint approach limit and joint velocity damper threshold.
        Create the ...
        '''

        self.setJointApproachLimit = approach_limit
        self.setJointApproachDamperThreshold = damper_threshold

        self.Ain[:self.robot['joints'], :self.robot['joints']], self.bin[:self.robot['joints']] = self.robot['robot'].joint_velocity_damper(
            self.setJointApproachDamperThreshold, 
            self.setJointApproachLimit, 
            self.robot['joints']
            )
        
    def objectDetection(self, ideal = False, sensor = None):

        '''
        This is a method to check of obstacles within sensible regions.
        '''

        if not hasattr(self, 'obstacles'):
            print(f"There are no obstacles to detect.", end = '\r')
        else:
            for collision in self.obstacles:
                # Check the obstacle sensible state // account for ideal testing
                if ideal:
                    state = True
                    print(f"Running in the ideal state...", end = '\r')
                else:
                    if sensor:
                        #TODO: add object handling...
                        sensor.updatePose(self.getPose())
                        state = sensor.inVolume(self.obstacles[collision]['object'].__dict__['_SceneNode__T'][:3,3])
                        # print(f"{collision} -> {state} -----", end = '\r')
                    else:
                        state = self.obstacles[collision]['detected']
                # If the obstacle is detectable do stuff
                if state == True:
                    # Form the velocity damper inequality contraint for each collision
                    # object on the robot to the collision in the scene
                    c_Ain, c_bin = self.robot['robot'].link_collision_damper(
                        self.obstacles[collision]['object'],
                        self.robot['robot'].q[:self.robot['joints']],
                        0.3,
                        0.05,
                        1.0,
                        start=self.robot['robot'].link_dict["panda_link1"],
                        end=self.robot['robot'].link_dict["panda_hand"],
                        )

                    # If there are any parts of the robot within the influence distance
                    # to the collision in the scene
                    if c_Ain is not None and c_bin is not None:
                        c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], 4))]

                        # Stack the inequality constraints
                        self.Ain = np.r_[self.Ain, c_Ain]
                        self.bin = np.r_[self.bin, c_bin]

    def objectDetectionPrime(self, ideal = False, sensor = None):

        '''
        This is a method to check of obstacles within sensible regions.
        '''

        if not hasattr(self, 'obstacles'):
            print(f"There are no obstacles to detect.", end = '\r')
        else:
            scorecard = {}
            for collision in self.obstacles:
                # CHECK IF ANY OBSTACLE HAS HIT THE ROBOT
                if self.robot['robot'].iscollided(self.robot['robot'].q, self.obstacles[collision]['object']):
                    # self.obstacles[collision]['object'].color = self._to_hex([1,0,0,1])
                    self.robot['crash'] = True

                else:
                    # self.obstacles[collision]['object'].color = [0.3, 0.3, 0.3, 1]
                    pass

                # Check the obstacle sensible state // account for ideal testing
                if ideal:
                    state = True
                    print(f"Running in the ideal state...", end = '\r')
                else:
                    if sensor:
                        state = False
                        for unit in self.sensing_regions:
                            sensed = self.sensing_regions[unit]['object'].iscollided(self.obstacles[collision]['object'])
                            if sensed:
                                state = True
                    else:
                        state = self.obstacles[collision]['detected']
                # UPDATE SCORECARD
                scorecard[collision] = (state, self.obstacles[collision]['name']) # self.obstacles[collision]['object'].to_dict()['stype']
                
                # If the obstacle is detectable do stuff
                if state == True:
                    # Form the velocity damper inequality contraint for each collision
                    # object on the robot to the collision in the scene
                    c_Ain, c_bin = self.robot['robot'].link_collision_damper(
                        self.obstacles[collision]['object'],
                        self.robot['robot'].q[:self.robot['joints']],
                        0.3,
                        0.05,
                        1.0,
                        start=self.robot['robot'].link_dict["panda_link1"],
                        end=self.robot['robot'].link_dict["panda_hand"],
                        )

                    # If there are any parts of the robot within the influence distance
                    # to the collision in the scene
                    if c_Ain is not None and c_bin is not None:
                        c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], 4))]

                        # Stack the inequality constraints
                        self.Ain = np.r_[self.Ain, c_Ain]
                        self.bin = np.r_[self.bin, c_bin]
                # CHECK IF THE ROBOT HAS COLLIDED WITH ANY OBSTACLES
            scorecard['crash'] = self.robot['crash']               
            print(f"Detected Obstacles: {scorecard}                           ", end = '\r')

    def getManipulabilityJacobian(self):

        '''
        This is a method to get the linear component of the objective function (i.e., the manipulability Jacobian)
        '''

        self.manipulability_jacobian = np.r_[-self.robot['robot'].jacobm(self.robot['robot'].q).reshape((self.robot['joints'],)), np.zeros(6)]

    def getJointParameterBounds(self):

        '''
        This is is a method to get the bounds of the joint velocity and slack variable.
        '''

        if not hasattr(self, 'joint_parameter_limits'):
            self.joint_parameter_limits = {}
        self.joint_parameter_limits['lower'] = -np.r_[self.robot['robot'].qdlim[:self.robot['joints']], 10 * np.ones(6)]
        self.joint_parameter_limits['upper'] = np.r_[self.robot['robot'].qdlim[:self.robot['joints']], 10 * np.ones(6)]

    def getJointVelocityDQ(self):

        '''
        This is a method to get the joint velocity DQ.
        '''

        self.qd = qp.solve_qp(
            self.quadratic_component_Q, 
            self.manipulability_jacobian, 
            self.Ain, 
            self.bin, 
            self.Aeq, 
            self.beq, 
            lb=self.joint_parameter_limits['lower'], 
            ub=self.joint_parameter_limits['upper'], 
            solver='daqp'
            )
        
        # print(f"self.qd: {self.qd}                                              ", end = '\r')

    def setJointVelocity(self):

        '''
        This is a method to set the joint velocities that have been solved for.
        '''

        self.robot['robot'].qd[:self.robot['joints']] = self.qd[:self.robot['joints']]
        

    def step(self, step_time = 0.01, ideal = False, sensor = None):

        '''
        This is a method to step throught the experiment.

        Parameters
        --------------------
        step_time : float
            Time in seconds to step environment.

        ideal : bool
            Set to True to run under purely reactive with infinite sensing.
            Set false to run with sensing conditions.
        
        sensor : ObstacleDetector.ObstacleDetector
            Sensor used to observe obstacles.

        Returns
        --------------------

        '''
        self.setEndPose()
        self.getPose()
        if hasattr(self, 'sensing_regions'):
            for sensor in self.sensing_regions:
                sensor_pose = self.getPose()
                sensor_pose[:3,3] += np.dot(self.getPose()[:3, :3], self.sensing_regions[sensor]['working_distance'].T)
                if not self.sensing_regions[sensor]['fixed']:
                    self.transformObject(self.sensing_regions[sensor]['object'], sensor_pose)
        self.setTranformPose()
        self.getSpatialError()
        self.getVelocity(0.5, 0.01) # Example Parameters : 0.5, 0.01 
        self.setControlMinimization(0.01) # Example Parameters : 0.01
        self.setQuadraticComponent()
        self.setJointVelocityQ()
        self.setSlackComponentQ()
        self.getEqualityConstaints()
        self.getInequalityConstraints()
        self.setJointParameters(0.05, 0.9) # Example Parameters : 0.05, 0.9
        self.objectDetectionPrime(ideal = ideal, sensor = sensor)
        self.getManipulabilityJacobian()
        self.getJointParameterBounds()
        self.getJointVelocityDQ()
        self.setJointVelocity()
        self.environment.step(step_time)

        return self.robot['arrived']
    
    def run(self, step_time, ideal = False, sensor = None):

        self.robot['arrived'] = False
        while not self.robot['arrived']:
            self.robot['arrived'] = self.step(step_time = step_time, ideal = ideal, sensor = sensor)
        print(f"The robot reached the target! {' ' * 200}")

def genRandomColor(opacity):
    
    color = []
    while len(color) < 3:
        color.append(random.random())
    color.append(opacity)

    return color
    

def runExperiment(
    realtime: bool = False,
    headless: bool = False,
    sensor_args: list = None,
    config_args: list = None
 ):
    
    Experiment = Experimentor()
    Experiment.setEnvironment(realtime = False, headless = False, browser = 'chrome')
    Experiment.environment.set_camera_pose([-1.0, -1.0, 1.0], [0.0, 0.0, 0.0])
    Experiment.setRobot(7)
    
    # -----------------------------------
    # SET SENSING PROVISIONS
    sensors = []
    for args in sensor_args:
        
        # create sensor
        for region in Experiment.createSensor(args):
            sensors.append(region)

    Experiment.setSensingRegion(
        sensors
        )
    for sensor in Experiment.sensing_regions:
        if Experiment.sensing_regions[sensor]['fixed'] == False: # check if sensor is fixed or not
            init_sensor_pose = Experiment.getPose()
            init_sensor_pose[:3,3] += Experiment.sensing_regions[sensor]['working_distance'].T
            Experiment.transformObject(Experiment.sensing_regions[sensor]['object'], init_sensor_pose)
    # -----------------------------------
    # -----------------------------------
    # SET OBSTACLES 
    # end effector ~0.5m from the base of the arm
    fixed_obstacles = [
        ('Cuboid', [1.0, 2.0, 0.01], [0.5, 0.0, -0.01], [0, 0, 0, 0, 0, 0], 'Floor'),               # FLOOR
        ('Cuboid', [1.0, 0.1, 0.6], [0.5, 1.0, 0.3], [0, 0, 0, 0, 0, 0], 'Left_Wall'),               # WALL LEFT
        ('Cuboid', [1.0, 0.1, 0.6], [0.5, -1.0, 0.3], [0, 0, 0, 0, 0, 0], 'Right_Wall'),               # WALL RIGHT
        ('Cuboid', [0.1, 2.0, 0.6], [1.0, 0.0, 0.3], [0, 0, 0, 0, 0, 0], 'Front_Wall')               # WALL FRONT
    ]
    all_obstacles = fixed_obstacles + config_args
    Experiment.setObstables(all_obstacles)
    
    # SET TARGET
    Experiment.setTarget(('Sphere', 0.03, [0.75, 0.0, 0.1]))
    # -----------------------------------
    Experiment.step(step_time = 0.01, ideal = False, sensor = None) #Sensor)
    Experiment.run(step_time = 0.01, ideal = False, sensor = None)
    minutes, seconds = divmod(Experiment.environment.sim_time, 60)
    print(f"The time to reach the target was: {int(minutes)} minutes, {seconds:.3f} seconds")
    # Experiment.environment.stop_recording()