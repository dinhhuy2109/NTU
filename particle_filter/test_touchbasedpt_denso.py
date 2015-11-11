#!/usr/bin/env python
import rospkg, rospy, os
import numpy as np
import actionlib
# Transformations
import tf.transformations as tr
import tf_conversions.posemath as posemath

# Denso
from denso_openrave.utils import read_parameter
from denso_openrave.interfaces import Manipulator, TFtoOpenRAVE
from denso_openrave.conversions import vector_msg_to_np
from denso_control.controllers import JointPositionController
from denso_openrave.spatial_algebra import force_frame_transform, transform_inv, skew
# Messages
from denso_msgs.msg import EndpointState, RigidBodyArray
from robotiq_action_server.msg import CModelCommandAction, CModelCommandGoal
# OpenRAVE
from openravepy import Environment, databases
from openravepy import DebugLevel, RaveSetDebugLevel, PlanningError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
# Particle Filer
import ParticleFilterLib


class PoseRefinement(object):
  global_frame = 'left/base_link'
  def __init__(self):
    # Generic configuration
    np.set_printoptions(precision = 4, suppress=True)
    # Read configuration parameters
    left_robot_name = read_parameter('~left_robot_name', 'left')
    self.js_rate = read_parameter('/%s/joint_state_controller/publish_rate' % left_robot_name, 125.0)
    self.table_id = int(read_parameter('~table_id', 2))
    self.stick_id = int(read_parameter('~stick_id', 3))
    # Populate the OpenRAVE environment
    rospack = rospkg.RosPack()
    models_dir = rospack.get_path('denso_openrave')
    env = Environment()
    left_robot = env.ReadRobotXMLFile(models_dir + '/robots/denso_ft_gripper_with_base.robot.xml')
    left_robot.SetName('left_robot')
    table = env.ReadKinBodyXMLFile(models_dir + '/objects/table.kinbody.xml')
    pin = env.ReadKinBodyXMLFile(models_dir + '/objects/pin.kinbody.xml')
    wood_stick = env.ReadKinBodyXMLFile(models_dir + '/objects/wood_stick.kinbody.xml')
    env.Add(left_robot)
    env.Add(table)
    env.Add(pin)
    env.Add(wood_stick)
    self.left_rave = Manipulator(env, left_robot, 'denso_ft_sensor_gripper')
    #Change the limits of the gripper in the model so that the model in openrave can close until touch the pin
    [lowerlimits, upperlimits] = left_robot.GetDOFLimits()
    upperlimits[-1] = np.deg2rad(43)
    left_robot.SetDOFLimits(lowerlimits, upperlimits)
    
    RaveSetDebugLevel(DebugLevel.Fatal)
    # Place elements in OpenRAVE using TF information
    self.tf_updater = TFtoOpenRAVE(env, self.global_frame)
    self.tf_updater.add_body('%s/base_link' % left_robot_name, left_robot, duration=5.)
    self.tf_updater.add_body('rigid_body_%d' % self.table_id, table, duration=5.)
    self.tf_updater.add_body('rigid_body_%d' % self.stick_id, wood_stick)
    self.tf_updater.start()
    env.SetViewer('QtCoin')
   
    # Denso Controllers
    self.left_denso = JointPositionController(left_robot_name)
    self.left_rave.SetDOFValues(self.left_denso.get_joint_positions())
   
    self.left_rave.EnableCollisionChecker()
    rospy.Subscriber('/optitrack_rigid_bodies', RigidBodyArray, self.optitrack_cb)
    rospy.loginfo('Reading object poses from optitrack...')
    
    #Set the pin into the position-in openrave env (grasped by the gripper)
    q_denso = self.left_denso.get_joint_positions()
    T_denso = self.left_rave.GetEndEffectorTransform(q_denso)
    c90 = np.cos(np.pi/2)
    s90 = np.sin(np.pi/2)
    T_pininhand = np.array(
          [[c90,-s90, 0, 0.015],
           [s90, c90, 0, 0  ],
           [0  ,0   , 1, 0.005 ],
           [0  ,0   , 0, 1  ]])
    pin.SetTransform(np.dot(T_denso, T_pininhand))
    self.left_rave.CloseGripper()
    self.left_rave.robot.WaitForController(5)
    with self.left_rave.env:
      self.left_rave.robot.Grab(pin)
    
    # Position-based explicit force control
    self.state_alive = False
    rospy.Subscriber('/%s/endpoint_state' % left_robot_name, EndpointState, self.endpoint_state_cb)
    rospy.loginfo('Waiting for [/%s/endpoint_state] topic' % left_robot_name)
    self.state_alive = False
    while not self.state_alive and not rospy.is_shutdown():
      rospy.sleep(0.1)
     
    rospy.Subscriber('/%s/ft_sensor/diagnostics' % left_robot_name, DiagnosticArray, self.diagnostics_cb)
    rospy.loginfo('Waiting for [/%s/ft_sensor/] topics' % left_robot_name)
    while not rospy.is_shutdown():
      rospy.sleep(0.1)
      if hasattr(self, 'wrench') and hasattr(self, 'ft_status'):
        break
    rospy.loginfo('Succefully connected to [/%s/ft_sensor/] topics' % left_robot_name)
    
    
    # Real code that make the robot move
    exploration_data = [] 
    # TODO: identify 3 points of 3 planes to explore (will be developed further, using region growing or planar detector..)
    point1 = np.array([0,0,0,1])
    direction = np.array([0,0,-1])
    ex1 = [point1,direction]
    exploration_data.append(ex1)
    
    point2 = np.array([-0.03,-0.01,-0.05,1])
    direction2 = np.array([1,0,0])
    ex2 = [point2,direction2]
    exploration_data.append(ex2)
    
    point3 = np.array([0., -0.01,-0.05,1])
    direction3 = np.array([0,1,0])
    ex3 = [point3,direction3]
    exploration_data.append(ex3)
    
    
    #TODO: Estimate the pin position
    do_the_pin_estimation = raw_input('Press E if you want to estimate the pin position ')
    if do_the_pin_estimation == 'e' or do_the_pin_estimation == 'E':
      estimate_hand2pin()
    else: 
      self.T_hand2pin = np.eye(4)
      self.T_hand2pin[:3,3] = np.array([-0.03,0,-0.015])
    
    
    # TODO: Collect measurements
    o_p = 2e-3
    o_n = 15/180.0*np.pi
    
    Tws = wood_stick.GetTransform()
    Tws[:3,3] = Tws[:3,3]  + np.array([0.00,0.0,0.017])
    
    collecteddata = []
    self.forcelist = []
    
    for i in range(3):
      waitkey = raw_input("Press Enter to collect new data point...")
      num_points = 0
      point = np.dot(Tws,exploration_data[i][0])
      direction = np.dot(Tws[:3,:3],exploration_data[i][1])
      touched = self.touch_surface(point[:3], direction, 0.4 , 0.03)
      if not((touched==np.zeros(3)).all()):
        num_points +=1
        d = [touched, direction, o_p,o_n]
        collecteddata.append(d)
        print "Found ", num_points, " point(s) on the surface\n" 
        print "New point found: ", touched ,"\n"
      else:
        print "Not touched! Moving to another exploration position"
        
      while num_points < 2:
        #TODO: plan new point
        waitkey = raw_input("Press Enter to continue...")
        radius = np.random.uniform(0.01,0.015)
        rand_perpendicular_vector = perpendicular_vector(direction)/np.linalg.norm(perpendicular_vector(direction))
        new_point = point[:3] + radius*rand_perpendicular_vector
        new_direction = direction
        #TODO: approach and touch
        touched = self.touch_surface(new_point, new_direction, 0.4 , 0.03)
        #TODO: update num_points
        if not((touched==np.zeros(3)).all()):
          num_points +=1
          d = [touched, new_direction, o_p,o_n]
          collecteddata.append(d) 
          print "Found ", num_points, " point(s) on the surface\n" 
          print "New point found: ", touched ,"\n"
        else:
          print "Not touched! Moving to another exploration position"
      if rospy.is_shutdown():
        return
    rospy.loginfo("COLLECT MEASUREMENT DONE")
    
    print collecteddata
    # TODO: Insert the pin into the hole
    rospy.loginfo("Move the pin to the correct position and start inserting")
    delta0 = 22
    dim = 6 # 6 DOFs
    
    ptcl0 = ParticleFilterLib.Particle(Tws[:3,3], tr.euler_from_matrix(Tws[:3,:3])) 
    V0 = ParticleFilterLib.Region([ptcl0], delta0)    
    M = 6 # No. of particles per delta-neighbohood
    delta_desired = 0.9 # Terminal value of delta
    
    woodstick = env.ReadKinBodyXMLFile('wood_stick.xml')
    env.Add(woodstick,True)
    env.Remove(wood_stick)
    list_particles, weights = ParticleFilterLib.ScalingSeries(V0, collecteddata, M, delta0, delta_desired,  dim, body = woodstick, env = env)
    est = ParticleFilterLib.VisualizeParticles(list_particles, weights, env= env, body=woodstick, showestimated = True)
    # print "Real transf. ", Tws
    # self.touch_surface(est[:3,3], direction, 0.4 , 0.03)
    
    
    rospy.loginfo("DONE")
      

  def touch_surface(self, point, direction, touch_force = 0.4, tolerance = 0.02): 
    """
    Move the robot to the pre-touched point with the given DIRECTION and TOLERANCE, then approach the surface following the given direction. Return the touched point if any  
    
    @type point:    array([x,y,z])
    @param point:    stores the point position
    @type direction:  array([n1,n2,n3])
    @param directions:  stores the appoaching direction (counter the normal of the surface)
    @type force:    float
    @param force:    condition for the touch event
    @type tolerance:  float
    @param tolerance:   distance to the surface (in meter)
    """
    direction = direction/np.linalg.norm(direction)
    found = False
    
    
    #------1ST STAGE------#
    # TODO: move to the pre-touched point with given direction  
    Tmove = np.eye(4)
    Tpin = np.eye(4)
    
    Ppin = point - tolerance*direction
    Tpin[:3,3] = Ppin
    for angle in np.linspace(0,2*np.pi,36):
      Rpin = get_rotation_matrix(angle, direction)
      Tpin[:3,:3] = Rpin[:3,:3]
      Tmove = np.dot(Tpin, self.T_hand2pin) #note that: Tmove this the transformation of the gripper NOT the pin. The transf. sent to the robot is Tmove
      try:
        self.left_rave.MoveToHandPosition(Tmove)
        rospy.loginfo('Moving the hand to pre-touched position')
        found = True
        break
      except PlanningError:
        continue
    if not found:
      rospy.logwarn('Failed MoveToHandPosition for moving the hand to the pre-touched position')
      return
    
    rate = rospy.Rate(self.js_rate)
    while not self.left_rave.IsControllerDone() and not rospy. is_shutdown():
      q = self.left_rave.GetDOFValues()
      self.left_denso.set_joint_positions(q)
      rate.sleep()
      
    # waitkey = raw_input("Press Enter to continue...")
    #~ #------2ND STAGE------#
    #~ # TODO: approach the surface with the given direction
    dt = 1. / self.js_rate
    q = self.left_rave.GetDOFValues()
    q_exploration = np.array(q)
    Wref = np.array(self.wrench)
    dx = 0.005*dt*direction #0.007
    rospy.loginfo('Approaching')
    while not rospy.is_shutdown():
      if self.ft_status == DiagnosticStatus().OK:
        J = self.left_rave.manipulator.CalculateJacobian()
        dq = np.dot(np.linalg.pinv(J), dx)
        q += dq
        self.left_rave.SetDOFValues(q)
        self.left_denso.set_joint_positions(q)
        rate.sleep()
        # FT sensor
        W = np.array(self.wrench)
        F = W[:3] - Wref[:3]
        if (np.linalg.norm(F) > touch_force):
          rospy.sleep(1)
          self.forcelist.append(np.array(self.wrench) - Wref)
          rospy.loginfo('Contact detected')
          rospy.loginfo('Moving back to pre-touched position')
          q_b = self.left_rave.GetDOFValues()
          dx_b = -0.025*dt*direction
          while not rospy.is_shutdown():
            if self.ft_status == DiagnosticStatus().OK:
              J_b = self.left_rave.manipulator.CalculateJacobian()
              dq_b = np.dot(np.linalg.pinv(J_b), dx_b)
              q_b += dq_b
              self.left_rave.SetDOFValues(q_b)
              self.left_denso.set_joint_positions(q_b)
              rate.sleep()
              # FT sensor
              W_b = np.array(self.wrench)
              # print 1
              if np.linalg.norm(self.left_rave.GetEndEffectorTransform(q_b)[:3,3] - (self.left_rave.GetEndEffectorTransform(q_exploration)[:3,3])) < 1e-4:
                rospy.loginfo('Successfully move hand to pre-touched position')
                return  np.dot(self.left_rave.GetEndEffectorTransform(q),np.linalg.inv(self.T_hand2pin))[:3,3]
        elif np.linalg.norm(self.left_rave.GetEndEffectorTransform(q)[:3,3] - (self.left_rave.GetEndEffectorTransform(q_exploration)[:3,3])) > tolerance + (0.007):
          rospy.loginfo('No object surface here')
          rospy.loginfo('Moving back to pre-touched position')
          q_b = self.left_rave.GetDOFValues()
          dx_b = -0.025*dt*direction
          while not rospy.is_shutdown():
            if self.ft_status == DiagnosticStatus().OK:
              J_b = self.left_rave.manipulator.CalculateJacobian()
              dq_b = np.dot(np.linalg.pinv(J_b), dx_b)
              q_b += dq_b
              self.left_rave.SetDOFValues(q_b)
              self.left_denso.set_joint_positions(q_b)
              rate.sleep()
              # FT sensor
              W_b = np.array(self.wrench)
              if np.linalg.norm(self.left_rave.GetEndEffectorTransform(q_b)[:3,3] - (self.left_rave.GetEndEffectorTransform(q_exploration)[:3,3])) < 1e-4:
                rospy.loginfo('Successfully move hand to pre-touched position')
                return np.array([0,0,0])
    rospy.loginfo('Finished!')
    #------3RD STAGE------#
    # TODO: return the touched point
    return 
    
  def estimate_hand2pin():
    #TODO: Estimate the pin position ###NOT UPDATED
    """Position of the pin is crucial (especially in z direction) (y = 0, x is compensated as the approaching direction is always follow x direction)
    """
    self.T_hand2pin = np.eye(4)
    self.T_hand2pin[:3,3] = np.array([-0.03,0,-0.015]) #initial transf.
    list_planepoints = []
    self.forcelist = []
    waitkey = raw_input("Press Enter to start the pin estimation...")
    plane = []
    num_points = 0
    point = np.dot(Tws,exploration_data[0][0])
    direction = np.dot(Tws[:3,:3],exploration_data[0][1])
    print "oringinal", point
    touched = self.touch_surface(point[:3], direction, 0.4 , 0.03)
    
    if not((touched==np.zeros(3)).all()):
      num_points +=1
      plane.append(touched)
      self.T_hand2pin[0,3] = self.T_hand2pin[0,3] + np.linalg.norm(point[:3]-touched[:3]) 
      waitkey = raw_input("Press Enter to start the pin estimation...")
      print "Found ", num_points, " point(s) on the surface\n" 
      print "New point found: ", touched ,"\n"
    else:
      print "Not touched\n"
    while num_points <3:
      #plan new point
      waitkey = raw_input("Press Enter to continue...")
      radius = np.random.uniform(0.005,0.0125)
      rand_perpendicular_vector = perpendicular_vector(direction)/np.linalg.norm(perpendicular_vector(direction))
      new_point = point[:3] + radius*rand_perpendicular_vector
        
      new_direction = direction
      #approach and touch
      touched = self.touch_surface(new_point, new_direction, 0.4 , 0.03)
      #update num_points
      if not((touched==np.zeros(3)).all()):
        num_points +=1
        plane.append(touched)
        print "Found ", num_points, " point(s) on the surface\n" 
        print "New point found: ", touched ,"\n"
        print new_point, "\n"
    avg = np.zeros(6)
    for w in self.forcelist:
      avg = avg + w
    avg = avg/len(self.forcelist)
    print avg
    # print np.linalg.norm(avg[3:])/np.linalg.norm(avg[:3])-0.155
    self.T_hand2pin[2,3] = 0.155-np.linalg.norm(avg[3:])/np.linalg.norm(avg[:3]) # update z direction
    print "The new-estimated-pin-hand transformation:\n",self.T_hand2pin
    
  def endpoint_state_cb(self, msg):
    force = vector_msg_to_np(msg.wrench.force)
    torque = vector_msg_to_np(msg.wrench.torque)
    self.wrench = np.hstack((force, torque))
    self.jnt_pos_filt = np.array(msg.joint_state.position)
    if not self.state_alive:
      self.state_alive = True
  
  def diagnostics_cb(self, msg):
    self.ft_status = msg.status[0].level

  def optitrack_cb(self, msg):
    if msg.header.frame_id != self.global_frame:
      return
    for rigid_body in msg.bodies:
      if rigid_body.tracking_valid:
        if rigid_body.id == self.table_id:
          frame = posemath.fromMsg(rigid_body.pose)
          self.Ttable = posemath.toMatrix(frame)
        if rigid_body.id == self.stick_id:
          frame = posemath.fromMsg(rigid_body.pose)
          self.Tshort = posemath.toMatrix(frame)
        #~ if rigid_body.id == self.pin_id:
          #~ frame = posemath.fromMsg(rigid_body.pose)
          #~ self.Tpin = posemath.toMatrix(frame)

def get_rotation_matrix(angle,axis):
  """
  This function returns a rot matrix (The new frame has the x axis aligned with the GIVEN axis)
  @type angle  : float  
  @param angle : radian
  @type axis   : array([x,y,z])
  @param axis  : new x-axis (normalized)
  @type rot_matrix: 3x3 matrix
  @param rot_matrix: rotation matrix
  """
  old_x = np.array([1,0,0])
  v = np.cross(old_x, axis)
  I = np.eye(3)
  K = skew(v)
  c = np.dot(old_x, axis)
  s = np.linalg.norm(v)
  R = I + s*K + (1-c)*np.linalg.matrix_power(K,2)
  R_angle =  tr.rotation_matrix(angle, old_x)
  rot_matrix = np.dot(R, R_angle[:3,:3])
  return rot_matrix

def perpendicular_vector(v):
  """ Finds an arbitrary perpendicular vector to *v*. The idea here is finding a perpendicular vector then rotating abitraryly it around v
  @type v np.array([x,y,z])
  """
  # for two vectors (x, y, z) and (a, b, c) to be perpendicular,
  # the following equation has to be fulfilled
  #  0 = ax + by + cz
  # x = y = z = 0 is not an acceptable solution
  if v[0] == v[1] == v[2] == 0:
    raise ValueError('zero-vector')

  # If one dimension is zero, this can be solved by setting that to
  # non-zero and the others to zero. Example: (4, 2, 0) lies in the
  # x-y-Plane, so (0, 0, 1) is orthogonal to the plane.
  random_rotation = tr.rotation_matrix(np.random.uniform(0, np.pi),v)[:3,:3]
  if v[0] == 0:
    return np.dot(random_rotation,np.array([1, 0, 0]))
  if v[1] == 0:
    return np.dot(random_rotation,np.array([0, 1, 0]))
  if v[2] == 0:
    return np.dot(random_rotation,np.array([0, 0, 1]))

  # set a = b = 1
  # then the equation simplifies to
  #  c = -(x + y)/z
  return np.dot(random_rotation,np.array([1, 1, -1.0 * (v[0] + v[1]) / v[2]]))

def fitPLaneLTSQ(plist):
  """
  Given an list of points 
  representing points in d-dimensional space,
  fit an d-dimensional plane to the points.
  Return 4 parameters (using a point, ctr, on the plane (the point-cloud centroid), and the normal). 
  """
  G = np.ones((len(plist),3))
  G[:,0] = plist[:][0]  #X
  G[:,1] = plist[:][1]  #Y
  G[:,2] = plist[:][2]  #Z

  points = np.reshape(G, (np.shape(G)[0], -1)) # Collapse trialing dimensions
  assert points.shape[0] <= points.shape[1], "There are only {} points in {} dimensions.".format(points.shape[1], points.shape[0])
  ctr = points.mean(axis=1)
  x = points - ctr[:,np.newaxis]
  M = np.dot(x, x.T) # Could also use np.cov(x) here.
  normal = np.linalg.svd(M)[0][:,-1]
  d = np.dot(normal,ctr)
  return np.array([normal[0], normal[1],normal[2],d])

if __name__ == '__main__':
  # Initialize the node
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  demo = PoseRefinement()
  rospy.spin()
  rospy.loginfo('Shuting down [%s] node' % node_name)
