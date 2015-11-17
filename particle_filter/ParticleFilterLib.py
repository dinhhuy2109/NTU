import numpy as np

import Transformation
import Utils

def VisualizeParticles(list_particles,weights,env='',body ='', showestimated = False):
    maxweight = 0
    for w in weights:
        if w > maxweight:
            maxweight = w    
    if showestimated is False:
        weight_threshold = 0.7*maxweight
        from openravepy import RaveCreateKinBody
        with env:
            env.Reset()
            newbodies = []
            for i in range(len(list_particles)):
                if weights[i] > weight_threshold:
                    p = list_particles[i]
                    transparency = 1 - weights[i]/maxweight
                    transf = p.transformation()
                    newbody = RaveCreateKinBody(env,body.GetXMLId())
                    newbody.Clone(body,0)
                    newbody.SetName(body.GetName())
                    for link in newbody.GetLinks():
                        for geom in link.GetGeometries():
                            geom.SetTransparency(transparency)
                            geom.SetDiffuseColor([0.64,0.35,0.14])
                            env.AddKinBody(newbody,True)
                    with env:
                        newbody.SetTransform(transf)

                    newbodies.append(newbody)
    else:
        acum_weight = 0
        acum_trans = np.zeros(3)
        acum_euler = np.zeros(3)
        weight_threshold = 0.7*maxweight
        for i in range(len(list_particles)):
            if weights[i] > weight_threshold:
                p = list_particles[i]
                acum_trans += p.trans*weights[i]
                acum_euler += p.euler*weights[i]
                acum_weight += weights[i]
        estimated_particle = Particle( acum_trans/acum_weight, acum_euler/acum_weight)
        transf = estimated_particle.transformation()
        print "Resulting estimation:\n", transf
        print "Trans ", estimated_particle.trans, " Euler ", estimated_particle.euler
        body.SetTransform(transf)
        env.AddKinBody(body,True)
        #~ from openravepy import RaveCreateKinBody
        #~ with env:
            #~ env.Reset()
            #~ transf = estimated_particle.transformation()
            #~ print "Resulting estimation:\n", transf
            #~ print "Trans ", estimated_particle.trans, " Euler ", estimated_particle.euler
            #~ newbody = RaveCreateKinBody(env,body.GetXMLId())
            #~ newbody.Clone(body,0)
            #~ newbody.SetName(body.GetName())
            #~ for link in newbody.GetLinks():
                #~ for geom in link.GetGeometries():
                    #~ geom.SetDiffuseColor([0.64,0.35,0.14])
                    #~ env.AddKinBody(newbody,True)
            #~ with env:
                #~ newbody.SetTransform(transf)
        return transf

            

'''Measurement unit:
trans: meter
rot: degree
initial delta: should be large, e.g. 10 or 5
'''
class Region(object):
    def __init__(self, particles_list, delta):
        self.particles_list = particles_list
        self.delta = delta

class Particle(object):
    def __init__(self, trans, euler):
        assert len(trans)==3 & len(euler) == 3, "Wrong type, trans and euler are vectors of 3 elements"
        self.trans =  trans
        self.euler = euler
    
    def rotation(self):
        rot = Transformation.euler_matrix(self.euler[0]/180*np.pi,self.euler[1]/180*np.pi,self.euler[2]/180*np.pi,axes='sxyz')[:3,:3]
        return rot
    def transformation(self):
        T = np.eye(4)
        T[:3,:3] = self.rotation()
        T[:3,3] = self.trans
        return T
        
def EvenDensityCover(region, M):
    '''Input: region - sampling region represented as a union of neighborhoods, M - number of particles to sample per neighborhood
    Output: a set of particles that evenly cover the region
    '''
    list_particles = []
    SO3radius = Utils.R3Distance(np.array([region.delta,region.delta,region.delta]),np.zeros(3))
    R3radius = Utils.R3Distance(np.array([region.delta/1000.,region.delta/1000.,region.delta/1000.]), np.zeros(3))
    for i  in range(len(region.particles_list)):
        center_particle = region.particles_list[i]
        #TODO: Count how many particles in list_particles are in the sphere of center_particle? Save as existing_p (if existing_p > M, error)
        num_existing_p = 0
        for p in list_particles:
            distanceSO3 = Utils.R3Distance(p.euler,center_particle.euler)
            distanceR3 = Utils.R3Distance(p.trans, center_particle.trans)
            ############RECHECK this distance
            if (distanceSO3 < SO3radius) and (distanceR3 < R3radius):
                num_existing_p += 1
        #Following loop is to sample and reject
        for m in range(M-num_existing_p):
            #TODO: Sample a particle in the sphere region.delta (sample in SE(3) using the parameter delta)
            new_trans = np.asarray(center_particle.trans) + np.random.uniform(-region.delta/1000.0,region.delta/1000.0,size = 3)
            new_euler = np.asarray(center_particle.euler) + np.random.uniform(-region.delta,region.delta,size = 3)  ############## HOW TO SAMPLE!! It should be around the center which we are considering
            new_p = Particle(new_trans, new_euler)
            #TODO: Check distances from the new particle to other spheres (1 -> previous center_particle's sphere (sphere here is jst a word to visualize the idea, proper word should be neighborhood)
            accepted = True
            for k in range(i-1):
                previous_sphere_center = region.particles_list[k]
                distanceSO3_2prvsphere = Utils.R3Distance(new_p.euler,previous_sphere_center.euler)
                distanceR3_2prvsphere = Utils.R3Distance(new_p.trans,previous_sphere_center.trans)
            ############RECHECK this distance
                if distanceSO3_2prvsphere < SO3radius and distanceR3_2prvsphere < R3radius:
                    accepted = False
                    break
            #TODO: if satified, add this particle to list_particles
            if accepted:
                list_particles.append(new_p)
    return list_particles

def ComputeNormalizedWeights(list_particles, weights,measurements,tau,body='',env=''):
    new_weights = np.zeros(len(weights))
    for i in range(len(list_particles)):
        p = list_particles[i]
        T = p.transformation()
        body.SetTransform(T)
        poly = env.Triangulate(body)
        total_energy = sum([CalculateMahaDistancePolygon(poly,d)**2 for d in measurements])
        new_weights[i] = weights[i]*np.exp(-total_energy/tau)
    #print "Weights before normalization", new_weights
    return normalize(new_weights)

def normalize(weights):
    norm_weights = np.zeros(len(weights))
    sum_weights = np.sum(weights)
    for i in range(len(weights)):
        norm_weights[i] = weights[i]/sum_weights
    return norm_weights

def CalculateMahaDistanceMesh(trimesh,d):
    '''

    :param trimesh:     Vector [p1,p2,p3,v]: three points and one vector
    :param d:           Measurement data [p,n,o_n,o_p]: measurement point and vector
    :return:
    '''
    p1,p2,p3 = trimesh
    normal = lambda x,y,z: np.cross(y-x,z-x)/np.linalg.norm(np.cross(y-x,z-x))
    v = normal(p1,p2,p3)
    p,n,o_p,o_n = d
    v = -v #reverse normal of the mesh
    # Get distance to surface
    norm = lambda x: np.linalg.norm(x)
    inner = lambda a, b: np.inner(a,b)
    diff_distance   = norm(inner((p-p1), v)/norm(v))
    # Get differences in normal direction
    diff_angle      = np.arccos(inner(v, n)/norm(v)/norm(n))
    # Maha distance
    ans = np.sqrt(diff_distance**2/o_p**2+diff_angle**2/o_n**2)
    return ans

def CalculateMahaDistancePolygon(poly,d):
    '''
    :param  poly:       A trimesh object
    :param  d   :       A measurement [p,n,o_n,o_p]
    '''
    dis = [CalculateMahaDistanceMesh([poly.vertices[x],poly.vertices[y],poly.vertices[z]],d) for x,y,z in poly.indices]
    return min(dis)

def Pruning(list_particles, weight):
    assert (len(list_particles)==len(weight)),"Wrong input data, length of list of particles are not equal to length of weight"
    pruned_list = []
    maxweight = 0
    for w in weight:
        if w > maxweight:
            maxweight = w
    threshold = 0.8*maxweight
    for i in range(len(list_particles)):
        if weight[i] > threshold:
            pruned_list.append(list_particles[i])
    return pruned_list

def ScalingSeries(V0, D, M, delta0, delta_desired, dim = 6, body = '', env ='', visualize = False):
    """
    @type  V0:  ParticleFilterLib.Region
    @param V0:  initial uncertainty region
    @param  D:  a list of measurements [p,n,o_n,o_p] p is the contacted point, n is the approaching vector (opposite to normal)
    @param  M:  the no. of particles per neighborhood
    @param delta_desired: terminal value of delta
    @param dim: dimension of the state space (6 DOFs)
    """ 
    zoom = 2**(-1./dim)
    N = int(np.round(np.log2((delta0/delta_desired)**dim)))
    uniform_weights = normalize(np.ones(len(V0.particles_list)))
    # Main loop
    delta_prv = delta0
    V_prv = V0
    list_particles = []
    weights =[]
    for n in range(N):
        delta = zoom*delta_prv
        tau = (delta/delta_desired)**2
        # Sample new set of particles based on from previous region and M
        list_particles = EvenDensityCover(V_prv,M)
        print "No. of particles of the ", n+1, " run: ", len(list_particles), "particles"
        # Compute normalized weights
        uniform_weights = normalize(np.ones(len(list_particles)))
        # print "uniform ",uniform_weights 
        weights = ComputeNormalizedWeights(list_particles, uniform_weights, D, tau, body,env)
        # print "weights after normalizing",  weights
        
        if visualize:
            VisualizeParticles(list_particles, weights, env, body)
        
        # for p in list_particles:
        #     print p.transformation()
        # Prune based on weights
        pruned_list_particles = Pruning(list_particles,weights)
        
        print 'No. of particles, after pruning:', len(pruned_list_particles)
        # raw_input("Press Enter to continue...")
        # Create a new region from the set of particle left after pruning
        V_prv = Region(pruned_list_particles,delta)
        delta_prv = delta
        print "delta_prv",  delta_prv
    new_set_of_particles = EvenDensityCover(V_prv,M)
    print V_prv.delta
    uniform_weights = normalize(np.ones(len(new_set_of_particles)))
    new_weights = ComputeNormalizedWeights(new_set_of_particles, uniform_weights,D,1,body,env)
    return new_set_of_particles,new_weights

def PartialScalingSeries(V0, D, M, delta0, delta_desired, dim = 6, body = '', env ='', visualize = False):
    """
    @type  V0:  ParticleFilterLib.Region
    @param V0:  initial uncertainty region
    @param  D:  a list of measurements [p,n,o_n,o_p] p is the contacted point, n is the approaching vector (opposite to normal)
    @param  M:  the no. of particles per neighborhood
    @param delta_desired: terminal value of delta (is an array)
    @param dim: dimension of the state space (6 DOFs)
    """ 
    zoom = 2**(-1./dim)
    N = int(np.round(np.log2((max(delta0/delta_desired)**dim))))
    uniform_weights = normalize(np.ones(len(V0.particles_list)))
    # Main loop
    delta_prv = delta0
    V_prv = V0
    list_particles = []
    weights =[]
    for n in range(N):
        delta = delta_prv
        for i in range(6):
          if delta_prv[i] > delta_desired[i]: #we only update the delta element that is bigger than desired one
            delta[i] = zoom*delta_prv[i]
        tau = (max(delta/delta_desired))**2 # What tau should I use?
        # Sample new set of particles based on from previous region and M
        list_particles = PartialEvenDensityCover(V_prv,delta_desired,M)
        print "No. of particles of the ", n+1, " run: ", len(list_particles), "particles"
        # Compute normalized weights
        uniform_weights = normalize(np.ones(len(list_particles)))
        # print "uniform ",uniform_weights 
        weights = ComputeNormalizedWeights(list_particles, uniform_weights, D, tau, body,env)
        # print "weights after normalizing",  weights
        
        if visualize:
            VisualizeParticles(list_particles, weights, env, body)
        
        # for p in list_particles:
        #     print p.transformation()
        # Prune based on weights
        pruned_list_particles = Pruning(list_particles,weights)
        
        print 'No. of particles, after pruning:', len(pruned_list_particles)
        # raw_input("Press Enter to continue...")
        # Create a new region from the set of particle left after pruning
        V_prv = Region(pruned_list_particles,delta)
        delta_prv = delta
        print "delta_prv",  delta_prv
    new_set_of_particles = PartialEvenDensityCover(V_prv,delta_desired,M)
    print V_prv.delta
    uniform_weights = normalize(np.ones(len(new_set_of_particles)))
    new_weights = ComputeNormalizedWeights(new_set_of_particles, uniform_weights,D,1,body,env)
    return new_set_of_particles,new_weights

def PartialEvenDensityCover(region, desired_delta, M):
    '''Input: region - sampling region represented as a union of neighborhoods, M - number of particles to sample per neighborhood
    Output: a set of particles that evenly cover the region
    '''
    delta = region.delta
    list_particles = []
    for i  in range(len(region.particles_list)):
        center_particle = region.particles_list[i]
        #TODO: Count how many particles in list_particles are in the neighborhood of center_particle? Save as existing_p (if existing_p > M, error)
        num_existing_p = 0
        for p in list_particles:
            if InTheNeighborhoodRegion(p,center_particle,delta):
                num_existing_p += 1
        #Following loop is to sample and reject
        for m in range(M-num_existing_p):
            #TODO: Sample a particle in the neighborhood region.delta (sample in SE(3) using the parameter delta)
            ############## HOW TO SAMPLE!! It should be around the center which we are considering
            new_p = RandomParticleGivenDelta(delta,desired_delta,center_particle)
            #TODO: Check distances from the new particle to other regions (1 -> previous center_particle's region)
            accepted = True
            for k in range(i-1):
                previous_sphere_center = region.particles_list[k]
                if InTheNeighborhoodRegion(new_p,previous_sphere_center,delta):
                  accepted = False
                  break
            #TODO: if satified, add this particle to list_particles
            if accepted:
                list_particles.append(new_p)
    return list_particles

def InTheNeighborhoodRegion(particle, center_particle, delta):
  for i in range(3):
    if np.abs(particle.trans[i] - center_particle.trans[i]) > delta[i]:
      return False
    if np.abs(particle.euler[i] - center_particle.euler[i]) > delta[3+i]:
      return False
  return True

def RandomParticleGivenDelta(delta,desired_delta,center_particle):
  # Generate a random particle within a given region: ex: trans_region = [1,2,3] -> randvect = [rand(-1,1),rand(-2,2), rand(-3,3)]
  randtrans = np.zeros(3)
  randeuler = np.zeros(3)
  for i in range(3):
    randtrans[i] = np.random.uniform(-delta[i],delta[i])/1000.
    randeuler[i] = np.random.uniform(-delta[3+i],delta[3+i])
  return Particle(np.asarray(center_particle.trans) + randtrans, np.asarray(center_particle.euler)+randeuler)
