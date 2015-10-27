import numpy as np

import Transformation
import Utils



class SimplePolygon(object):
    '''
    A very simple  polygon object, created for this test
    '''
    def __init__(self):
        p0 = np.array([0,0,0])
        p1 = np.array([1,0,0])
        p2 = np.array([0,1,0])
        p3 = np.array([0,0,1])
        self.vertices = [p0,p1,p2,p3]
        # normal = lambda x,y,z: np.cross(y-x,z-x)/np.linalg.norm(np.cross(y-x,z-x))
        self.indices = [[0,2,1],
                       [0,1,3],
                       [1,2,3],
                       [2,0,3]
                       ]

def particleTransform(par):
    '''Input: par = [x,y,t] vector of x,y translation and t rotation
    Output Transform Matrix
    '''
    from openravepy import matrixFromAxisAngle
    T = matrixFromAxisAngle([0,0,1],par[2])
    T[0,3] = par[0]
    T[1,3] = par[1]
    return T

def VisualizeParticles(list_particles,weights,env='',body =''):
    maxweight = 0
    for w in weights:
        if w > maxweight:
            maxweight = w
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

def generateParticles(N,bounds):
    '''Input: Number of particles
    Output: List of particles'states
    '''
    x_min,x_max = bounds[0]
    y_min,y_max = bounds[1]
    t_min,t_max = bounds[2]

    f = lambda r,x_min,x_max: r/1.0*(x_max-x_min)+x_min
    list_particles = []
    for i in range(N):
        x,y,t = np.random.rand(3)
        list_particles.append([f(x,x_min,x_max),f(y,y_min,y_max),f(t,t_min,t_max)])
    return list_particles

def inferFromMeasurements(particles,weight,measurements,body='',env=''):
    new_weight = np.zeros(len(weight))
    for i in range(len(particles)):
        p = particles[i]
        T = particleTransform(p)
        body.SetTransform(T)
        poly = env.Triangulate(body)
        total_energy = sum([calculateMahaDistancePolygon(poly,d)**2 for d in measurements])
        new_weight[i] = weight[i]*np.exp(-total_energy)
    return normalize(new_weight)


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
        self.trans =  trans
        self.euler = euler
    
    def rotation(self):
        rot = Transformation.euler_matrix(self.euler[0],self.euler[1],self.euler[2],axes='sxyz')[:3,:3]
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
    for i  in range(len(region.particles_list)):
        center_particle = region.particles_list[i]
        #TODO: Count how many particles in list_particles are in the sphere of center_particle? Save as existing_p (if existing_p > M, error)
        num_existing_p = 0
        for p in list_particles:
            distanceSE3 = Utils.SE3Distance(p.transformation(),center_particle.transformation(),1.0,1.0*1000) ############RECHECK this distance
            if distanceSE3 < region.delta:
                num_existing_p += 1
        #Following loop is to sample and reject
        for m in range(M-num_existing_p):
            #TODO: Sample a particle in the sphere region.delta (sample in SE(3) using the parameter delta)
            new_trans = np.random.rand(3)*region.delta/1000
            new_euler = np.random.rand(3)*region.delta                                                        ############## HOW TO SAMPLE!! this may not be correct!
            new_p = Particle(new_trans, new_euler)
            #TODO: Check distances from the new particle to other spheres (1 -> previous center_particle's sphere (sphere here is jst a word to visualize the idea, proper word should be neighborhood)
            accepted = True
            for k in range(i-1):
                previous_sphere_center = region.particles_list[k]
                distance_2_previous_sphere_center = Utils.SE3Distance(new_p.transformation(),previous_sphere_center.transformation(),1.0,1.0*1000)
                if distance_2_previous_sphere_center < region.delta:
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
    print "Weights before normalization", new_weights
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
    threshold = 0.1*maxweight
    for i in range(len(list_particles)):
        if weight[i] > threshold:
            pruned_list.append(list_particles[i])
    return pruned_list

def ScalingSeries(V0, D, M, delta0, delta_desired, body = '', env ='', dim = 6):
    """
    @type  V0:  ParticleFilterLib.Region
    @param V0:  initial uncertainty region
    @param  D:  a list of measurements [p,n,o_n,o_p]
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
    for n in range(N):
        delta = zoom*delta_prv
        tau = (delta/delta_desired)**2
        # Sample new set of particles based on from previous region and M
        list_particles = EvenDensityCover(V_prv,M)
        print "No. of particles of the ", n+1, " run: ", len(list_particles), "particles"
        # Compute normalized weights
        uniform_weights = normalize(np.ones(len(list_particles)))
        weights = ComputeNormalizedWeights(list_particles, uniform_weights, D, tau, body=body,env=env)
        # raw_input("Press Enter to continue...")
        # for p in list_particles:
        #     print p.transformation()
        # print weights
        # Prune based on weights
        pruned_list_particles = Pruning(list_particles,weights)
        
        print 'No. of particles, after pruning:', len(pruned_list_particles)
        # Create a new region from the set of particle left after pruning
        V_prv = Region(pruned_list_particles,delta)
        delta_prv = delta
        print "delta_prv",  delta_prv
    new_set_of_particles = EvenDensityCover(V_prv,M)
    uniform_weights = normalize(np.ones(len(new_set_of_particles)))
    new_weights = ComputeNormalizedWeights(new_set_of_particles, uniform_weights,D,1,body=body,env=env)
    return new_set_of_particles,new_weights



# # Create data

# dim = 6. # 6 DOFs
# delta0 = 5.
# zoom = 2**(-1./dim)
# ptcl0 = ParticleFilterLib.Particle([0,0,0],[0,0,0]) 
# V0 = ParticleFilterLib.Region(ptcl0, delta0)

# M = 6 # No. of particles per delta-neighbohood

# delta_desired = 1. # Terminal value of delta
# N = np.log2((delta/delta_desired)**dim)

# # Main loop
# delta_prv = delta0
# V_prv = V0
# for n in range(N):
#     delta = zoom*delta_prv
#     tau = (delta/delta_desired)**2
#     # Sample new set of particles based on from previous region and M
#     list_particles = ParticleFilterLib.EvenDensityCover(V_prv,M)
#     print "No. of particles of the ", n+1, " run: ", len(list_particles), "particles"
#     # Compute normalized weights
    
#     # Prune based on weights

#     # Create a new region from the set of particle left after pruning
