import numpy  as np
import SE3UncertaintyLib as SE3lib
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

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
                    transf = p
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
        acum_tf = np.zeros((4,4))
        weight_threshold = 0.7*maxweight
        for i in range(len(list_particles)):
            if weights[i] > weight_threshold:
                p = list_particles[i]
                acum_tf += p*weights[i]
                acum_weight += weights[i]
        estimated_particle = acum_tf*(1/acum_weight)
        transf = estimated_particle
        print "Resulting estimation:\n", transf
        body.SetTransform(transf)
        env.AddKinBody(body,True)
        return transf

class Region(object):
    def __init__(self, particles_list, sigma):
        self.particles_list = particles_list #List of particles (transformations)
        self.sigma = sigma
        
def EvenDensityCover(region, M):
    '''Input: region - sampling region represented as a union of neighborhoods, M - number of particles to sample per neighborhood
    Output: a set of particles that evenly cover the region
    '''
    list_particles = []

    for i  in range(len(region.particles_list)):
        center_particle = region.particles_list[i]
        sigma = region.sigma
        cholsigma = np.linalg.cholesky(sigma).T
        num_existing_p = 0
        for p in list_particles:
            if SE3lib.isinside(SE3lib.tran2vec(p), SE3lib.tran2vec(center_particle), sigma):
                num_existing_p += 1
        for m in range(M-num_existing_p):
            #TODO: Sample a particle in the sigma region
            notinside = True;
            while notinside:
                uniformsample = np.random.uniform(-1,1,size = 6)
                xisample = np.dot(cholsigma, uniformsample)
                if SE3lib.isinside(xisample, np.zeros(6), sigma):
                    Tsample = SE3lib.vec2tran(xisample)
                    new_p = np.dot(center_particle,Tsample)
                    notinside = False
            #TODO: Check distances from the new particle to other sigma neighborhoods (other center particle's sigma region)
            accepted = True
            for k in range(i-1):
                previous_center = region.particles_list[k]
                if SE3lib.isinside(SE3lib.tran2vec(new_p), SE3lib.tran2vec(previous_center),sigma):
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
        T = p
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

def Pruning(list_particles, weights):
    assert (len(list_particles)==len(weights)),"Wrong input data, length of list of particles are not equal to length of weight"
    pruned_list = []
    maxweight = 0
    for w in weights:
        if w > maxweight:
            maxweight = w
    threshold = 0.7*maxweight
    for i in range(len(list_particles)):
        if weights[i] > threshold:
            pruned_list.append(list_particles[i])
    return pruned_list

def ScalingSeries(V0, D, M, sigma0, sigma_desired, dim = 6, body = '', env ='', visualize = False):
    """
    @type  V0:  ParticleFilterLib.Region
    @param V0:  initial uncertainty region
    @param  D:  a list of measurements [p,n,o_n,o_p] p is the contacted point, n is the approaching vector (opposite to normal)
    @param  M:  the no. of particles per neighborhood
    @param delta_desired: terminal value of delta
    @param dim: dimension of the state space (6 DOFs)
    """ 
    zoom = 2**(-1./dim)
    R, s , RT = np.linalg.svd(sigma0)
    Rd,sd, RTd = np.linalg.svd(sigma_desired)
    nr = np.linalg.norm(s)
    nr_desired = np.linalg.norm(sd)
    N = int(np.round(np.log2((nr/nr_desired)**dim))) ############################
    print N 
    uniform_weights = normalize(np.ones(len(V0.particles_list)))
    
    sigma_prv = sigma0
    V_prv = V0
    list_particles = []
    weights = []
    # Main loop
    for n in range(N):
        sigma = sigma_prv*zoom
        tau = 1.0     ##########################################
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
        V_prv = Region(pruned_list_particles,sigma)
        sigma_prv = sigma
        print "delta_prv",  sigma
    new_set_of_particles = EvenDensityCover(V_prv,M)
    print V_prv.sigma
    uniform_weights = normalize(np.ones(len(new_set_of_particles)))
    new_weights = ComputeNormalizedWeights(new_set_of_particles, uniform_weights,D,1.0,body,env)
    return new_set_of_particles,new_weights
