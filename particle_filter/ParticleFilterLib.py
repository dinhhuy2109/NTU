__author__ = 'hung'

import numpy as np

class Particle(object):

    def __init__(self):
        raise(NotImplemented)


def calculateMahaDistanceMesh(trimesh,d):
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

def calculateMahaDistancePolygon(poly,d):
    '''
    :param  poly:       A trimesh object
    :param  d   :       A measurement [p,n,o_n,o_p]
    '''
    dis = [calculateMahaDistanceMesh([poly.vertices[x],poly.vertices[y],poly.vertices[z]],d) for x,y,z in poly.indices]
    return min(dis)

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

def visualizeParticles(particles,weight,env='',body =''):
    weight_threshold = 1e-5
    from openravepy import RaveCreateKinBody
    with env:
        env.Reset()
        newbodies = []
        for i in range(len(particles)):
            if weight[i] > weight_threshold:
                p = particles[i]
                transparency = (1-weight[i])
                trans = particleTransform(p)
                newbody = RaveCreateKinBody(env,body.GetXMLId())
                newbody.Clone(body,0)
                newbody.SetName(body.GetName())
                for link in newbody.GetLinks():
                    for geom in link.GetGeometries():
                        geom.SetTransparency(transparency)
                env.AddKinBody(newbody,True)
                with env:
                    newbody.SetTransform(trans)

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

def normalize(weight):
    norm_weight = np.zeros(len(weight))
    sum_weight = np.sum(weight)
    for i in range(len(weight)):
        norm_weight[i] = weight[i]/sum_weight
    return norm_weight

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
