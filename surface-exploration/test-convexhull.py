import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

import polygon

# Create testing data
# A 2D rectangle 2*5 cm

points = np.array([[0,0],[2,0],[0,5],[2,5]])
# hull = ConvexHull(points)

# plt.plot(points[:,0], points[:,1], 'o')
# for simplex in hull.simplices:
#     plt.plot(points[simplex, 0], points[simplex, 1], 'r-')

# plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'r--', lw=2)
# plt.plot(points[hull.vertices[0],0], points[hull.vertices[0],1], 'ro')
# plt.show()
new_point = np.array((3,4))
if polygon.in_hull(new_point,points):
    print "new point is inside the hull"
    plt.plot(new_point[0],new_point[1], 'or')
else:
    print "new point is NOT inside the hull"
    plt.plot(new_point[0],new_point[1], 'ok')

plt.plot(points[:,0], points[:,1], 'o')
hull = ConvexHull(points)
for simplex in hull.simplices:
    plt.plot(points[simplex, 0], points[simplex, 1], 'r-')


plt.show()
