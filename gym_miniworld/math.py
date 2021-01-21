import math
import numpy as np

# X, Y, Z-axis vectors
X_VEC = np.array([1, 0, 0])
Y_VEC = np.array([0, 1, 0])
Z_VEC = np.array([0, 0, 1])

def gen_rot_matrix(axis, angle):
    """
    Rotation matrix for a counterclockwise rotation around a given axis
    The angle should be in radians
    """

    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(angle / 2.0)
    b, c, d = -axis * math.sin(angle / 2.0)

    return np.array([
        [a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c)],
        [2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b)],
        [2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c]
    ])

def intersect_circle_segs(point, radius, segs):
    """
    Test if a circle intersects with any wall segments
    """

    # Ignore Y coordinate
    px, _, pz = point
    point = np.array([px, 0, pz])

    a = segs[:, 0, :]
    b = segs[:, 1, :]
    ab = b - a
    ap = point - a

    dotAPAB = np.sum(ap * ab, axis=1)
    dotABAB = np.sum(ab * ab, axis=1)

    proj_dist = dotAPAB / dotABAB
    proj_dist = np.clip(proj_dist, 0, 1)
    proj_dist = np.expand_dims(proj_dist, axis=1)

    # Compute the closest point on the segment
    c = a + proj_dist * ab

    # Check if any distances are within the radius
    dist = np.linalg.norm(c - point, axis=1)
    dist_lt_rad = np.less(dist, radius)

    if np.any(dist_lt_rad):
        return True

    # No intersection
    return None

def intersect_wall_points(pos1,pos2,segs):
    """
    Test if there is a wall between two points
    """
    # Ignore Y coordinate
    px1, _, pz1 = pos1
    p1 = Point(px1,pz1)
    px2, _, pz2 = pos2
    p2 = Point(px2,pz2)

    for i in range(len(segs)):
        curSeg = segs[i]
        qx1, _, qz1 = segs[i,0,:]
        q1 = Point(qx1,qz1)
        qx2, _, qz2 = segs[i,1,:]
        q2 = Point(qx2,qz2)
        if doIntersect(p1, p2, q1, q2):
            return True
    return False

class Point: 
    def __init__(self, x, y): 
        self.x = x 
        self.y = y 

# Given three colinear points p, q, r, the function checks if  
# point q lies on line segment 'pr'  
def onSegment(p, q, r): 
    if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and 
           (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))): 
        return True
    return False
  
def orientation(p, q, r): 
    # to find the orientation of an ordered triplet (p,q,r) 
    # function returns the following values: 
    # 0 : Colinear points 
    # 1 : Clockwise points 
    # 2 : Counterclockwise 
      
    # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/  
    # for details of below formula.  
      
    val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y)) 
    if (val > 0): 
          
        # Clockwise orientation 
        return 1
    elif (val < 0): 
          
        # Counterclockwise orientation 
        return 2
    else: 
          
        # Colinear orientation 
        return 0
  
# The main function that returns true if  
# the line segment 'p1q1' and 'p2q2' intersect. 
def doIntersect(p1,q1,p2,q2): 
      
    # Find the 4 orientations required for  
    # the general and special cases 
    o1 = orientation(p1, q1, p2) 
    o2 = orientation(p1, q1, q2) 
    o3 = orientation(p2, q2, p1) 
    o4 = orientation(p2, q2, q1) 
  
    # General case 
    if ((o1 != o2) and (o3 != o4)): 
        return True
  
    # Special Cases 
  
    # p1 , q1 and p2 are colinear and p2 lies on segment p1q1 
    if ((o1 == 0) and onSegment(p1, p2, q1)): 
        return True
  
    # p1 , q1 and q2 are colinear and q2 lies on segment p1q1 
    if ((o2 == 0) and onSegment(p1, q2, q1)): 
        return True
  
    # p2 , q2 and p1 are colinear and p1 lies on segment p2q2 
    if ((o3 == 0) and onSegment(p2, p1, q2)): 
        return True
  
    # p2 , q2 and q1 are colinear and q1 lies on segment p2q2 
    if ((o4 == 0) and onSegment(p2, q1, q2)): 
        return True
  
    # If none of the cases 
    return False