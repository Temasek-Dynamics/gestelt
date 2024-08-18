# solid geometry
# this file is to do some calculation of solid geometry to do the collision detection of quadrotor
# this file consists of several classes
import numpy as np
import casadi
import torch
## return the maginitude of a vector
def magni(vector):
    return np.sqrt(np.dot(np.array(vector),np.array(vector)))

def magni_casadi(vector):
    return casadi.norm_2(vector)

## return the unit vector of a vector
def norm(vector):
    return np.array(vector)/magni(np.array(vector))


## define a class of a plane (using three points on the plane)
class plane():
    def __init__(self, point1, point2, point3):

        # point1 is the centroid of the gate
        self.point1 = np.array(point1)
        self.point2 = np.array(point2)
        self.point3 = np.array(point3)
        self.vec1 = self.point2 - self.point1
        self.vec2 = self.point3 - self.point1
        self.normal = norm(np.cross(self.vec2,self.vec1))
    
    # normal vector of the plane 
    def nor_vec(self, ):
        return self.normal

    # normal vector of one side 
    def n1(self):
        return norm(np.cross(self.vec1,self.normal))

    # normal vector of one side 
    def n2(self):
        return norm(np.cross(self.normal,self.vec2))

    # normal vector of one side 
    def n3(self):
        self.vec3 = self.point3 - self.point2
        return norm(np.cross(self.normal,self.vec3))
    
    ## intersection with another line 
    def interpoint(self, point1, point2):
        dir = norm(np.array(point1)-np.array(point2))
        t = 1/(np.dot(dir,self.normal))*(np.dot(self.normal,np.array(point1)-self.point1))
        point = np.array(point1) - t * dir
        return point

## define a class of a line
class line():
    def __init__(self, point1, point2):
        self.point1 = np.array(point1)
        self.point2 = np.array(point2)
        self.dir = norm(self.point1 - self.point2)

    ## return the distance from a point to the line
    def vertical(self, point):
        point3 = np.array(point)
        normal = np.cross(point3 - self.point1, self.dir)
        return magni(normal)

    ## return the distance from a point to the line section
    def distance(self,point):
        a = self.vertical(point)
        b = magni(point-self.point1)
        c = magni(point-self.point2)
        d = magni(self.point1-self.point2)
        if(b>c):
            if((b**2-d**2)>a**2):
                dis = c
            else:
                dis = a
        else:
            if((c**2-d**2)>a**2):
                dis = b
            else:
                dis = a
        return dis

## define the narrow window which is also the obstacle for the quadrotor
class obstacle():
    def __init__(self, point1, point2, point3, point4):
        self.point1 = np.array(point1)
        self.point2 = np.array(point2)
        self.point3 = np.array(point3)
        self.point4 = np.array(point4)

        #define the centroid of obstacle
        self.centroid = np.array([(point1[0]+point2[0]+point3[0]+point4[0])/4,\
            (point1[1]+point2[1]+point3[1]+point4[1])/4,(point1[2]+point2[2]+point3[2]+point4[2])/4])
        
        #define the cross section
        self.plane1 = plane(self.centroid,point1,point2)
        self.plane2 = plane(self.centroid,point2,point3)
        self.plane3 = plane(self.centroid,point3,point4)
        self.plane4 = plane(self.centroid,point4,point1)

        #define the bound
        self.line1 = line(point1, point2)
        self.line2 = line(point2, point3)
        self.line3 = line(point3, point4)
        self.line4 = line(point4, point1)

    def collis_det(self, vert_traj, horizon):
        """
        detect collision for each corner of the quadrotor
        """
        ## define the state whether find corresponding plane
        collision = 0
        self.co = 0

        PASS_GATE_PLANE = False
   
        ## judge whether the first plane is the traversal plane
        # find two points of traverse
        d_min = 0.2
        for t in range(horizon):
            if(np.dot(self.plane1.nor_vec(),vert_traj[t]-self.centroid)<0):
                PASS_GATE_PLANE = True
                # intersect is s_iw
                intersect = self.plane1.interpoint(vert_traj[t],vert_traj[t-1])
                
                
                # judge whether they belong to plane1 and calculate the distance
                if(np.dot(self.plane1.n1(),intersect-self.centroid)>0 and np.dot(self.plane1.n2(),intersect-self.centroid)>0):
                    
                    if(np.dot(self.point1-intersect,self.plane1.n3())>0):

                        # m is the d_i
                        m = min(self.line1.vertical(intersect),self.line2.vertical(intersect),\
                            self.line3.vertical(intersect),self.line4.vertical(intersect))
                        collision = - max(0,d_min-m)**2
                        self.co = 1
                    else:
                        m = min(self.line4.distance(intersect),self.line1.distance(intersect),self.line2.distance(intersect))
                        collision =   - 2*d_min*m - d_min**2
                

       
                # judge whether the intersection belongs to plane2 and calculate the distance  
                if(np.inner(self.plane2.n1(),intersect-self.centroid)>0 and np.inner(self.plane2.n2(),intersect-self.centroid)>0):
                    if(np.dot(self.point2-intersect,self.plane2.n3())>0):
                        m = min(self.line1.vertical(intersect),self.line2.vertical(intersect),\
                            self.line3.vertical(intersect),self.line4.vertical(intersect))
                        collision = - max(0,d_min-m)**2
                        self.co = 1
                    else:
                        m = min(self.line1.distance(intersect),self.line2.distance(intersect),self.line3.distance(intersect))
                        collision =   - 2*d_min*m - d_min**2
                    

                # judge whether the intersection belongs to plane3 and calculate the distance
                if(np.inner(self.plane3.n1(),intersect-self.centroid)>0 and np.inner(self.plane3.n2(),intersect-self.centroid)>0):
                    if(np.dot(self.point3-intersect,self.plane3.n3())>0):
                        m = min(self.line1.vertical(intersect),self.line2.vertical(intersect),\
                            self.line3.vertical(intersect),self.line4.vertical(intersect))
                        collision = - max(0,d_min-m)**2
                        self.co = 1
                    else:
                        m = min(self.line2.distance(intersect),self.line3.distance(intersect),self.line4.distance(intersect))
                        collision =   - 2*d_min*m - d_min**2
                    

                # judge whether the intersection belongs to plane4 and calculate the distance
                if(np.inner(self.plane4.n1(),intersect-self.centroid)>0 and np.inner(self.plane4.n2(),intersect-self.centroid)>0):
                    if(np.dot(self.point4-intersect,self.plane4.n3())>0):
                        m = min(self.line1.vertical(intersect),self.line2.vertical(intersect),\
                            self.line3.vertical(intersect),self.line4.vertical(intersect))
                        collision = - max(0,d_min-m)**2
                        self.co = 1
                    else:
                        m = min(self.line3.distance(intersect),self.line4.distance(intersect),self.line1.distance(intersect))
                        collision =   - 2*d_min*m - d_min**2
                break
        
        if not PASS_GATE_PLANE:
            # means the solve failed, avoid this situation
            collision = 0   
                  
        return collision

############################################################################################################
### support torch auto differentiation ####
def norm_torch(tensor):
    return tensor/torch.norm(tensor)

def magni_torch(tensor):
    return torch.norm(tensor)

# Define a class for a plane using three points on the plane
class Plane_torch:
    def __init__(self, point1, point2, point3):
        # point1 is the centroid of the gate
        # self.point1 = torch.tensor(point1,  requires_grad=True)
        # self.point2 = torch.tensor(point2,  requires_grad=True)
        # self.point3 = torch.tensor(point3,  requires_grad=True)
        self.point1=point1
        self.point2=point2
        self.point3=point3
        self.vec1 = self.point2 - self.point1
        self.vec2 = self.point3 - self.point1
        self.normal = norm_torch(torch.cross(self.vec2, self.vec1))

    # Normal vector of the plane
    def nor_vec_torch(self):
        return self.normal

    # Normal vector of one side
    def n1_torch(self):
        return norm_torch(torch.cross(self.vec1, self.normal))

    # Normal vector of one side
    def n2_torch(self):
        return norm_torch(torch.cross(self.normal, self.vec2))

    # Normal vector of one side
    def n3_torch(self):
        self.vec3 = self.point3 - self.point2
        return norm_torch(torch.cross(self.normal, self.vec3))
    
    # Intersection with another line
    def interpoint_torch(self, point1, point2):

        dir = norm_torch(point1 - point2)
        t = torch.dot(self.normal, point1 - self.point1) / torch.dot(dir, self.normal)
        point = point1 - t * dir
        return point

# Define a class for a line
class Line_torch:
    def __init__(self, point1, point2):
        self.point1 = point1
        self.point2 = point2
        self.dir = norm_torch(self.point1 - self.point2)

    # Return the distance from a point to the line
    def vertical_torch(self, point):
        point3 = point
        normal = torch.cross(point3 - self.point1, self.dir)
        return magni_torch(normal)

    # Return the distance from a point to the line segment
    def distance_torch(self, point):
        point = point
        a = self.vertical_torch(point)
        b = magni_torch(point - self.point1)
        c = magni_torch(point - self.point2)
        d = magni_torch(self.point1 - self.point2)
        if b > c:
            if (b**2 - d**2) > a**2:
                dis = c
            else:
                dis = a
        else:
            if (c**2 - d**2) > a**2:
                dis = b
            else:
                dis = a
        return dis

# Define the narrow window which is also the obstacle for the quadrotor
class obstacle_torch:
    def __init__(self, point1, point2, point3, point4):
        self.point1 = torch.tensor(point1,  requires_grad=True)
        self.point2 = torch.tensor(point2,  requires_grad=True)
        self.point3 = torch.tensor(point3,  requires_grad=True)
        self.point4 = torch.tensor(point4,  requires_grad=True)

        # Define the centroid of the obstacle
        self.centroid = torch.tensor([(self.point1[0] + self.point2[0] + self.point3[0] + self.point4[0]) / 4,
                                      (self.point1[1] + self.point2[1] + self.point3[1] + self.point4[1]) / 4,
                                      (self.point1[2] + self.point2[2] + self.point3[2] + self.point4[2]) / 4],
                                       requires_grad=True)

        # Define the cross sections
        self.plane1_torch = Plane_torch(self.centroid, self.point1, self.point2)
        self.plane2_torch = Plane_torch(self.centroid, self.point2, self.point3)
        self.plane3_torch = Plane_torch(self.centroid, self.point3, self.point4)
        self.plane4_torch = Plane_torch(self.centroid, self.point4, self.point1)

        # Define the bounds
        self.line1_torch = Line_torch(self.point1, self.point2)
        self.line2_torch = Line_torch(self.point2, self.point3)
        self.line3_torch = Line_torch(self.point3, self.point4)
        self.line4_torch = Line_torch(self.point4, self.point1)

    def collis_det_torch(self, vert_traj, horizon):
        """
        Detect collision for each corner of the quadrotor
        """
        # Define the state whether the corresponding plane is found
        collision = torch.tensor(0.0, requires_grad=True)
        self.co = 0
        PASS_GATE_PLANE = False
        d_min = 0.2

        for t in range(horizon):
            if torch.dot(self.plane1_torch.nor_vec_torch(), vert_traj[t] - self.centroid) < 0:
                PASS_GATE_PLANE = True
                intersect = self.plane1_torch.interpoint_torch(vert_traj[t], vert_traj[t-1])

                # Judge whether they belong to plane1 and calculate the distance
                if torch.dot(self.plane1_torch.n1_torch(), intersect - self.centroid) > 0 and torch.dot(self.plane1_torch.n2_torch(), intersect - self.centroid) > 0:
                    if torch.dot(self.point1 - intersect, self.plane1_torch.n3_torch()) > 0:
                        # m = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect),
                        #               self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        min_12 = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect))
                        min_34 = torch.min(self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        m = torch.min(min_12, min_34)
                        collision = - torch.max(torch.tensor(0.0), d_min - m)**2
                        self.co = 1
                    else:
                        min_41= torch.min(self.line4_torch.distance_torch(intersect), self.line1_torch.distance_torch(intersect))
                        m= torch.min(min_41, self.line2_torch.distance_torch(intersect))
                        # m = torch.min(self.line4_torch.distance_torch(intersect), self.line1_torch.distance_torch(intersect), self.line2_torch.distance_torch(intersect))
                        collision = - 2 * d_min * m - d_min**2

                # Judge whether the intersection belongs to plane2 and calculate the distance
                if torch.dot(self.plane2_torch.n1_torch(), intersect - self.centroid) > 0 and torch.dot(self.plane2_torch.n2_torch(), intersect - self.centroid) > 0:
                    if torch.dot(self.point2 - intersect, self.plane2_torch.n3_torch()) > 0:
                        # m = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect),
                        #               self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        min_12 = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect))
                        min_34 = torch.min(self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        m = torch.min(min_12, min_34)
                        collision = - torch.max(torch.tensor(0.0), d_min - m)**2
                        self.co = 1
                    else:
                        min_12 = torch.min(self.line1_torch.distance_torch(intersect), self.line2_torch.distance_torch(intersect))
                        m = torch.min(min_12, self.line3_torch.distance_torch(intersect))
                        # m = torch.min(self.line1_torch.distance_torch(intersect), self.line2_torch.distance_torch(intersect), self.line3_torch.distance_torch(intersect))
                        collision = - 2 * d_min * m - d_min**2

                # Judge whether the intersection belongs to plane3 and calculate the distance
                if torch.dot(self.plane3_torch.n1_torch(), intersect - self.centroid) > 0 and torch.dot(self.plane3_torch.n2_torch(), intersect - self.centroid) > 0:
                    if torch.dot(self.point3 - intersect, self.plane3_torch.n3_torch()) > 0:
                        # m = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect),
                        #               self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        min_12 = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect))
                        min_34 = torch.min(self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        m = torch.min(min_12, min_34)
                        collision = - torch.max(torch.tensor(0.0), d_min - m)**2
                        self.co = 1
                    else:
                        min_23 = torch.min(self.line2_torch.distance_torch(intersect), self.line3_torch.distance_torch(intersect))
                        m = torch.min(min_23, self.line4_torch.distance_torch(intersect))
                        # m = torch.min(self.line2_torch.distance_torch(intersect), self.line3_torch.distance_torch(intersect), self.line4_torch.distance_torch(intersect))
                        collision = - 2 * d_min * m - d_min**2

                # Judge whether the intersection belongs to plane4 and calculate the distance
                if torch.dot(self.plane4_torch.n1_torch(), intersect - self.centroid) > 0 and torch.dot(self.plane4_torch.n2_torch(), intersect - self.centroid) > 0:
                    if torch.dot(self.point4 - intersect, self.plane4_torch.n3_torch()) > 0:
                        # m = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect),
                        #               self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        min_12 = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect))
                        min_34 = torch.min(self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        m = torch.min(min_12, min_34)
                        collision = - torch.max(torch.tensor(0.0), d_min - m)**2
                        self.co = 1
                    else:
                        min_34 = torch.min(self.line3_torch.distance_torch(intersect), self.line4_torch.distance_torch(intersect))
                        m = torch.min(min_34, self.line1_torch.distance_torch(intersect))
                        # m = torch.min(self.line3_torch.distance_torch(intersect), self.line4_torch.distance_torch(intersect), self.line1_torch.distance_torch(intersect))
                        collision = - 2 * d_min * m - d_min**2
                break

        if not PASS_GATE_PLANE:
            # Means the solve failed, avoid this situation
            collision = torch.tensor(0.0, requires_grad=True)

        return collision