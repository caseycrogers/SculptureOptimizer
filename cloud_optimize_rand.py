from digifab import *
import math
import numpy
import random


# Writing bounds #
bounds= numpy.asarray([[10,10,25],[140,140,140]])


def cloud_optimize(cloud, edges):
  while(optimize(cloud, edges)):
    pass
  return

# Attempts ten times to improve the worst point in the cloud
# ALMOST DONE, BOUNDING BOX
def optimize(cloud, edges):
  # find the bounding box of the sculpture

  # finish

  i = 0
  while (i < 1500):
    print i

    before = hCloud(cloud, edges)
    #worstPT = findWorstPoint(cloud, edges)
    worstPT = random.randint(0, 5)
    #if worstPT == None:
    #  print "Sculpture is awesome!"
    #  return False

    originalPos = randomMove(cloud, worstPT, 75, bounds)
    
    # Failure, try again  
    if originalPos == None:
      print 'failed move'
      return False

    after = hCloud(cloud,edges)
    # Success!
    if after > before:
      print "CHANGED A POINT"
      return True
    # Failure, try again
    else:
      #print 'failed move: ' + str(after) + ": " + str(before)
      cloud[worstPT][0] = originalPos[0]
      cloud[worstPT][1] = originalPos[1]
      cloud[worstPT][2] = originalPos[2]

    i += 1
  # Out of tries!
  return False

# Returns a number for goodness of fit given a point, higher = better
# DONE
def hPoint(cloud, edges, index):
  return worst_angle(cloud, edges, index)

def randPoint(cloud, edges):
  badPts = []
  for i in range(len(cloud)):
    if joint_volume(cloud, edges, i) != 125.0:
      #print joint_volume(cloud, edges, i)
      badPts.append(i)
  if badPts == []:
    return None
  else:
    return random.choice(badPts)

    #perfectPts.add(worstPT)
    #if len(perfectPts) == len(cloud):
    #  return False
    #worstPT = randint(0,5)

# Returns a number for goodness of fit for the cloud, higher = better
# DONE
def hCloud(cloud, edges):
  worst = 0

  for i in range(len(cloud)):
    h = hPoint(cloud, edges, i)

    worst += h

  return worst

# Finds the point with the most acute diahedral angle
# DONE
def findWorstPoint(cloud, edges):
  worstPT = 0
  worstVal = None

  for i in range(len(cloud)):
    h = hPoint(cloud, edges, i)

    if worstPT == None:
      worstVal = h
      worstPT = i
    elif h < worstVal:
      worstVal = h
      worstPT = i

  return worstPT


# Moves the point within radius sphere randomly
# DONE
def randomMove(cloud, index, radius, bounds):
  def dist(a,b):
    return numpy.linalg.norm(pts_to_vec(a,b))

  def off_bounds(pts,bds):
    return (bds[0][0] <= pts[0] <= bds[1][0]) and (bds[0][1] <= pts[1] <= bds[1][1]) and (bds[0][2] <= pts[2] <= bds[1][2])

  t = 0
  while t < 1000:
    rand_vec = numpy.array([random.uniform(-radius, radius), random.uniform(-radius, radius), random.uniform(-radius, radius)]) # just to give a specific arbitrary direction
    #rand_radius = numpy.random.uniform(0.0, radius, 1)[0]
    #rand_norm = numpy.array([rand_vec[0]*rand_radius, rand_vec[1]*rand_radius, rand_vec[2]*rand_radius]) # normalize the vector to radius distance
  
    # Moving the point #
    mv_cloud = cloud[index].copy()
    og = cloud[index].copy()
    mv_cloud = mv_cloud + rand_vec

  
    failure = False
    for i in range(len(cloud)):
      if i != index and dist(mv_cloud,cloud[i]) < 30.0 or not off_bounds(mv_cloud, bounds):
        #cloud[i] = mv_cloud
        failure = True

    if not failure:
      cloud[index][0] = mv_cloud[0]
      cloud[index][1] = mv_cloud[1]
      cloud[index][2] = mv_cloud[2]
  
      return og
    t += 1

  return None

# Vector constructor #
# DONE
def pts_to_vec (pt_a, pt_b):
  start = numpy.asarray(pt_a)
  end = numpy.asarray(pt_b)
  vec = end - start
  return vec

# Find the diahedral angle given two vectors #
# DONE
def vectors_angle (vec_0, vec_1):
  dot_product = numpy.dot(vec_0, vec_1)
  vec_0_norm = numpy.linalg.norm(vec_0)
  vec_1_norm = numpy.linalg.norm(vec_1)
  return math.acos(dot_product / (vec_0_norm * vec_1_norm))

# Find the worst angle given a point # 
#DONE
def worst_angle(cloud, edges, p_i):
  neighbors_pts = neighbors(edges, p_i)
  vector_lst = []
  angle_lst = []
  
  for i in range(len(neighbors_pts)):
    vector_lst.append(pts_to_vec(cloud[p_i], cloud[neighbors_pts[i]]))
  
  prev = vector_lst[-1]
  for i in (range(len(vector_lst))):
    angle_lst.append(vectors_angle(vector_lst[i], prev))
    prev = vector_lst[i]

  return min(angle_lst)


def joint_volume(cloud, edges, p_i):
  # Calculating the angles of all pair of axes in a connector
  the_angle= worst_angle(cloud, edges, p_i)
  #print the_angle
  
  r = (3.0/2.0) + 0.4

  if round(math.sin(the_angle),2)== 0.0:
      radii = 5.0  
  else:
      radii= (r/(math.sin(the_angle)))
  
  max_radii= max(radii , 5.0)

  #print max_radii

  return max_radii**3

# Find the points a given point is connected to
# DONE
def neighbors(edges, index):
  n = []

  for edge in edges:
    if edge[0] == index:
      n.append(edge[1])
    elif edge[1] == index:
      n.append(edge[0])

  return n
