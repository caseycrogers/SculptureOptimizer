
#!/usr/bin/python

from digifab import *
import math
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import Voronoi, Delaunay
from cloud_optimize_rand import cloud_optimize

# EDIT -  Luis #
# Helper function to calculate the diahedral angle between two angles #
# Fixed #
def vectors_angle (vec_0, vec_1):
  dot_product = numpy.dot(vec_0, vec_1)
  vec_0_norm = numpy.linalg.norm(vec_0)
  vec_1_norm = numpy.linalg.norm(vec_1)
  return math.acos(dot_product / (vec_0_norm * vec_1_norm))

# EDIT - Luis 
# Helper function to be included in the connector class - Check with Casey #
# Fixed #
def height_connector(stick_diameter, axes):
  # Calculating the angles of all pair of axes in a connector
  axes_angles=[]
  prev = axes[-1]
  for i in (range(len(axes))):
    axes_angles.append(vectors_angle(axes[i], prev))
    prev = axes[i]

  # Selecting the minimum angle of axes_angles
  the_angle = (min(axes_angles))/2.0

  r = (stick_diameter/2.0) + 0.4

  if round(math.sin(the_angle),2)== 0:
      radii = 6.0  
  else:
      radii= (r/(math.sin(the_angle)))
  return max(radii , 6.0)

class Stick(Body):

  STICK_COUNT = 0
  
  def __init__(self, length=50.0, diameter= 3.0, **kwargs):
    """Make a 3mm diameter cylinder of specified length."""

    if 'name' not in kwargs.keys():
      kwargs['name'] = 'stick_%d' % Stick.STICK_COUNT
      Stick.STICK_COUNT = Stick.STICK_COUNT + 1

    self.length = length
    
    #if 'joints' not in kwargs.keys():
      #kwargs['joints'] = [ORIGIN_POSE, ((0,0,length),ORIGIN_POSE[1])]
    
    if 'layers' not in kwargs.keys():
      kwargs['layers'] = Layer(
        PolyMesh(generator=solid.cylinder(diameter/2.0,length)),
        name='stick',color='white')
    
    super(Stick, self).__init__(**kwargs)

  def setLength(self, length):
    self.length = length
    self.elts = [Layer(PolyMesh(generator=solid.cylinder(1.5,length)),
      name='stick',color='white')]

  def guide(self):
    """Return guide PolyLine with name label for this stick."""

    bar_pl = PolyLine([[0,-2],[0,2]])

    stick_pl = PolyLine([[0,0],[self.length,0]])
    
    stick_num = self.name.split('_')[-1]
    label_pl = (2,-2,0) * PolyLine(generator=solid.text(stick_num, 4))
    
    len_pose = (self.length, 0, 0)

    guide_pl = bar_pl + stick_pl + len_pose * (bar_pl + label_pl)

    return guide_pl

class Connector(Body):
  
  CONNECTOR_COUNT = 0

  def __init__(self, axes=None, **kwargs):
    """Make a connector with specified connection axes.
    
    Args:
      axes [[float]]: List of unit vectors in directions of connections
        to sticks.
    """

    if axes is None:
      axes = []

    self.axes = axes

    #print len(axes) # just for sanity check

    if 'name' not in kwargs.keys():
      kwargs['name'] = 'connector_%d' % Connector.CONNECTOR_COUNT
      Connector.CONNECTOR_COUNT = Connector.CONNECTOR_COUNT + 1
    
    ## CASEY ##
    radius = height_connector(3.0, axes.values())
    sphere_radius = max(radius - 2.5, 6)# 

    # make a basic vertical mortis with text
    mortise = solid.translate([0,0,-3])(solid.cylinder(3.7, 9))
    cut = PolyMesh(generator=solid.cylinder(1.7,7))
    mortise_pl= PolyMesh(generator = mortise) # transforming the mortise into a PolyMesh object
    mortise_pl *= translation_matrix([0,0,radius])
    cut *= translation_matrix([0,0,radius])

    # for each stick coming into connector, rotate and translate a mortis and union it with the connector
    temp = PolyMesh(generator=solid.sphere(r = radius))
    cutTemp = PolyMesh()
    for stick in axes.keys():
      axis = axes[stick]
      name = stick[6:]
      a_text = solid.rotate(a=90, v=[1, 0, 0])((solid.linear_extrude(height = 1)(
        solid.text(name, size = 2.5, halign="center", valign="center"))))#fixed - first rotate
      a_text = solid.translate([0,-3,2.5])(a_text)
      a_text_pl = PolyMesh (generator = a_text)
      a_text_pl *= translation_matrix([0,0,radius])
      text_mortise = mortise_pl.unioned(a_text_pl)

      rot_angle = numpy.arccos(axis[2])
      rot_axis = numpy.cross([0,0,1],axis)

      if axis[2] == 1.0 and len(axes) > 2:
        temp = temp.unioned(text_mortise)
        cutTemp = cutTemp.unioned(cut)
      elif axis[2] == -1.0 and len(axes) > 2:
        rot_axis = [1,0,0]
        rot_angle= numpy.pi 
        cut2 = cut.clone()
        cut2 *= rotation_matrix(rot_angle, rot_axis, [0,0,0])
        cutTemp = cutTemp.unioned(cut2)
        temp2 = text_mortise.clone()
        temp2 *= rotation_matrix(rot_angle, rot_axis, [0,0,0])
        temp = temp.unioned(temp2)
      elif len(axes) > 2:
        cut2 = cut.clone()
        cut2 *= rotation_matrix(rot_angle, rot_axis, [0,0,0])
        cutTemp = cutTemp.unioned(cut2)
        temp2 = text_mortise.clone()
        temp2 *= rotation_matrix(rot_angle, rot_axis, [0,0,0])
        temp = temp.unioned(temp2)
    
    if len(axes)== 2: # all fixed Luis
      temp = temp.unioned(text_mortise)
      cutTemp = cutTemp.unioned(cut)
      small_box = solid.cube([9,9,3.0], center = True)
      small_box = solid.translate([0,0,-3.0/2.0]) (small_box) 
      big_box = solid.cube(12, center = True)
      big_box = solid.translate([0,0,-6])(big_box)
      small_box_pm = PolyMesh(generator = small_box)
      big_box_pm = PolyMesh(generator = big_box)
      temp = temp.differenced(big_box_pm)
      temp = temp.unioned(small_box_pm)

    temp = temp.differenced(cutTemp)
    #self.elts = [Layer(geometries= temp, name='print', color='blue')] # not necessary

    if 'layers' not in kwargs.keys():
      kwargs['layers'] = Layer(temp, name='print',color='blue')

    super(Connector, self).__init__(**kwargs)

class Plate(Body):
  
  PLATE_COUNT = 0

  def __init__(self, base_pts=None, **kwargs):
    """Make a support plate with specified connection holes at base_pts
    
    Args:
      base_pts [[float]]: list of 3D coordinates of support connectors (the
        z coordinate will be 0).
    """

    if base_pts is None:
      base_pts = []
    
    self.base_pts = base_pts


    if 'name' not in kwargs.keys():
      kwargs['name'] = 'plate_%d' % Plate.PLATE_COUNT
      Plate.PLATE_COUNT = Plate.PLATE_COUNT + 1
    
    if 'layers' not in kwargs.keys():
      gen = solid.translate([0,0,-3])(solid.cube([150,150,3]))# check it out the 3 height
      gen_pm = PolyMesh(generator = gen)
      for pt in base_pts:
        a_cube = solid.translate([pt[0] - 4.5 , pt[1] - 4.5 , -6])(solid.cube(9)) # fixed
        a_cube_pm = PolyMesh(generator= a_cube)
        gen_pm = gen_pm.differenced(a_cube_pm)
        
      kwargs['layers'] = Layer(gen_pm, name = 'cut', color='red')

    super(Plate, self).__init__(**kwargs)
  
  def cut(self):
    """Return cut geometry for this support plate."""

    return PolyLine(
      generator=solid.projection(cut=True)(self[0][0].get_generator())
    )

def make_sparse_cloud(n_points = 6, scale=150.0):
  """Return a 3D point cloud with minimum distance between points."""

  cloud = scale * numpy.random.rand(1,3)

  while cloud.shape[0] < n_points:
    new_pt = scale * numpy.random.rand(1,3)
    dists = (((cloud - new_pt)**2).sum(axis=1))**0.5
    if all(dists > scale/5.0):
      cloud = numpy.vstack([cloud,new_pt])

  return cloud + [0,0,50.0]

class Sculpture(Layout):
  def __init__(self, cloud = None, **kwargs):
    if cloud is None:
      cloud = make_sparse_cloud()

    self.cloud = cloud
    
    self.edges = []
    
    delaunay = Delaunay(self.cloud)
    
    for simplex in delaunay.simplices:
      sl = simplex.tolist()
      simplex_edges = []
      for i in range(len(sl)):
        for j in range(i+1,len(sl)):
          simplex_edges.append((sl[i],sl[j]))
      for edge in simplex_edges:
        if edge not in self.edges and edge[::-1] not in self.edges:
          self.edges.append(edge)

    print self.cloud
    print "Hello World"
    #print self.cloud[0]
    cloud_optimize(self.cloud,self.edges)
    print self.cloud


    min_z = self.cloud[:,2].argsort()[0:3]
    base_pts = self.cloud[min_z,:]
    base_pts[:,2] = 0.0

    self.edges.extend([(self.cloud.shape[0]+i,min_z[i]) for i in range(3)])
    self.cloud = numpy.vstack([self.cloud,base_pts])
    
    if 'blocks' not in kwargs.keys():
      blocks = []
      connector_axes = [dict() for i in range(self.cloud.shape[0])]

      stickList = {}

      # Iterate through edges, create sticks for each, and record axes for connector
      for edge in self.edges:
        pt0, pt1 = self.cloud[edge,:]
        vector = pt1 - pt0
        length = ((vector)**2).sum()**0.5 
        axis = vector/length
        
        # If the current axis is very nearly vertical, just coerce to vertical to
        # avoid numerical instability problems
        if (1.0-abs(axis[2])) < 0.001:
          if axis[2] > 0:
            quat = ORIGIN_POSE[1]
          else:
            quat = NZ_JOINT_POSE[1]

        # Otherwise, use cross product to find axis of rotation to get z axis
        # aligned with stick direction, and dot product to find magnitude of
        # rotation angle
        else:
          rot_axis = numpy.cross([0,0,1],axis)
          rot_angle = numpy.arccos(axis[2])
          quat = quaternion_about_axis(rot_angle, rot_axis)
        
        # EDIT - Luis + Casey #   
        # Add Sticks

        new_stick = Stick(length, pose =(pt0, quat)) 
        blocks.append(new_stick)

        stickList[new_stick.name] = new_stick

        # Update connector axes list with mapping from stick name to the unit
        # vector axis  along that stick
        connector_axes[edge[0]][new_stick.name] = axis.tolist()
        connector_axes[edge[1]][new_stick.name] = (-axis).tolist()

        stick_diameter = 3.0

      ## Luis ##
      for connector in connector_axes:
        axisList = connector.values()
        height = height_connector(stick_diameter,axisList)
        for stickName in connector.keys():
          stick = stickList[stickName]

          pt0 = stick.pose[0]
          quat0 = stick.pose[1]
          axis = connector[stickName]

          rot_axis = numpy.cross([0,0,1],axis)
          rot_angle = numpy.arccos(axis[2])
          quat1 = quaternion_about_axis(rot_angle, rot_axis)

          if numpy.array_equal(quat0, quat1):
            stick.pose = (numpy.add(pt0, numpy.multiply(height,axis)), quat0)
          stick.setLength(stick.length - height) # making the sticks shorter
      ## Luis ##

      # Add Plate body
      plate = Plate(base_pts)
      blocks.append(plate)

      # Add an axis for connection to plate for the last three connectors
      for axes in connector_axes[-3:]:
        axes[plate.name] = [0.0,0.0,-1.0]

      # Add Connector bodies with connection axes and correct translation pose
      for i in range(len(connector_axes)):
        blocks.append(
          Connector(connector_axes[i], pose=(self.cloud[i,:].tolist(),ORIGIN_POSE[1]))
        )
      
      kwargs['blocks'] = blocks
    
    super(Sculpture, self).__init__(**kwargs)
    
  def show_wireframe(self):
    """Show matplotlib 3D plot of wireframe of sculpture."""
    
    fig = plt.figure(2)
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(*self.cloud.T)
    
    for edge in self.edges:
      ax.plot(*self.cloud[edge,:].T)
    
    ax.axis('equal')

    plt.show()

  def show(self):
    super(Sculpture,self).show(is_2d=False)

  def get_layouts(self):
    """Return layouts for making this sculpture.
    
    Returns:
      guide_layout: 2D Layout for printing, use to cut sticks
      cut_layout: 2D Layout for laser cutting base
      print_layout: 3D Layout for 3D printing connectors

    """

    sticks = [body.clone() for body in self if type(body) is Stick]
    guide_layout = Layout(
      [Block(Layer(stick.guide())) for stick in sticks],
      size = (215,279) # Letter paper size
    )
    for i in range(len(sticks)):
      guide_layout[i] *= (0, 2.5+i*5.0, 0)
    
    plates = [body for body in self if type(body) is Plate]
    cut_layout = Layout(
      Block(Layer(plates[0].cut(),color='red')),
      size = (300,300) # Laser Cutter bed is larger, but use 30x30cm for now
    )
    
    connectors = [
      body.clone(pose=ORIGIN_POSE) for body in self if type(body) is Connector
    ]
    max_dims = numpy.vstack([conn.dims(2) for conn in connectors]).max(axis=0)
    n_x = numpy.floor(200/max_dims[0])
    i = 0
    j = 0
    print_layout = Layout(size=(200,200))
    for conn in connectors:
      z_min = conn.bounds()[0,2]
      conn.pose = ((i*max_dims[0],j*max_dims[1],-z_min),ORIGIN_POSE[1])
      print_layout += conn.transformed()
      if i == n_x-1:
        i = 0
        j += 1
      else:
        i += 1
     
    return guide_layout, cut_layout, print_layout

def seeded_solution(seed = 1):
  numpy.random.seed(seed)
  sculpture = Sculpture()
  guide_layout, cut_layout, print_layout = sculpture.get_layouts()
  return sculpture, guide_layout, cut_layout, print_layout

def solid_hole_example():
  h_g = solid.translate([0,0,-3])(solid.cylinder(3,20))+solid.hole()(solid.cylinder(1.5,20))
  h_g += solid.sphere(16)
  PolyMesh(generator=h_g).show()
  # Holes always get added at the end, so the geometry is ensured to be empty
  # This ALWAYS happens, so if you want to restrict the scope of holes
  h_g = solid.part()(solid.translate([0,0,-3])(solid.cylinder(3,20))+solid.hole()(solid.cylinder(1.5,20)))
  h_g += solid.sphere(16)
  PolyMesh(generator=h_g).show()
      
if __name__ == '__main__':
  sculpture, guide_layout, cut_layout, print_layout = seeded_solution()
  sculpture.save('sculpture_opt.scad')
  sculpture.save('sculpture_opt.stl')
  guide_layout.save('guide_layout.dxf')
  cut_layout.save('cut_layout.dxf')
  print_layout.save('print_layout.stl')
  #sculpture.show()