#!/usr/bin/python

from digifab import *
from solution_lab5 import * #importing the SynthFourBar class
import numpy
import solid

# EDITED - Luis #

# USEFULL THING #

origin = numpy.array([0,0,0])

def pts_to_vec (pt_a, pt_b):
  start = numpy.asarray(pt_a)
  end = numpy.asarray(pt_b)
  vec = end - start
  return vec


# AUXILIAR FUNCTION #
# Selecting the ternary link #
# Making a filter function #
# Test: which body has 3 joints? #

def filter_three_joints (a_mechanism):
	for body in a_mechanism:
		if len(body.joints) == 3:
			ternary_link = body
		else:
			pass
	return ternary_link

###################################

# GETTING JOINT COORDINATES AND MODIFIED BOUNDING BOX (always a square for simplification issues) CORNER COORDINATES #

def joints_bb_coord (mechanism):
	joints_coord=[]
	bb_coord=[]
	origin = numpy.array([0,0,0])
	
	ternary_body = filter_three_joints(mechanism)
	ternary_joints = ternary_body.joints
	for i in range(len(ternary_joints)):
		joints_coord.append(ternary_joints[i].pose[0])
	
	BBox = ternary_body.bounding_box()
	BBox_points= BBox.points
	BBox_point0 = numpy.append(BBox_points[0], [0]) #numpy append to add a z coord

	width = numpy.linalg.norm(pts_to_vec(BBox_points[0],BBox_points[2]))
	height = numpy.linalg.norm(pts_to_vec(BBox_points[0],BBox_points[1]))
	
	if width == height:
		bb_square = BBox
	elif width > height:
		bb_square = PolyLine(generator = solid.square(width)).simplified()
		tr_vec = translation_matrix(pts_to_vec(origin, BBox_point0))
		bb_square *= tr_vec
	else:
		bb_square = PolyLine(generator = solid.square(height)).simplified()
		tr_vec = translation_matrix(pts_to_vec(origin, BBox_point0))
		bb_square *= tr_vec

	bb_square_points = [numpy.append(bb_square.points[0], [0]), numpy.append(bb_square.points[2], [0])] #numpy append to add a z coord - consistency

	# For the sake of clarity transforming the arrays of bb_square_points to tuplles #
	# In this way the output of the function would be always lists of tupples #

	bb_pts_tpl = map (tuple, bb_square_points)

	return joints_coord, bb_pts_tpl

###################################

# APPLYING THE JOINT COORD TO MECHANISMS#

#gripper_joints_bb = joints_bb_coord(gripper_arm) 

###################################

# EDITED - Luis ###################


# EDITED - Casey #

# Transforming the coordinate system #

def dumb_coordinates(cList, bounds, dim):
  bound0 = bounds[0]
  bound1 = bounds[1]
  ret = []
  dim = dim + 1
  for c in cList:
    gX = numpy.round( (c[0]-bound0[0])/(bound1[1]-bound0[0])*dim )
    gY = numpy.round( (c[1]-bound0[1])/(bound1[1]-bound0[1])*dim )
    ret.append(dim*(gX) + gY)
  return ret

# EDITED - Casey ##################

# EDITED - Luis #

###################################

# IMPORTING MECHANISMS #
#gripper_r_arm = SynthFourBar(B = 35+18.52j, D = 84.15 + 52.93j, P= (100+237.1j, 76.95+244.45j, 53+250j, 28.6+253.3j, 4+254.73j)) #the solution is gripper_r_arm.children[0]
#gripper_arm = gripper_r_arm.children[0]

robot_leg = SynthFourBar(B= 0+80j, D= 0+20j, P= pattern(0.5, 10+0j)) #the solution is robot_leg.children[1]
robot_leg = robot_leg.children[1]
ternary_part = filter_three_joints(robot_leg)

###################################

# ALTERNATIVE WAY TO MAKE THE COORDINATE SYSTEM TRANSFORMATION #
# Usefull stuff #

ternary_bbox_min = numpy.append(numpy.asarray(ternary_part.bounding_box().bounds()[0]), [0])
ternary_bbox_max = numpy.append(numpy.asarray(ternary_part.bounding_box().bounds()[1]), [0])

ternary_tr_vec = pts_to_vec(ternary_bbox_min, origin)

# Principal functions #

def grid_cart (cell_number, cell_length): #returns a grid with the same number of cells in the y and the x direction
	dim = cell_length
	vertex = []
	column_num = cell_number + 1
	row_num = cell_number + 1
	for i in range(column_num): 
		for j in range(row_num):
			vertex.append([i*dim, j*dim, 0])
	return numpy.asarray(vertex)


def grid_cart_pts (cell_number_x, cell_number_y, cell_length_x, cell_length_y): #returns a grid with the different number of cells in the y and the x direction
	dim1 = cell_length_x
	dim2 = cell_length_y
	vertex = []
	column_num = int(cell_number_x + 1)
	row_num = int(cell_number_y + 1)
	for i in range(column_num): 
		for j in range(row_num):
			vertex.append([i*dim1, j*dim2, 0])
	return numpy.asarray(vertex)

def closest_points(node, nodes): #computes what is the point in topy vertex notation that is closest to a given point
	node = numpy.asarray(node)
	dist_2 = numpy.sum((nodes - node)**2, axis=1)
	return numpy.argmin(dist_2) + 1

def remap_pts_px(cart_pts, dim, dist): #remaps cartesian coordinates to "pixel" coordinates of a square grid
  px_pts_x = (cart_pts[0] * dim) / dist
  px_pts_y = (cart_pts[1] * dim) / dist
  px_pts = numpy.array([px_pts_x, px_pts_y, 0]) 
  return px_pts

def remap_pts_px1(cart_pts, dim_x, dim_y, w, h): #remaps cartesian coordinates to "pixel" coordinates of any grid
  px_pts_x = (cart_pts[0] * dim_x) / w
  px_pts_y = (cart_pts[1] * dim_y) / h
  px_pts = numpy.array([px_pts_x, px_pts_y, 0]) 
  return px_pts

joints_pose = numpy.asarray(joints_bb_coord(robot_leg)[0]) # to make math with arrays

joints_pose_tr = [x + ternary_tr_vec for x in joints_pose] # translating the original joints pose origin
joints_pose_tr = (map(tuple, joints_pose_tr )) # making the transformed joints pose into a tupple 

# CREATING NEW POSES FOR THE JOINTS OF THE TRANSLATE TERNARY PART #

joints_pose_tr_0 = (joints_pose_tr[0], Z_JOINT_POSE[1])
joints_pose_tr_1 = (joints_pose_tr[1], Z_JOINT_POSE[1]) 
joints_pose_tr_2 = (joints_pose_tr[2], Z_JOINT_POSE[1]) 

ternary_part_tr = ternary_part.transformed((tuple(ternary_tr_vec), Z_JOINT_POSE[1]))
ternary_part_pl = ternary_part_tr[0] # to do: make this a layer with the 2 on it


# CALCULATING CELL LENGTH (cell_l) KNOWING CELL NUMBER (cell_n) #

cell_n = 240

ternary_bBox= ternary_part_pl.bounding_box().points

ternary_w = numpy.linalg.norm(pts_to_vec(ternary_bBox[0],ternary_bBox[3]))
ternary_h = numpy.linalg.norm(pts_to_vec(ternary_bBox[0],ternary_bBox[1]))

max_distance = max(ternary_w, ternary_h)

cell_y = int(round((ternary_h * cell_n)/ternary_w))

#cell_l = 0.265 #mm per pixel

# REMAPPING THE POINTS IN PIXEL(grid) WORLD #

nodes_pts = [remap_pts_px1(i, cell_n, cell_y, ternary_w, ternary_h) for i in joints_pose_tr]

# GENERATING THE GRID AND THE NODES #

grid = grid_cart(cell_n,1)
grid = grid_cart_pts(cell_n, cell_y, 1, 1) # to be consistent we assume that each cell has 1 unit
topo_nodes = [closest_points(x,grid) for x in nodes_pts]

if __name__ == '__main__':

  p = PolyLine(filename='ternary_link_030.png')
  p_bBox = p.bounding_box()
  p_bBox_origin = numpy.append(p_bBox.points[0],[0])
  tr_mtx = translation_matrix(pts_to_vec(p_bBox_origin, origin))
  p *= tr_mtx
  
  # Lets create big circles and position them #
  a_circle= PolyLine(generator= solid.circle(5.5)).simplified()
  a_circle_lst=[]

  for i in range(len(joints_pose_tr)):
  	b = a_circle.clone()
  	trl_mtx = translation_matrix(joints_pose_tr[i])
  	b *= trl_mtx
  	a_circle_lst.append(b)

  # Lets create circles/holes for the joints and position them #
  c_circle= PolyLine(generator= solid.circle(3.5)).simplified()
  c_circle_lst=[]

  for i in range(len(joints_pose_tr)):
    d = c_circle.clone()
    trl_mtx = translation_matrix(joints_pose_tr[i])
    d *= trl_mtx
    c_circle_lst.append(d)

  p_bBox= p.bounding_box().points
  
  p_w = numpy.linalg.norm(pts_to_vec(p_bBox[0],p_bBox[3]))
  p_h = numpy.linalg.norm(pts_to_vec(p_bBox[0],p_bBox[1]))

  scale_factor_x = ternary_w/p_w
  scale_factor_y = ternary_h/p_h

  p_scl = PolyLine(generator= solid.scale([scale_factor_x,scale_factor_y,1])(p.get_generator())).simplified()

  for i in range(len(a_circle_lst)):
    p_scl = p_scl | a_circle_lst[i]
   
  # Lets create layers #
  p_scl_layer= Layer(p_scl, color= 'red')
  circle_layer= Layer(c_circle_lst, color='red')

  Final_block = Block([p_scl_layer, circle_layer])

  a_layout= Layout(blocks= Final_block).save('opt_foot.dxf')