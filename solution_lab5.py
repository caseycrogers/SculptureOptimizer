#!/usr/bin/python

from digifab import *
import numpy
import solid

## EDITED - Luis ##
"""
IMPORTANTE NOTICE: FOR THIS TO WORK THE CLASS DEFINED IN DISK_PLANAR_BODY.py
NEEDS TO BE UPDATED WITH THE FILE THAT COMES WITH THIS DELIVER. PLEASE REPLACE
THAT FILE IN DIGIFAB'S EXAMPLE FOLDER
"""

# USEFULL STUFF #
def pts_to_vec (pt_a, pt_b):
  start = numpy.asarray(pt_a)
  end = numpy.asarray(pt_b)
  vec = end - start
  return vec

"""
Original Points:

B = -9.65+96.6j, D = -73.5-91.3j,
P = (-100+44j, -110+21j, -114-1.5j, -113-23j, -110-41j)

CALL:

### THIS IS COOL - number 1
def pattern(s, off):
  tpl=[(-50 - 0j) * s, (-5 - 0j)*s, (-12 + 10j)*s, (-25+15j)*s, (-40+10j)*s]
  for i in range(len(tpl)):
    tpl[i] += off
  return tpl

sfb = SynthFourBar(B= 0+80j, D= 0+20j, P= pattern(0.5, 10+0j))
sfb.show()
###

### THIS IS NOT COOL YET - number 2
def pattern(s, off):
  tpl=[(-75 - 0j) * s, (-5 - 0j)*s, (-15 + 10j)*s, (-45+20j)*s, (-60+10j)*s]
  for i in range(len(tpl)):
    tpl[i] += off
  return tpl

sfb = SynthFourBar(B= 0+80j, D= 0+40j, P= pattern(0.5, 10+0j))
sfb.show()
###
"""
## EDITED - Luis ##

class SynthFourBar(Mechanism):
  def __init__(self, B = -9.65+96.6j, D = -73.5-91.3j,
    P = (-100+44j, -110+21j, -114-1.5j, -113-23j, -110-41j), signs=(1,1,1,1,1),
    origins=None, **kwargs):
    
    """Show all solutions of a synthesis problem showing output points.
    
    Args:
      B,D,P : synthesis arguments, see fourbar_synthesis.py
      origins: list of positions to put generated mechanisms at. By default
        will be spaced apart to not overlap
    """

    if 'name' not in kwargs.keys():
      kwargs['name'] = 'synth_four_bar'


    if 'children' not in kwargs.keys():
      # Get synthesis solutions, and remove those that don't have A/Ac and C/Cc
      # as complex conjugates
      solns = filter(is_consistent, synthesis(B, D, P, signs))
      
      # Remove 2R solutions
      solns = [s for s in solns if abs(s[0]-B) > 0.01]

      if not len(solns):
        raise Exception('No consistent solution found for synthesis')
      
      children = []
      constraints = []
      child_offset = 0.0
      soln_count = 0
      origins = []

      for A,_,C,_ in solns:
        # Create an elbow up and elbow down mechanism for each synthesis solution
        vectors = [B-A,D-B,(C-D,P[0]-D),A-C]
        
        up_child = FourBar(
          vectors=vectors, elbow=0, name='soln_%d_up' % soln_count,
        )

        down_child = up_child.clone(
          elbow=1, name='soln_%d_down' % soln_count,
        )
        
        # space children out by twice the maximum link length to guarantee no
        # overlapping
        offset = 2 * max(up_child.lengths)
        up_pos = (child_offset + A.real, A.imag, 0.0)
        down_pos = (child_offset + A.real, offset + A.imag, 0.0)

        up_child.constraints = [('body',(0,0,0),(up_pos, ORIGIN_POSE[1]))]
        down_child.constraints = [('body',(0,0,0),(down_pos, ORIGIN_POSE[1]))]

        origins.extend([(child_offset, 0.0),(child_offset,offset)])

        children.extend([up_child,down_child])
        
        constraints.extend([
          ('body', (up_child.name,0,0), (up_pos,ORIGIN_POSE[1])),
          ('body', (down_child.name,0,0), (down_pos,ORIGIN_POSE[1]))
        ])
        
        soln_count += 1
        child_offset += offset
      
      kwargs['children'] = children
      kwargs['constraints'] = constraints
    
    if type(B) is complex:
      self.B = (B.real, B.imag)
    else:
      self.B = B

    if type(D) is complex:
      self.D = (D.real, D.imag)
    else:
      self.D = D

    if type(P[0]) is complex:
      self.P = [(p.real,p.imag) for p in P]
    else:
      self.P = P
    
    self.origins = origins
    self.signs = signs

    super(SynthFourBar, self).__init__(**kwargs)
    
  def plot(self, plotter, **kwargs):
    x,y = zip(*self.P)
    x,y = numpy.array(x), numpy.array(y)
    for x_o, y_o in self.origins:
      plotter.ax.plot(x+x_o,y+y_o,'k*')

    super(SynthFourBar, self).plot(plotter, **kwargs)

  def synth_angle(self, synth_idx, child_idx):
    """Given a synthesis point index, return the angle that should be between
      body 0 and body 1

    Args:
      synth_idx (int): index into P from synthesis problem
      child_idx (int): which of self.children to get angle for
    """
    
    P = [complex(*pi) for pi in self.P]
    S,T = inv_kin_2R(complex(*self.B), complex(*self.D), P[0], P[synth_idx])[self.signs[synth_idx]]
    return self.children[child_idx].init_angle + numpy.angle(S)

  def show(self, state=None, **kwargs):
    """Show collection of synthesized mechanisms
    
    Args:
      state (int|float): if int, use synth_angle to assign mechanism to pose
        that reaches output point P[state]. If float, assign all children
        mechanism 
    """

    if type(state) is int:
      for i in range(len(self.children)):
        self.children[i].state[0] = self.synth_angle(state,i)

    elif type(state) is float:
      for child in self.children:
        child.state[0] = state
    
    super(SynthFourBar, self).show(**kwargs)

## EDITED - Luis#
# A good motion pattern for the leg #
def pattern(s, off):
  tpl=[(-50 - 0j) * s, (-5 - 0j)*s, (-12 + 10j)*s, (-25+15j)*s, (-40+10j)*s]
  for i in range(len(tpl)):
    tpl[i] += off
  return tpl

# Automating the generation of solved layouts fot Laser Cutting #

def aux(body):
  L=[]
  for i in range(len(body)):
    geom = body[i][0].split()
    a_l= Layer(geom, color = body[i].color)
    L.append(a_l)
  return Block(layers= L)

def gen_laser_cuts(a_mechanism, name ='cuts', sheet = (600,300), plot_to_file = True):
  bl=[]
  for i in range(len(a_mechanism)):
    bl.append(aux(a_mechanism[i]))
  a_layout= Layout(blocks= bl, size = sheet).solved(margin = 3.0)
  if plot_to_file == True:
    return a_layout[0].save(name + '.dxf')
  else:
    return a_layout[0] # GET ITEM BECAUSE SOLVED LAYOUTS RETURNS ALWAYS A LIST

# PRODUCING GEOMETRY #
# BELOW THE REAL CALLS #
# 1ST CALL TO ROBOT LEG #
# OTHER CALLS FOR EACH ARM OF THE GRIPPER #

robot_leg = SynthFourBar(B= 0+80j, D= 0+20j, P= pattern(0.5, 10+0j)) #the solution is robot_leg.children[1]
gripper_r_arm = SynthFourBar(B = 35+18.52j, D = 84.15 + 52.93j, P= (100+237.1j, 76.95+244.45j, 53+250j, 28.6+253.3j, 4+254.73j)) #the solution is gripper_r_arm.children[0]
#gripper_r_arm = SynthFourBar(B = 36.21+18.52j, D = 85.36 + 52.93j, P= (101.67+237.1j, 78.17+244.45j, 54.2+250j, 29.8+253.3j, 5.19+254.73j)) #the solution is gripper_r_arm.children[0] 
gripper_l_arm = SynthFourBar(B = -36.21+18.52j, D = -85.36 + 52.93j, P= (-101.67+237.1j, -78.17+244.45j, -54.2+250j, -29.8+253.3j, -5.19+254.73j)) #the solution is gripper_r_arm.children[2] 

# SELECTING GOOD INDIVIDUALS - ALWAYS NEED A VISUAL CHECK WITH "*.show()"

bot_leg = robot_leg.children[1]
grip_r = gripper_r_arm.children[0]
grip_l = gripper_l_arm.children[2]

# GET A SUPPORT FOR THE GRIPPER # 
# Just a square #

# Calculating translation vectors
Pt0 = numpy.asarray(grip_r[0].joints[1].pose[0])
Pt1 = numpy.asarray(grip_r[0].joints[0].pose[0])
Pt0_inv = numpy.array([Pt0[0] * (-1), Pt0[1], 0])

tr_matrix = translation_matrix(pts_to_vec(Pt0, Pt1))
tr_matrix_inv = translation_matrix(pts_to_vec(Pt0_inv, Pt1))

# The holes #
a_circle = PolyLine(generator = solid.circle(r = 7/2.0))
right_circle = a_circle.clone()
left_circle = a_circle.clone()

right_circle *= translation_matrix([40,0,0])
left_circle *= translation_matrix([-40,0,0])

right_down_circ = right_circle.clone()
right_down_circ *= tr_matrix 

left_down_circle = left_circle.clone()
left_down_circle *= tr_matrix_inv

circles = right_circle + right_down_circ + left_circle + left_down_circle

rec = PolyLine(generator = solid.square(size = [175,175], center = True))

base = Block(Layer(geometries = circles + rec, color= 'red'), name = 'base_gripper')

# TO PRODUCE LAYOUT CUTS #

gen_laser_cuts(bot_leg, name = 'robot_leg')

grip_l_layout= gen_laser_cuts(grip_r, name = 'gripper_l_arm', plot_to_file = False) # Because it is just a mirror of grip_r 

gen_laser_cuts(grip_r, name = 'gripper_r_arm', sheet = (1200,600), plot_to_file = False).augmented(base).augmented(grip_l_layout).solved(margin = 3.0)[0].save('gripper.dxf')
 
# NOW CUT AND ASSEMBLE YOU LASY BASTARD #

"""
TEST FOR GRIPPER ARMS - this was a nightmare, I wish I was a more math guy

plier1= SynthFourBar(B = 0+0j, D = 2 + 20j, P= (-35+60j, -30+59j, -25+58j, -20+57j, -15+56j))

plier3= SynthFourBar(B = 0-30j, D = 2 + 10j, P= (-35+60j, -30+59j, -25+58j, -20+57j, -15+56j))

plier3= SynthFourBar(B = 0-30j, D = 2 + 10j, P= (-40+60j, -30+59j, -25+58j, -20+57j, -15+56j))

plier3= SynthFourBar(B = 0-30j, D = 0.5 + 10j, P= (-40+60j, -30+59j, -25+58j, -20+57j, -15+56j))

plier2= SynthFourBar(B = 0-10j, D = 2 + 30j, P= (-35*2+60j, -30*2+59j, -25*2+58j, -20*2+57j, -15*2+56j))


plier1= SynthFourBar(B = 0+0j, D = 2 + 20j, P= (-35+50j, -30+49j, -25+48j, -20+47j, -15+46j))


plier4= SynthFourBar(B = 0+0j, D = 4 + 4j, P= (2+8j, 1+8.25j, 0+8.5j, -1+8.75j, -2+9j))

plier4= SynthFourBar(B = 0+0j, D = 40 + 40j, P= (20+80j, 10+82.5j, 0+85j, -10+87.5j, -20+90j))

plier4= SynthFourBar(B = 0+0j, D = 40 + 40j, P= (30+120j, 10+122.5j, -10+125j, -30+127.5j, -60+130j))

####
plier4= SynthFourBar(B = 36.21+18.5j, D = 86.93 + 52.13j, P= (101.88+230.17j, 78.36+237.73j, 54.3+243.3j, 29.85+246.82j, 5.19+248.28j))
###

plier4= SynthFourBar(B = 93.75+0j, D = 143 + 34.5j, P= (101.67+237.1j, 78.17+244.45j, 54.2+245j, 29.8+253.3j, 5.19+254.73j))

#### - # - works the best
plier4= SynthFourBar(B = 36.21+18.5j, D = 86.93 + 52.13j, P= (101.67+237.1j, 78.17+244.45j, 54.2+250j, 29.8+253.3j, 5.19+254.73j))
### - #

#### - # - works even better
plier4= SynthFourBar(B = 36.21+18.52j, D = 85.36 + 52.93j, P= (101.67+237.1j, 78.17+244.45j, 54.2+250j, 29.8+253.3j, 5.19+254.73j))
### - #plier4.children[0]

#### - #
plier5= SynthFourBar(B = 38.68+0j, D = 113.68 + 0j, P= (85 + 160.96j, 79.29 + 169.66j, 63.03+ 183.99j, 38.7 + 195.25j, 10 + 195.96j))
### - #
"""

## EDITED - Luis #