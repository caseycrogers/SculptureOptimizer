# Lab 6: Optimization and Design Space Exploration

**Released Fri. Oct. 16. Due Fri. Oct. 23, 17:00**

In this lab you will learn how to:
- Use the ToPy library to create optimized structures via topology optimization
- Integrate ToPy generated solids into digifab bodies
- Perform design space exploration (DSE), followed by optimization 

## Topology Optimization

[ToPy](https://github.com/williamhunter/topy) is a Topology Optimization toolkit
resulting from William Hunter's thesis work. Given a 2D or 3D space and 
force/movement constraints, this framework will produce a structure that has
maximal stiffness for a given amount of material. This can produce some 
really neat organic looking [structures](http://www.daytondiode.org/2013/12/prusa-mendel-reprap-frame-vertex-topology-optimization-topy.html)
since bones tend to follow the same stress/strain optimization policies.

You will use ToPy to optimize a planar ternary link from either your walker or
gripper (or anything other fourbar output link you want) for laser cutting.

You can find details on the problem types and file formats in Hunter's
[thesis](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&ved=0CB4QFjAAahUKEwjoq7v_2sfIAhWLOIgKHX-EBsY&url=https%3A%2F%2Fir1.sun.ac.za%2Fbitstream%2Fhandle%2F10019.1%2F2648%2Fhunter_predominantly_2009.pdf%3Fsequence%3D1&usg=AFQjCNGqm0Dg_GR3X025oL4upuEXo_s7bg&sig2=Jnx94G96CefvH_oV0Okk9Q).
Specifically check out page 88 and 89 for details on how nodes are indexed, and how
input force and fixed node constraints are added.
You should only have to mess with FXTR\_NODE and LOAD\_NODE values in 
`ternary_link.tpd` to change where the forces and connections are for your link,
though there are a lot of knobs you can play with in the optimization.

### Exercises

1. Install ToPy and all required dependencies. 
 1. `git clone https://github.com/williamhunter/topy.git`
 2. from topy directory, `sudo python setup.py install`
 3. `sudo apt-get install python-sparse`
 5. `sudo pip install sympy`
 6. `sudo pip install pyvtk`
2. From lab\_6, run optomizer on example ternary link problem specification: `./optimise.py ternary_link.tpd` The first run will take a long time to generate stiffness matrices, after that it should run in less than 30 seconds. When done it should look like this:
![Ternary Link Grid](https://github.com/CS194-028/starter/blob/master/lab_6/assets/ternary_link_grid.png)
3. View the mechanism by running the sample code `./solution.py`. It should look like this (things get flipped around and rotated due to image indexing, but you should be able to fix this with basic transformations):
![Ternary Link Outline](https://github.com/CS194-028/starter/blob/master/lab_6/assets/ternary_link_outline.png)
4. Create or include your own fourbar mechanism, and modify `ternary_link.tpd` to match
the geometry of your mechanism.
5. Run the structure optimization, and add joint geometry to the resulting part
make a functional fourbar mechanism.
5. Construct the fourbar with the optimized geometry and bring it to class.

## Design Challenge

Based on feedback from the class, we'd like to give you a chance to revist one of your old 
labs. For this design challenge, you will perfrom a DSE on a design using fabrication methods from 
one of your previous labs (of your choosing), followed by an optimization. 

First choose what you want to make (chair, box, something exciting ...)

Define a parametrization for the design. The number of parameters determines the dimensionality of the 
optimization, and more parameters greatly increases the amount of computation required.

*Question: What are your parameters? What phyical properties do they correspond to?*

Also create a well-defined objective function, which you will optimize. We highly suggest not invovling SCAD operations in 
your evaluation of the objective function; it gets run 100's/1000's of times so it needs to be fast. Ideally define the objective 
on a generator for a design (eg.: the 3D point cloud for lab 3) rather than the fully rationalized object.

*Question: What are you trying to optimize, and how will you numerically evaluate the success of the design?*

Now perform a design space exploration on your parameters. This could take the form of a 
[grid-search](https://en.wikipedia.org/wiki/Hyperparameter_optimization#Grid_search), where 
you evaluate your design for regularly spaced values of your parameters, or a random search, 
if you have more than 2-4 parameters. 

Evaluate your objective function for each of the parameter sets in your exploration. 
Does the value of the objective function match with your intuition of what makes a 
good design? Feel free to modify your objective if it doesn't.

Pick a design point from your exploration, and optimize your parameters to minimize your 
objective function. We have provided skeleton code for using scipy's optimization functions.


### Deliverables
 1. Images of the designs from your DSE.
 2. Before and after pictures of the selected design point, and the subsequently optimized version.
 3. Fabricated version of the final design
