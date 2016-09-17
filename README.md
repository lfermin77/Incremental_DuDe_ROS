# Incremental Contour Based Segmentation #

Implementation of the Incremental Contour-Based Topological Segmentation  in structured or unstructured environments.
We transform the occupancy grid into a set of polygons, then we use the function  [Dual Space Decomposition](http://masc.cs.gmu.edu/wiki/Dude2D) from to segment them. The incremental version only decompose the Differences in the Maps



### Pre requisites ###

The requisites for [Dual Space decomposition]

1. CGAL library: 
        $ sudo apt-get install libcgal-dev

2. [Freeglut](http://freeglut.sourceforge.net/)

3. MPFR

The requisites for ROS node

1. [ROS Indigo](http://wiki.ros.org/indigo)


