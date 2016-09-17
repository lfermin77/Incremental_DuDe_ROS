# Incremental Contour Based Segmentation #

Implementation of the Incremental Contour-Based Topological Segmentation  in structured or unstructured environments.
We transform the occupancy grid into a set of polygons, then we use the function  [Dual Space Decomposition](http://masc.cs.gmu.edu/wiki/Dude2D) from to segment them. The incremental version only decompose the Differences in the Maps



### Pre requisites ###

The requisites for [Dual Space decomposition]

*[CGAL library]: sudo apt-get install libcgal-dev

*[Freeglut](http://freeglut.sourceforge.net/)

*[MPFR]

The requisites for ROS node

*[ROS Indigo](http://wiki.ros.org/indigo)


