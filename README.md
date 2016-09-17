# Incremental Contour Based Segmentation #

Implementation of the Incremental Contour-Based Topological Segmentation  in structured or unstructured environments.
We transform the occupancy grid into a set of polygons, then we use the function  [Dual Space Decomposition](http://masc.cs.gmu.edu/wiki/Dude2D) from to segment them. The incremental version only decompose the Differences in the Maps



### Pre requisites ###

The requisites for [Dual Space decomposition]

1. CGAL library:

      $ sudo apt-get install libcgal-dev

2. [Freeglut](http://freeglut.sourceforge.net/)

3. MPFR

      $ sudo apt-get install libmpfr-dev
      
The requisites for ROS node

1. [ROS Indigo](http://wiki.ros.org/indigo)


### Instructions ###

* OpenCV wrapper

      Transform cv::Mat image into a set of to cv::Contours

* ROS nodes

      * inc_dude:

            Transform occupancy grid msg in topic /map and publish the decomposition in the topic /tagged_image.  /chatter topic is used to save the image of the current decomposition.

      * evaluation

            Transform the images in the dataset and print the resulting images











