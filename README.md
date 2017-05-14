# Incremental Contour Based Segmentation #

Implementation of the Incremental Contour-Based Topological Segmentation  in structured or unstructured environments.
We transform the occupancy grid into a set of polygons, then we use the function  [Dual Space Decomposition](http://masc.cs.gmu.edu/wiki/Dude2D) to segment them. The incremental version only decompose the Differences in the Maps



### Pre requisites ###

The requisites for [Dual Space decomposition]

1. CGAL library:

      $ sudo apt-get install libcgal-dev

2. [Freeglut](http://freeglut.sourceforge.net/)

3. MPFR

      $ sudo apt-get install libmpfr-dev
      
The requisites for the OpenCV wrapper

1. [OpenCV](http://opencv.org/)
      
The requisites for ROS nodes

1. [ROS Indigo](http://wiki.ros.org/indigo)


### Description ###

- OpenCV Wrapper

      Transform *cv::Mat* image into a set of to *cv::Contours* and decompose it using [DuDe](http://masc.cs.gmu.edu/wiki/Dude2D).*"Don't need ROS"*
      
- Incremental Decomposition

      Transform occupancy grid into an OpenCV binary image, it then compares it to the previous decompostion to obtain a difference image to be decomposed with OpenCV Wrapper.

- ROS nodes

      - **inc_dude:** 
            Transform occupancy grid msg in topic */map* and publish the decomposition in the topic */tagged_image*.  /chatter topic is used to save the image of the current decomposition. Optional parameter *concavity threshold* (3 m in this example).
            
            $ roscd && cd ..
            $ rosrun inc_dude inc_dude 3
            
      - **evaluation:** 
            Transform the images in the dataset and print the resulting images. Images are published in the topics */ground_truth_segmentation*, */DuDe_segmentation*, */inc_dude_segmentation*. The topic */cmd_vel* is used to cycle the images: *Up* to cycle, *Down* to process. Optional parameter *concavity threshold* (3 m in this example).
            
            $ roscd && cd ..
            $ rosrun inc_dude evaluation 3
            

### Related papers ###

- Fermin L., Neira J. and Castellanos, J.A. Incremental Contour-Based Topological Segmentation for Robot Exploration. *2017 IEEE International Conference on Robotics and Automation, ICRA 2017*. (*submitted*)









