# visual_tracking
1. This system is a ros package. After compiling it in a ros framework, the program can be run by following steps:
   1) Open one terminal: run a roscore
   2) Open a new terminal: rosbag play --clock name_of_input_bagfile.bag
   3) Open another terminal: roslaunch visual_tracking visual_tracking.launch
   4) Visualiza the output by opening another terminal and run the rqt.

2. Simple guidline for the code
   1) The function Vision::update() in vision_main.cpp file shows the process flow.
   2) Package FieldObjects: provides methods for detecting field boundary, obstacles, goal posts(haven't finished yet).
   3) Package LineMatcher: findLines.cpp-----apply line filters, find local optimal values, fit boundin squares.
                           findNodes.cpp-----construct node graph, cluster line segments
   4) Package DataAssociation: do the data association, and calculate the error.
   5) Package CalculatePose: hillClimbing.cpp-----hillclimbing method for pose optimization
                             calculatePose.cpp-----EPnP method for pose optimization
                             kalmanFilter.cpp------Kalman filter for state fusion

    Other used packages:
    Package Camera: read images
    Package Projection: do the projection of the 3D model to the image plane.
    Package Tools: m_Math.cpp
                   tools.cpp
