1. Run this node:
   1) Open one terminal: run a roscore
   1) Open a new terminal: rosbag play --clock name_of_bagfile.bag
   2) Open another terminal: roslaunch visual_tracking visual_tracking.launch
      or if your input bagfile was recorded in Bonn field:     roslaunch visual_tracking visual_tracking_Bonnfield.launch
2. Simple guidline for the code
   1) The function Vision::update() in vision_main.cpp file shows the process flow

    
 