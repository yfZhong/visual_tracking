
#include  <visual_tracking/rgb_yuyv/rgb2yuyv.h>

int main( int argc, char** argv ) {  // initialize
    ros::init(argc, argv, "rgb2yuyv");
    Rgb2Yuyv rgb2yuyv;
    ros::spin();
    return 0;
}