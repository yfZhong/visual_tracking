
#ifndef VISION_MAIN_H
#define VISION_MAIN_H


#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <sstream>
#include <boost/timer/timer.hpp>

#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int64.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <config_server/parameter.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "frameGrabber.h"
#include "Parameters/globaldefinitions.h"


#include "Projection/ForwardProjection.h"
#include "IcpPointMatcher/pointMatcher.h"
#include "CalculatePose/calculatePose.h"
#include "CalculatePose/kalmanFilter.h"

#include "CalculatePose/hillClimbing.h"
#include "DataAssociation/dataAssociation.h"
#include "Projection/DistortionModel.h"
#include "Parameters/Parameters.h"
#include "ParticleFilter/singleParticle.h"
#include "ParticleFilter/ParticleFilter.h"
#include "Tools/VisionRate.h"
#include "Tools/tools.h"

#include "camera/MatPublisher.h"
#include "camera/ICamera.h"
#include "camera/Camera.h"
#include "camera/CameraDummy.h"

#include "LineMatcher/lineMatcher.h"
#include "LineMatcher/findLines.h"
#include "LineMatcher/findNodes.h"
#include "FieldObjects/findField.h"
#include "FieldObjects/findGoalPosts.h"
#include "FieldObjects/findObstacle.h"


using namespace vision;
namespace vision
{


	class Vision
	{	
		public:			
			
			Vision(); 
			virtual ~Vision();

			/*! \fn virtual void onInit();
			*/
			virtual void onInit();
			
			enum{
			    H = ORG_IMAGE_HEIGHT,
			    W = ORG_IMAGE_WIDTH,
			    siX = UNDISTOTED_IMAGE_WIDTH, 
			    siY = UNDISTOTED_IMAGE_HEIGHT,
			
		        };

			/*!  \brief Executes the the image processing algorithms for each camera frame.
			*    \param img  pointer to the raw image.
			*/
			
			
			void update();
			
			void drawField();

			//! CameraFrame
 			/*! Object that contains static buffers that will be used during the identification process.*/
			FrameGrabber CameraFrame;
                        FindField FieldFinder;
			FindGoalPosts GoalPostsFinder;
			FindObstacle ObstacleFinder;
			FindLines LineFinder;
			FindNodes NodeFinder;
			DataAssociation AssociateData;
			

			
			PointMatcher pointMatcher;
			
			HillClimbing poseUpdate;
			
// 			PoseCalculator poseCalculator;
			M_KalmanFilter poseFilter;
			
			
			
// 			ParticleFilter particlefilter;
			LineMatcher lineMatcher;
			
                        ros::NodeHandle nh;
			
			ICamera *camera;
			bool useBagfile;
			
			
			geometry_msgs::PoseStamped robotPoseS;
			
		private:
                        
			
			image_transport::ImageTransport * it;
			image_transport::Subscriber image_sub_ ;
			image_transport::Publisher image_pub_;
			image_transport::Publisher image_pub_1;
			image_transport::Publisher image_pub_2;
			image_transport::Publisher image_pub_3;
			image_transport::Publisher image_pub_4;
			image_transport::Publisher image_pub_5;
			image_transport::Publisher image_pub_6;
			image_transport::Publisher image_pub_7;
			ros::Publisher robotPose_pub;
			ros::Publisher cameraPose_pub;
			ros::Publisher particlePose_pub;
			ros::Publisher particleCamPose_pub;
			ros::Publisher cameraPose_pub_ICP;
			ros::Publisher robotPose_pub_ICP;
			ros::Publisher robotPose_pub_KF;
			
			ros::Publisher associateLines_pub;
			ros::Publisher particlePoseMarker_pub;
			ros::Publisher fieldLineMarker_pub;
			
			ros::Publisher HypothesisArray_pub;
			
			
			  //Constants
// 			int H, W, siX, siY;
			
			sensor_msgs::Image::ConstPtr img;

			
			
	};

}//End namespace vision





#endif