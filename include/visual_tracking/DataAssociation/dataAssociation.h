// by Yongfeng

#ifndef DATA_ASSOCIATION_H
#define  DATA_ASSOCIATION_H


#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <visual_tracking/Parameters/camera_parameters.h>
#include <visual_tracking/Projection/ForwardProjection.h>
#include <visual_tracking/Parameters/globaldefinitions.h>
#include <visual_tracking/Projection/DistortionModel.h>

#include <visual_tracking/Parameters/Parameters.h>

#include <visual_tracking/LineMatcher/findNodes.h>

#include <visual_tracking/Tools/m_Math.h>
#include <robotcontrol/RobotHeading.h>
#include <visual_tracking/frameGrabber.h>




using namespace std;
using namespace  vision;
using namespace cv;

namespace vision
      {   
	
	class DataAssociation
	{
	  
	
	public:
	    DataAssociation();
	    ~DataAssociation(){};

	    
// 	    void generateModel(geometry_msgs::Pose &p, FrameGrabber & CamFrm);
	    Vector<pair<Vec2i,Vec2i> > DetectionToModelCorrespondences;
	    Vector<Vec2i> DetectionToModelOutliers;
	    float DTMInlierPct;
	    float DTMError;
	    
	    Vector<pair<Vec2i,Vec2i> > ModelToDetectionCorrespondences;
	    Vector<Vec2i> ModelToDetectionOutliers;
	    float MTDInlierPct;
	    float MTDError;

	    void AssociateDetectionToModel(FrameGrabber & CamFrm, FindNodes &NodeFinder, float &error);
	    void AssociateModelToDetection(FrameGrabber & CamFrm, FindNodes &NodeFinder, float &error);
            float getDTMInlierPct(){
	      return DTMInlierPct;
	    }
	    float getMTDInlierPct(){
	      return MTDInlierPct;
	    }
	    
	    float getDTMError(){
	      return DTMError;
	    }
	    float getMTDError(){
	      return MTDError;
	    }
	    
	    int getDTMInlierNum(){
	      return DetectionToModelCorrespondences.size();
	    }
	    int getMTDInlierNum(){
	      return ModelToDetectionCorrespondences.size();
	    }
	    int getDTMOutlierNum(){
	      return DetectionToModelOutliers.size();
	    }
	    int getMTDOutlierNum(){
	      return ModelToDetectionOutliers.size();
	    }
	    
	    float getAvgError(){//sqrt
	       if(getDTMInlierNum() ==0 || getMTDInlierNum()== 0  ){return std::numeric_limits<float>::max();/*;cout<<"no inlier"<<endl;*/}
	       return   sqrt( DTMError+ MTDError)/float(getDTMInlierNum()  + getMTDInlierNum());
	    }
	    
	    float getSumOfError(){//power
	        if(getDTMInlierNum() ==0 || getMTDInlierNum()== 0  ){return std::numeric_limits<float>::max();/*;cout<<"no inlier"<<endl;*/}
	        return   DTMError + MTDError;
	      
	    }
	    
	    
	    float getSumOfInlierNum(){//power
	        return   getDTMInlierNum() + getMTDInlierNum();
	      
	    }
	    
	    
	    float getConfidence(){
  
		  float ErrorThreshold1 = std::max(double(params.hillclimbing.ErrorThreshold1->get()), 1.0);
		  float ErrorThreshold2 = std::max(double(params.hillclimbing.ErrorThreshold2->get()), 1.0);
                  float error= getAvgError();

		  if(DTMInlierPct == 0 || MTDInlierPct == 0){return 0.0; /*cout<<"bad1"<<endl;*/}
		  if(DTMError == 0 || MTDError == 0){return 0.0; /* cout<<"bad2"<<endl;*/}

		  error = error/DTMInlierPct;
	   
		  if(error< ErrorThreshold1){ return 1.0;}
		  else if(error > ErrorThreshold2) {return 0.0;}
		  else{
		    return 1.0- (error - ErrorThreshold1)/(ErrorThreshold2 -ErrorThreshold1) ;
		  }
  
            }
	    
	    
	    
	    

       };



} 
   
   

#endif