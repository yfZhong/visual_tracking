
#ifndef TOOLS_H
#define TOOLS_H


#include <math.h>
#include <algorithm>
// #include <boost/concept_check.hpp>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <visual_tracking/Parameters/globaldefinitions.h>


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/time.h>       /* time */



#include <geometry_msgs/Quaternion.h>


#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
// #include <visual_tracking/Parameters/Parameters.h"

#define Width 640
#define Height 480


namespace vision
{
    class Tools
    {

    //         template < class T, int W, int H >
    // 	void writePGM( const T ( & arr )[ H ][ W ], const char* filename)
    // 	{
    // 		double maxval = *std::max_element((T*)arr,(T*)arr+H*W);
    // 		maxval = std::max((double)maxval,0.00001);
    // 		std::string absName = ros::package::getPath("vision_6d_pose") + "/tmp_file/" + filename;
    // 		std::ofstream os(absName.c_str());
    // 		os << "P2 "<<W << " " << H<<" 255"<<std::endl;
    // 		for ( int y = 0; y < H; y++ ) {
    // 			for ( int x = 0; x < W; x++ ) {
    // 				//os.width ( 3 );
    // 				os<<(int)(255 * arr[H-1-y][x] / maxval)<<" ";
    // 			}
    // 			os<<std::endl;
    // 		}
    // 		os.close();
    // 	}
      
    //      template < class T, int W, int H >
    // 	void
    // 	writeDAT( const T ( & arr )[ H ][ W ], const char* filename)
    // 	{
    // 	  
    // 	  
    // 	    //output values of a matrix  
    // 	      std::ofstream ofs ;
    // 	      std::string absName = ros::package::getPath("vision_6d_pose") + "/tmp_file/" + filename;
    // 	      ofs.open (absName.c_str(),std::ofstream::out|std::ofstream::trunc);
    // 	      
    // 	      for ( int j =0; j< H; j++){
    // 		  for ( int i = 0; i< W; i++){  
    // 
    // 		      ofs<<  arr[H-j][i]<< " ";
    // 		      
    // 		  }
    // 		  ofs<< std::endl;
    // 	    }
    // 	
    // 	   ofs.close();
    // 
    // 	}
	    
// 	    int Width = params.camera.width->get();
// 	    int Height = params.camera.height->get();
	    
//       	    int Width = ORG_IMAGE_WIDTH;
// 	    int Height = ORG_IMAGE_HEIGHT;
    public:
	    template < class T >
	    void writePGM( std::vector<T> arr, const char* filename)
	    {
	      
    // 	     
		    double maxval = *std::max_element(arr.begin(),arr.end());
		    maxval = std::max((double)maxval,0.00001);
		    std::string absName = ros::package::getPath("vision_6d_pose") + "/tmp_file/" + filename;
		    std::ofstream os(absName.c_str());
		    os << "P2 "<<Width << " " << Height<<" 255"<<std::endl;
		    for ( int y = 0; y < Height; y++ ) {
			    for ( int x = 0; x < Width; x++ ) {
				    //os.width ( 3 );
				    os<<(int)(255 * arr[(Height-1-y)*Width +x] / maxval)<<" ";
			    }
			    os<<std::endl;
		    }
		    os.close();
	    }
	    
	    
	    template < class T >
	    void writeDAT(std::vector<T> arr, const char* filename)
	    {

		//output values of a matrix  
		  std::ofstream ofs ;
		  std::string absName = ros::package::getPath("vision_6d_pose") + "/tmp_file/" + filename;
		  ofs.open (absName.c_str(),std::ofstream::out|std::ofstream::trunc);
		  
		  for ( int j =0; j< Height; j++){
		      for ( int i = 0; i< Width; i++){  

			  ofs<<  arr[ (Height-j-1)*Width + i]<< " ";
			  
		      }
		      ofs<< std::endl;
		}
	    
	      ofs.close();

	    }
	    
	    template < class T >//vector<vector<int> > skeletonPixel
	    void writeCSV(std::vector<std::vector<T> > arr, const char* filename)
	    {

		//output values of a matrix  
		  std::ofstream ofs ;
		  std::string absName = ros::package::getPath("vision_6d_pose") + "/tmp_file/" + filename;
		  ofs.open (absName.c_str(),std::ofstream::out|std::ofstream::trunc);
		  
		  for ( int j =0; j< arr.size(); j++){
		      for ( int i = 0; i< 2; i++){  

			  ofs<<  arr[j][i]<< " ";
			  
		      }
		      ofs<< std::endl;
		  }
	    
	      ofs.close();

	    }
	    
	    
	    template < class T >
	    void cvshowImg( std::vector<T> arr, const char* windowName)
	    {
		  unsigned char *data_for_vis= new unsigned char[Width*Height];

		  double maxval = *std::max_element(arr.begin(),arr.end());
		  maxval = std::max((double)maxval,0.00001);
		  double minval = *std::min_element(arr.begin(),arr.end());
		  

		  for ( int j =0; j< Height; j++){
		    for ( int i = 0; i< Width; i++){  
			      data_for_vis[(j)*Width + i] =  double(arr[j*Width + i]-minval)/double(maxval-minval) *255;
		      }	     
		    }

		    cv::Mat grayH0(Height, Width, CV_8UC1, data_for_vis);
	    // 	cv::threshold(grayH0, grayH0, 8, 255, cv::THRESH_BINARY);
		    cv::imshow(windowName ,grayH0);
		    cv::waitKey(1);
		    delete[] data_for_vis;

	    }
	    
	   
	    
	    
	    
	    void generateKernelImg2(const char* kernelname);
	    //generate seeds for random variables
	    void seed_rand();
	    bool findPointInVector(cv::Point3f p, std::vector<cv::Point3f> points);
	    bool findPointInVector(cv::Point3f p, std::vector<tf::Vector3>  points);
	    void quaternionNormalize(geometry_msgs::Quaternion &a);
	    geometry_msgs::Quaternion quaternionMultiplication(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2);

	    
	    
	    
    };
};
extern vision::Tools tools;

#endif