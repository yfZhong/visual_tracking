 ///--------- by Yongfeng--------///


#ifndef POINT_MATCHER_H
#define POINT_MATCHER_H


#include <iostream>
#include <vector>
#include "icpPointToPlane.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "icpPointToPoint.h"
#include <visual_tracking/Parameters/Parameters.h>

using namespace std;

namespace vision
{
  class PointMatcher{
    
  public:
      
      vector<int> correspondences;
      int inlierNum;
      vector<int> inlierIdx;
      PointMatcher(){inlierNum=0;};
      
//       void applyTransform(double *T,const int32_t T_num, Matrix R,Matrix t ,std::vector<vector<int> > &out){
// 	  if (t.m ==2) {
// 	    
// 	      // extract matrix and translation vector
// 	      double r00 = R.val[0][0]; double r01 = R.val[0][1];
// 	      double r10 = R.val[1][0]; double r11 = R.val[1][1];
// 	      double t0  = t.val[0][0]; double t1  = t.val[1][0];
// 
// 	      // check for all points if they are inliers
// 	      for (int32_t i=0; i<T_num; i++) {
// 		std::vector<int> point;
// 		// transform point according to R|t
// 		point.push_back( r00*T[i*2+0] + r01*T[i*2+1] + t0); 
// 		point.push_back( r10*T[i*2+0] + r11*T[i*2+1] + t1); 
// 		out.push_back(point);
// 	      }
// 
// 	    // dimensionality 3
// 	  } 
// 	  else {
// 	      
// 	      // extract matrix and translation vector
// 	      double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
// 	      double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
// 	      double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
// 	      double t0  = t.val[0][0]; double t1  = t.val[1][0]; double t2  = t.val[2][0];
// 
// 	      // check for all points if they are inliers
// 	      for (int32_t i=0; i<T_num; i++) {
// 		std::vector<int> point;
// 		// transform point according to R|t
// 		point.push_back(r00*T[i*3+0] + r01*T[i*3+1] + r02*T[i*3+2] + t0); 
// 		point.push_back(r10*T[i*3+0] + r11*T[i*3+1] + r12*T[i*3+2] + t1); 
// 		point.push_back(r20*T[i*3+0] + r21*T[i*3+1] + r22*T[i*3+2] + t2);
// 		out.push_back(point);
// 	      }
// 	  }
// 	  
// 
//       };
    
//       void matcher(std::vector<vector<int> > vecIn, std::vector<vector<int> > ref/*, std::vector<vector<int> >& out  */){
      
        void getInfo(vector<int>& _correspondences,int& _inlierNum,  vector<int>& _inlierIdx){
	  	correspondences = _correspondences;
		inlierNum = _inlierNum;
		inlierIdx = _inlierIdx;
	};
      
	void matcher(std::vector<cv::Point> vecIn, std::vector<cv::Point> ref/*, std::vector<vector<int> >& out  */){
	    
	    if(vecIn.size()>5 && ref.size()>5){
		int32_t dim = 2;
  // 	      int32_t dim = vecIn[0].size();
		int32_t numT = vecIn.size();
		int32_t numM = ref.size();
		// allocate model and template memory
	      
		double* T = (double*)calloc(dim*numT,sizeof(double));
		double* M = (double*)calloc(dim*numM,sizeof(double));
		
		// set model and template points
// 		for(int i = 0;i< numT; i++){
// 		  for(int j = 0; j<dim; j++ ){
// 		      T[i*dim+j] = vecIn[i][j];
// 		  } 
// 		}
// 		for(int i = 0;i< numM; i++){
// 		  for(int j = 0; j<dim; j++ ){
// 		      M[i*dim+j] = ref[i][j];
// 		  } 
// 		}
// 		
		
		for(int i = 0;i< numT; i++){
		      T[i*dim+0] = vecIn[i].x;
		      T[i*dim+1] = vecIn[i].y;
		}
		
		for(int i = 0;i< numM; i++){
		      M[i*dim+0] = ref[i].x;
		      M[i*dim+1] = ref[i].y;
		  
		}
		

	      
	      
		// start with identity as initial transformation
		// in practice you might want to use some kind of prediction here
		Matrix R = Matrix::eye(dim);
		Matrix t(dim,1);

		// run point-to-plane ICP (-1 = no outlier threshold)
		IcpPointToPlane icp(M,numM,dim);
// 		IcpPointToPoint icp(M,numM,dim);
		/*icp.fit(T,numT,R,t, params.icp.IcpInlierDistThreshold->get());*///
		icp.fit(T,numT,R,t, -1);
		
		

		// results
// 		cout << endl << "Transformation results:" << endl;
// 		cout << "R:" << endl << R << endl << endl;
// 		cout << "t:" << endl << t << endl << endl;
		
		getInfo(icp.correspondences,icp.inlierNum, icp.inlierIdx);
		
// 		for(int i =0;i< inlierNum; i++){
// 		   cout<< correspondences[8* i + 4]<<"    "<< M[icp.inlierIdx[i]*2+0] <<endl;
// // 		   cout<< correspondences[8* i + 4]<<"    "<<icp.M_tree->the_data[icp.inlierIdx[i] ][0]<<endl;
// 		}
		
// 		applyTransform(T,  numT, R,t,out);
		
		
		// free memory
		free(M);
		free(T);
	      
		
	    }
	    else{ inlierNum = 0;  }

	
      };
      
      
    
    
    
    
  };
  
  
  
  
  
};


#endif