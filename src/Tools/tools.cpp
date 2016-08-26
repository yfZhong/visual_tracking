
// Created on: April 12, 2014
//    

//Authors: Yongfeng
#include <visual_tracking/Tools/tools.h>

// #include <visual_tracking/Parameters/Parameters.h"7
#include <visual_tracking/Tools/m_Math.h>

 
 
  void vision::Tools::generateKernelImg(const char* kernelname){
    
	std::string absName = ros::package::getPath("") + "/tmp_file/" + kernelname +".jpg";
	int DOG_kernel_9[7] = {-1,-3,3,4,3,-3,-1};
	int gaussian[7] = {1,2,5,8,5,2,1};
        
	int size= 100;
	int size2= 140;
        cv::Mat kernelv = cv::Mat::zeros(size,size, CV_8UC1);
	cv::Mat kernelh = cv::Mat::zeros(size,size, CV_8UC1);
        cv::Mat kerneld45 = cv::Mat::zeros(size2,size2, CV_8UC1);
	cv::Mat kerneld135 = cv::Mat::zeros(size2,size2, CV_8UC1);
	
//         int kernelv [size ][ size ]; 
// 	int kernelh [size ][ size ]; 
	int l = (size)/2/7;// l=14
	
	Line h(0, Vec2i(0, size/2 ),Vec2i(size-1, size/2) );
	Line v(0, Vec2i(size/2 , 0),Vec2i(size/2, size-1) );
	Line d45(0, Vec2i(0, 0),Vec2i(size2-1, size2-1) );
	Line d135(0, Vec2i(size2-1 , 0),Vec2i(0, size2-1) );
	
	
	int maxv = 32;
	int minv = -30;
	for ( int x = 0; x < size/2; x++ ) { // [0]
                for ( int y = 0; y < size/2; y++ ) {// [1]
		  
		  Vec2i p(x, y);
		  int dist  = m_Math::Point2LineSegDistance(p, h);
		  int dist2 = m_Math::Point2LineSegDistance(p, v);
		  int k,k2;
		 
		  if(dist < l ){k=0;}
		  else if(dist < 3*l ){k=1;}
	          else if(dist < 5*l ){k=2;}
	          else {k=3;}
                 
                  if(dist2 < l ){k2=0;}
		  else if(dist2 < 3*l ){k2=1;}
	          else if(dist2 < 5*l ){k2=2;}
	          else {k2=3;}
		  
		  
		  int value =  gaussian[3 - k2] * DOG_kernel_9[ 3 - k];
		  kernelh.data[ y*size + x]  = float((value-minv))/float((maxv-minv)) *255; 
		  kernelh.data[ y*size + (size-1-x)] =   kernelh.data[ y*size + x] ; 
		  kernelh.data[ (size-1 -y)*size + (size-1-x)]=   kernelh.data[ y*size + x] ; 
		  kernelh.data[ (size-1 -y)*size + x]  =  kernelh.data[ y*size + x];
		  
		  
		  value =  gaussian[3 - k] * DOG_kernel_9[ 3 - k2];
		  kernelv.data[ y*size + x]  = float((value-minv))/float((maxv-minv)) *255; 
		  kernelv.data[ y*size + (size-1-x)] =   kernelv.data[ y*size + x] ; 
		  kernelv.data[ (size-1 -y)*size + (size-1-x)]=   kernelv.data[ y*size + x] ; 
		  kernelv.data[ (size-1 -y)*size + x]  =  kernelv.data[ y*size + x];
		  
		 	  
		  
		  
		}
		
	 }
	 
	 
	 
	 for ( int x = 0; x < size2/2; x++ ) { // [0]
                for ( int y =x; y < size2-x; y++ ) {// [1]
	 
	 
	 	  Vec2i p(x, y);
		  int dist3  = m_Math::Point2LineSegDistance(p, d45);
		  int dist4 = m_Math::Point2LineSegDistance(p, d135);
		  int k3,k4;
		 
		  if(dist3 < l ){k3=0;}
		  else if(dist3 < 3*l ){k3=1;}
	          else if(dist3 < 5*l ){k3=2;}
	          else if(dist3< 7*l ){k3=3;}
	          else{  
// 		        k3=3;
		        continue;
		  }
                 
                  if(dist4 < l ){k4=0;}
		  else if(dist4 < 3*l ){k4=1;}
	          else if(dist4 < 5*l ){k4=2;}
	          else if(dist4 < 7*l ){k4=3;}
		  else{ 
// 		     k4=3;
		     continue;
		  }
		  
		  int value =  gaussian[3 - k4] * DOG_kernel_9[ 3 - k3];
		  
// 		  kerneld45.data[ y*size + x]  = float((value-minv))/float((maxv-minv)) *255; 
// 		  kerneld45.data[ x*size + y] =   kerneld45.data[ (size-1-x)*size + (size-1-y)] 
// 		  =  kerneld45.data[ (size-1-y)*size + (size-1-x)]  =  kerneld45.data[ y*size + x] ;
// 		  
// 		  
// 		  value =  gaussian[3 - k3] * DOG_kernel_9[ 3 - k4];
// 		  kerneld135.data[ y*size + x]  = float((value-minv))/float((maxv-minv)) *255; 
// 		  kerneld135.data[ x*size + y] =   kerneld135.data[ (size-1-x)*size + (size-1-y)] 
// 		  =  kerneld135.data[ (size-1-y)*size + (size-1-x)]  =  kerneld135.data[ y*size + x] ;
		  
		  kerneld45.data[ y*size2 + x]  = float((value-minv))/float((maxv-minv)) *255; 
		  kerneld45.data[ x*size2 + y] =   kerneld45.data[ (size2-1-x)*size2 + (size2-1-y)] 
		  =  kerneld45.data[ (size2-1-y)*size2 + (size2-1-x)]  =  kerneld45.data[ y*size2 + x] ;
		  
		  
		  value =  gaussian[3 - k3] * DOG_kernel_9[ 3 - k4];
		  kerneld135.data[ y*size2 + x]  = float((value-minv))/float((maxv-minv)) *255; 
		  kerneld135.data[ x*size2 + y] =   kerneld135.data[ (size2-1-x)*size2 + (size2-1-y)] 
		  =  kerneld135.data[ (size2-1-y)*size2 + (size2-1-x)]  =  kerneld135.data[ y*size2 + x] ;
		  
		  
		}
		
	 }
	 


	
		
	cv::imshow("kerneld45",kerneld45);
	imwrite( "/home/yvonne/Desktop/pic/line/kerneld45.jpg", kerneld45 );
	cv::waitKey(1);
	
	cv::imshow("kerneld135",kerneld135);
	imwrite( "/home/yvonne/Desktop/pic/line/kerneld135.jpg",kerneld135 );
	cv::waitKey(1);
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	
		
	cv::imshow("kernelh",kernelh);
	imwrite( "/home/yvonne/Desktop/pic/line/kernelh.jpg", kernelh );
	cv::waitKey(1);
	
	cv::imshow("kernelv",kernelv);
	imwrite( "/home/yvonne/Desktop/pic/line/kernelv.jpg", kernelv);
	cv::waitKey(1);
	
	
	
	
    
    
  }
 


	    void vision::Tools::generateKernelImg2(const char* kernelname){
		    int gaussian_kernel_vis[19] = {1,18,153,816,3060,8568,18564,31824,43758,48620,43758,31824,18564,8568,3060,816,153,18,1}; 
		    int DOG_kernel_vis[19] = {-1,-18,-137,-592,-1604,-2744,-2548,208,4290,6292,4290,208,-2548,-2744,-1604,-592,-137,-18,-1};

		    int k_s = 16;
		    
		    double kernel [ 19*k_s ][ 19*k_s ]; 
		    
		    int max_value = std::numeric_limits<float>::min(), min_value = std::numeric_limits<float>::max();
		    
		    
		    
	    // 	 for ( int i = 0; i< 19*k_s; i++){
	    // 	     for ( int j = 0; j< 19*k_s; j++){
	    // 	        kernel [ j ][ i ] = gaussian_kernel_vis[i/k_s]* DOG_kernel_vis[j/k_s] ;
	    // 		 if(max_value <kernel [ i ][ j ] ){max_value = kernel [ i ][ j ] ;}
	    // 	         if(min_value  >kernel [ i ][ j ] ){min_value = kernel [ i ][ j ] ;}
	    // 	     }
	    // 	}
		    
		      for ( int i = 0; i< 19*k_s; i++){
			for ( int j = 0; j< 19*k_s; j++){
			    kernel [ j ][ i ]=1;
			}
		    }
		    
		    for ( int i = 0; i< 19*k_s; i++){
			for ( int j = 0; j< 19* k_s/2; j++){
			    if(i+j< 19*k_s && i-j >0){
				kernel [ i+j ][ i-j ]= DOG_kernel_vis[9+ j/k_s] ;
				kernel [ i-j ][ i+j ]= kernel [ i+j ][ i-j ] ;
	    // 		     if(max_value <kernel [ i+j ][ i-j ] ){max_value = kernel [ i+j ][ i-j ] ;}
	    // 	             if(min_value  >kernel [ i+j ][ i-j ] ){min_value = kernel [ i+j ][ i-j ] ;}
			    }
			}
		    }
		    
		    
		  for ( int i = 0; i< 19*k_s; i++){
			for ( int j = 0; j< 19*k_s/2; j++){
			    if(19*k_s-i+j< 19*k_s && 19*k_s-i-j >0){
	    // 		    kernel [ i+j ][ 19*k_s-i +j ]=  kernel [  19*k_s-i+j ][ 19*k_s-i +j ] *  DOG_kernel_vis[9 + j/k_s] ;
	    // 		    kernel [ i-j ][ 19*k_s-i -j ]=  kernel [ 19*k_s-i+j ][ 19*k_s-i +j ] ;
				
			      
				kernel [ i+j ][ 19*k_s-i +j ]=  kernel [ i+j ][ 19*k_s-i +j ] * gaussian_kernel_vis[9 + j/k_s] ;
				kernel [ i-j ][ 19*k_s-i -j ]=  kernel [ i+j ][ 19*k_s-i +j ] ;
				
				if(max_value <kernel [ i+j ][ 19*k_s-i +j ] ){max_value =kernel [ i+j ][ 19*k_s-i +j ] ;}
				if(min_value  >kernel [ i+j ][ 19*k_s-i +j ] ){min_value = kernel [ i+j ][ 19*k_s-i +j ];}
			      
			    }
			    
			    
			}
		    }
		    
	    // 	NimbroStyleTools::writePGM(kenel, "/home/yvonne/master_thesis/data/kernel1");
		    
		    unsigned char *kernel_vis = new unsigned char[19*k_s*19*k_s];
		    memset(kernel_vis, 0, sizeof(unsigned char)*19*k_s*19*k_s);  
		    for ( int i = 0; i<  19*k_s; i++){//  satuate data to range [0, 255]
			for ( int j = 0; j<  19*k_s; j++){
			  
			    kernel_vis[j* 19*k_s + i]= double(kernel[j][i]- min_value)/ double(max_value-min_value)* 255;

		      }
		    }
		    cv::Mat gray_kernel(19*k_s, 19*k_s, CV_8UC1, kernel_vis);
		    
		    std::string absName = ros::package::getPath("") + "/tmp_file/" + kernelname +".jpg";
		    cv::imwrite( absName, gray_kernel );
	    
	      }
	      
	      
	    //generate seeds for random variables
	      void vision::Tools::seed_rand()
	      {
		//Seed random number generator with current microseond count
		timeval temp_time_struct;
		gettimeofday(&temp_time_struct,NULL);
		srand(temp_time_struct.tv_usec);
	      }
	      
	      
	      bool vision::Tools::findPointInVector(cv::Point3f p, std::vector<cv::Point3f> points){
      
		  for (unsigned int i = 0;i< points.size(); i++){
		    if(p.x == points[i].x &&  p.y == points[i].y && p.z == points[i].z){
		      return true;
		    }
		  }
		  return false;
	      
	      }
	      
	    bool vision::Tools::findPointInVector(cv::Point3f p, std::vector<tf::Vector3>  points){
      
		  for (unsigned int i = 0;i< points.size(); i++){
		    if(p.x == points[i].x() &&  p.y == points[i].y() && p.z == points[i].z()){
		      return true;
		    }
		  }
		  return false;
	      
	      }
    
	    void vision::Tools::quaternionNormalize(geometry_msgs::Quaternion &a){
		double len = sqrt(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w);
		len = std::max(len, 0.000001); 
		a.x /=len; 
		a.y /=len; 
		a.z /=len;
		a.w /=len;

	    }
	      
	    geometry_msgs::Quaternion vision::Tools::quaternionMultiplication(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2){
		    geometry_msgs::Quaternion q;
		  
		    q.x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
		    q.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
		    q.z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
		    q.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
    // 	    	      std::cout<< q.x<<", "<< q.y<<", "<< q.z<<", "<< q.w<<std::endl;
		  quaternionNormalize(q);
		  return q;
	    }

	   
	   
	   
	   
	  vision::Tools tools;    
	    
