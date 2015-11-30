
#include <visual_tracking/Tools/tools.h>

// #include <visual_tracking/Parameters/Parameters.h"

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
	    
