// #include "findLines.h"

#include <visual_tracking/LineMatcher/findLines.h>

FindLines::FindLines()/*:filtered_data(H * W,0)*/{
    debug_line_detector = 0;
    m_Top = 0;
    fpsData = 30;
    
    H = params.camera.height->get();
    W = params.camera.width->get();
    siX = params.camera.widthUnDistortion->get();
    siY = params.camera.heightUnDistortion->get();
    DIAGONAL_ANGLE_RNAGE = M_PI/2.0 /9.0 ;//10 degree
    
    H_data = new int[W*H]; 
    V_data = new int[W*H]; 
    diagonal_data45 = new int[W*H]; 
    diagonal_data135 = new int[W*H]; 
   
    MeasConf = 0;

}


FindLines::~FindLines(){      

   delete[] H_data; delete[] V_data; delete[] diagonal_data135; delete[] diagonal_data45;
  
}

		
void FindLines::findSkeletons(FrameGrabber & CamFrm){
  

    m_Top= CamFrm.m_Top;
//     boost::timer::cpu_timer timer;
    // apply line filters to detect lines in different orientations. Line point candidates would have higher respondses.
    applyLineFilter(/* in */CamFrm.Brightness_Channel, /* out */CamFrm.weightedWhiteValues);
     
//      fpsData = (0.9 * fpsData) + (0.1 * (1000000000l / timer.elapsed().wall));
//      cout<< fpsData<<endl;
 

    
    RetrieveSkeleton(/* in */CamFrm.fieldConvexHullMat, /* in */CamFrm.weightedWhiteValues, /* out */ detectedPoins );
    

// 	      cv::Mat skeletonPixels1(cv::Size(W,H),CV_8UC3,cv::Scalar(150, 150, 150));
// 
// 	      //draw detected points
// 	      for ( unsigned int i = 0; i < detectedPoinsWithType.size() ; i++ ) { 
// 		  cv::circle(skeletonPixels1, detectedPoinsWithType[i].first, 2, cv::Scalar(0, 10, 200),-1);
// 	      }
    
    
    removeObstacleBoarder (/* in */ CamFrm.ObstacleConvexHull  ,/* in_out */ detectedPoinsWithType );
//     
//     
// 	    cv::Mat skeletonPixels2(cv::Size(W,H),CV_8UC3,cv::Scalar(150, 150, 150));
// 
// 	      //draw detected points
// 	      for ( unsigned int i = 0; i < detectedPoinsWithType.size() ; i++ ) { 
// 		  cv::circle(skeletonPixels2, detectedPoinsWithType[i].first, 2, cv::Scalar(0, 10, 200),-1);
// 	      }
//     
//      
//      regionGlow( /* in */CamFrm.weightedWhiteValues , visited, /* in */CamFrm.fieldConvexHull, detectedPoinsWithType );
//      removeObstacleBoarder (/* in */ CamFrm.ObstacleConvexHull  ,/* in_out */ detectedPoinsWithType );
    
                
// 	      cv::Mat skeletonPixels3(cv::Size(W,H),CV_8UC3,cv::Scalar(150, 150, 150));
// 
// 	      //draw detected points
// 	      for ( unsigned int i = 0; i < detectedPoinsWithType.size() ; i++ ) { 
// 		  cv::circle(skeletonPixels3, detectedPoinsWithType[i].first, 2, cv::Scalar(0, 10, 200),-1);
// 	      }

    
     memset(CamFrm.skeletonPixelMatrix, 0, sizeof(CamFrm.skeletonPixelMatrix[0][0]) * H * W);
     CamFrm.MaximunNodeNum = detectedPoinsWithType.size();
      for(unsigned int i =0; i< detectedPoinsWithType.size(); ++i ){
	  cv::Point p = detectedPoinsWithType[i].first;
	  CamFrm.skeletonPixelMatrix[p.y ] [ p.x ] = detectedPoinsWithType[i].second + CamFrm.MaximunNodeNum;
      }
    
    

    
    
    
    
//     imshow(" skeletonPixels1  ", skeletonPixels1);
//     waitKey( 1);
    
       
    

    
   
    if(debug_line_detector){
      
      	tools.cvshowImg(CamFrm.weightedWhiteValues, "weightedWhiteValues");
        //the output file will be saved in package "tmp_file"
	tools.writePGM(CamFrm.weightedWhiteValues, "weightedWhiteValues");
	tools.writeDAT(CamFrm.weightedWhiteValues, "weightedWhiteValues.dat");
	
        tools.cvshowImg(CamFrm.weightedWhiteValues, "smothWhiteValues");
    }
    
   

}




void FindLines::applyLineFilter(/*in*/cv:: Mat Brightness_Channel,/*out*/std::vector< float > & weightedWhiteValues){
    float div_pct1 = params.line.Div1->get();
    float div_pct2 = params.line.Div2->get();//from  top to bottom [0-div_pct1, div_pct1-div_pct2, div_pct2-1]
  
    int offset_size_9 = 9; int half_size_9 = 4;
    int offset_v_9[9][2]  = {{0,-4},{0,-3},{0,-2},{0,-1},{0,0},{0,1},{0,2},{0,3},{0,4}}; //offset for vertical and horizontal kernel
    int offset_h_9[9][2]  = {{-4,0},{-3,0},{-2,0},{-1,0},{0,0},{1,0},{2,0},{3,0},{4,0}}; //offset for vertical and horizontal kernel
    int offset_d45_9[9][2]  = {{4,-4},{3,-3},{2,-2},{1,-1},{0,0},{-1,1},{-2,2},{-3,3},{-4,4}}; //offset for diagonal kernel
    int offset_d135_9[9][2]  = {{-4,-4},{-3,-3},{-2,-2},{-1,-1},{0,0},{1,1},{2,2},{3,3},{4,4}}; //offset for diagonal kernel
    
    int DOG_kernel_9[9] = {-3,-5,1,4,6,4,1,-5,-3}; //difference of gaussian kernel
 	
    memset(H_data, 0, sizeof(int)*W*H);  
    memset(V_data, 0, sizeof(int)*W*H);   
    memset(diagonal_data45, 0, sizeof(int)*W*H);   
    memset(diagonal_data135, 0, sizeof(int)*W*H); 
    
    
    double max_v =std::numeric_limits<double>::min();
//     min_v =std::numeric_limits<double>::max();
    
    
    
     int kernel_type = 3; // 3 times larger than the first kernel 
	 //apply the dfg kernel
         for ( int i = 0; i< W; i++){  
	    for ( int j = H-m_Top -1 ; j< H; j++){   
// 	    for ( int j = 0; j< H; j++){   
		if(j<div_pct1 * H ){
		    kernel_type = 1; // small kernel for upper part image
		}
		else if(j<div_pct2 * H){
		    kernel_type = 2; // middle size kernel for middle part of the image
		}
		else{
		    kernel_type = 3; // largest kernel for lower part of the image
		}
		
		
	        if(i<half_size_9*kernel_type+1 || i>= W -half_size_9*kernel_type-1 || j< half_size_9*kernel_type+1 || j>= H -half_size_9*kernel_type-1
		  || Brightness_Channel.data[ j *W +i] < params.line.MinBrightnessValue->get()){
		    weightedWhiteValues[(j)*W + i] = 0;
		}
		else{
		    int tmp_v = 0, tmp_h = 0, tmp_d45 =0 ,tmp_d135=0;
		    
		    for(int os =0; os < offset_size_9; os++){ //os: offset size
		          int currentpos = j*W + i; 
			  int delta_x,delta_y;
			  int new_pose;
			  delta_x = 0;
			  delta_y = offset_v_9[os][1] *kernel_type;
			  new_pose =currentpos + delta_y * W + delta_x;
			  tmp_h +=   DOG_kernel_9[os]* ((Brightness_Channel.data[new_pose] <<1) + Brightness_Channel.data[new_pose -1] +Brightness_Channel.data[new_pose +1] );
			  
			  
			  delta_x = offset_h_9[os][0]*kernel_type;
			  delta_y = 0;
			  new_pose = currentpos + delta_y * W + delta_x;
			  tmp_v +=  DOG_kernel_9[os]* ((Brightness_Channel.data[new_pose] <<1) + Brightness_Channel.data[new_pose +W]  + Brightness_Channel.data[new_pose -W]);
			  
			  delta_x = offset_d135_9[os][0]*kernel_type;
			  delta_y = offset_d135_9[os][1]*kernel_type;
			  new_pose = currentpos + delta_y * W + delta_x;

			  tmp_d45 +=  DOG_kernel_9[os]* ((Brightness_Channel.data[new_pose] <<1) + Brightness_Channel.data[new_pose + W -1] +Brightness_Channel.data[new_pose - W +1]);
			  
			  
			  delta_x = offset_d45_9[os][0]*kernel_type;
			  delta_y = offset_d45_9[os][1]*kernel_type;
			  new_pose = currentpos + delta_y * W + delta_x;

			  tmp_d135 +=  DOG_kernel_9[os]* ((Brightness_Channel.data[new_pose] <<1) + Brightness_Channel.data[new_pose - W -1] +Brightness_Channel.data[new_pose + W +1]);  
		    }
		    
		    H_data[j*W + i] = std::max(0,tmp_h );
		    V_data[j*W + i] = std::max(0,tmp_v );
		    diagonal_data45[j*W + i] = std::max(0,tmp_d45 );
		    diagonal_data135[j*W + i] = std::max(0,tmp_d135 );
		    

			  weightedWhiteValues[(j)*W + i] = sqrt( pow(H_data[j*W + i],2)+ pow(V_data[j*W + i],2)+ 
								 pow(diagonal_data45[j*W + i],2)+ pow(diagonal_data135[j*W + i],2));



		}
		
		    if(max_v <weightedWhiteValues[j*W + i]){max_v = weightedWhiteValues[(j)*W + i];}
// 		    if(min_v >weightedWhiteValues[j*W + i]){min_v = weightedWhiteValues[(j)*W + i];}
		
	    }
	   
	   
	}
	  
// 	  std::vector<float > test(ORG_IMAGE_HEIGHT * ORG_IMAGE_WIDTH,0);
// 	   for ( int i = 0; i< W; i++){  
// 	    for ( int j =0 ; j< H; j++){  
// 	       test[(H-1-j)*W + i] = frame_raw_data[(j*W + i)*3];
// 	      
// 	    }
// 	     
// 	  }
//         tools::cvshowImg(test, "test");

	 for ( int j =0; j< H; j++){
	     for ( int i = 0; i< W; i++){
	            if(j < H- m_Top -1 ){
		       weightedWhiteValues[j*W + i] = 0;
		      
		    }
		    else{
		     weightedWhiteValues[j*W + i] = (weightedWhiteValues[j*W + i]-0.0)/(max_v-0.0) *255;  
		
		    }
	     }
	   
	}
	 
    
    
// //     tools.cvshowImg(weightedWhiteValues, "weightedWhiteValues");
  
  
  
}


void FindLines::RetrieveSkeleton (/* in */cv:: Mat & fieldConvexHull, /* in */ std::vector<float > &  matrix , /* out */  std::vector<cv::Point> &detectedPoins ){

//             Mat out = Mat::zeros(fieldConvexHull.size(), CV_8UC1);
	     visited = Mat::zeros(fieldConvexHull.size(), CV_8UC1);
	    
	    detectedPoins.clear();
	    detectedPoinsWithType.clear();
            // * Loop through rows. * //
            for ( int x = 1; x < W - 1; x++ ) { // [0]
                // * Loop through columns. * //
                // * Process the pixels under m_FieldBoundary[x]-params.field.FieldBoarder->get()/2 for skeleton. * //
                for ( int y = H-m_Top; y < H; y++ ) {// [1]
		    if ( fieldConvexHull.at<uchar>(y,x ) == 0 ) { // [2]  //at<uchar>(i,j) i: rows, j: cols
                       continue;
                    }
                    register unsigned int vote = 0;
                    register int val = matrix[ y * W + x ];
		    if ( val < params.line.MinSkeletonValue->get() ) { // [2]
                       continue;
                    }
                    else { // [2]
                        if ( val <= matrix[ y * W + x - 1 ] ) { // [3]
                            vote++;
                        }
                        if ( val <= matrix[ y * W + x + 1 ] ) { // [3]
                            vote++;
                        }{ // [3]
                            vote++;
                        }
                        if ( val <= matrix[ (y - 1) * W + x ] ) 
                        if ( val <= matrix[ (y + 1) * W + x ] ) { // [3]
                            vote++;
                        }
                        if ( vote <= 3 ) { // [3]
                            if ( val <= matrix[ (y - 1) * W + x - 1 ] ) { // [4]
                                vote++;
                            }
                            if ( val <= matrix[ (y + 1) * W + x - 1 ] ) { // [4]
                                vote++;
                            }
                            if ( vote < 3 ) { // [4]
                                if ( val <= matrix[ (y - 1) * W + x + 1 ] ) { // [5]
                                    vote++;
                                }
                                if ( val <= matrix[ (y + 1) * W + x + 1 ] ) { // [5]
                                    vote++;
                                }
                                if ( vote < 3 ) { // [5]
				    
				     detectedPoins.push_back(cv::Point(x , y)); 
				     visited.at<uchar>(y,x) = 255;
				     detectedPoinsWithType.push_back(make_pair(cv::Point(x , y), vote)); 
// 				     skel.push_back(x); skel.push_back(y); skel.push_back(UNIT + vote); 
                                }// [5]
                            }// [4]

                        }// [3]
                    } // [2]
                } // [1]
            } // [0]
}

// ****************************************************************************** //
void FindLines::removeObstacleBoarder (/* in */ vector< vector <cv::Point > > ObstacleContours  ,/* in_out */ std::vector<pair<cv::Point, int> > & _detectedPoinsWithType ){
  
  
       std::vector<pair<cv::Point, int> > tmp = _detectedPoinsWithType;
       _detectedPoinsWithType.clear();
      
      for ( unsigned int i = 0; i <tmp.size() ; i++ ) { //[1]
	         cv::Point res= _detectedPoinsWithType[i].first ;
	         bool remove = false;
	
	         for(unsigned int j =0; j< ObstacleContours.size(); ++j){
		      double downDistance2Field = pointPolygonTest(ObstacleContours[j], res, true);
		      if(downDistance2Field >params.line.ObstacleLineDistance->get()){ 
			  remove = true; 
			  break;
		      } 
		 }
		  
		  if(remove == false){
		      _detectedPoinsWithType.push_back(tmp[i]); 
		  }
      }//end of [1]
}// END of pureSkeleton METHOD


void FindLines::regionGlow( std::vector<float > &  matrix , Mat & visited, cv:: Mat & _fieldConvexHull, std::vector<pair<cv::Point, int> > & _detectedPoinsWithType ){
  
  int size = _detectedPoinsWithType.size();
  for( int i =0; i< size; ++i){
//     int type = _detectedPoinsWithType[i].second;
     
        cv::Point seed = _detectedPoinsWithType[i].first;
        push_back_neighbors(seed.x, seed.y);
	while(!neighbors.empty()){
		Point pixel = neighbors.front();
		neighbors.pop();

		//first check if there already is a maximum in neighborhood
		if(visited.at<uchar>(pixel.y ,pixel.x) == 255){
                 continue;
		}
		//else check neighborhood of pixel if equal to (i,j)
		else if (visited.at<uchar>(pixel.y ,pixel.x) == 0 && _fieldConvexHull.at<uchar>(pixel.y ,pixel.x) >0
		       && matrix[ pixel.y * W + pixel.x ] >= matrix[ seed.y * W + seed.x ] -params.line.LocalOptimalRange->get()
		       && matrix[ pixel.y * W + pixel.x ] > params.line.MinSkeletonValue->get()){
                          visited.at<uchar>(pixel.y ,pixel.x) = 255;
			  _detectedPoinsWithType.push_back(make_pair(cv::Point( pixel.x, pixel.y ) , 2) );
			  push_back_neighbors(pixel.x ,pixel.y);
		  
		}
    }
    
  }

}




// ****************************************************************************** //
void FindLines::Smooth ( /* in_out */ std::vector<float > & matrix  ){
            std::vector<float > tmp(H * W,0);
	    // * Low-pass 1st Time. * // 
	    for ( int y = m_Top; y > -1; y-- ) {
		for ( int x = 1; x < W - 1; x++ ) {
		    tmp[ y * W + x ] = ( matrix[ y * W + x ] *2 ) + ( matrix[ y * W + x - 1 ] + matrix[ y * W + x + 1 ] );
		}
	    }

	    for ( int y =  m_Top-1  ; y > 0; y-- ) {
		for ( int x = 0; x < W; x++ ) {
		    matrix[ y * W + x ] = ( ( tmp[ y * W + x ] * 2 ) + ( tmp[ (y - 1) * W +x ] + tmp[ (y + 1) * W + x ] ) ) /16.0;
		}
	    }
	    
	    // * Low-pass 2nd Time. * //
	    for ( int y = m_Top; y > -1 ; y-- ) {
		for ( int x = 1; x < W - 1; x++ ) {
		    tmp[ y * W + x ] = ( matrix[ y * W + x ] * 2 ) + ( matrix[ y * W + x - 1 ] + matrix[ y * W + x + 1 ] );
		}
	    }

	    for ( int y = m_Top - 1; y > 0; y-- ) {
		for ( int x = 0; x < W; x++ ) {

		    matrix[ y * W + x ] = ( ( tmp[ y * W + x ] *2  ) + ( tmp[ (y - 1) * W + x ] + tmp[ (y + 1) * W + x ] ) ) / 16;
		}
	    }

	    return;

	} // END of Smooth METHOD
	
// ****************************************************************************** //





void FindLines::push_back_neighbors(int i, int j){ //i: cols  j: rows

			if((i-1) >= 0  && (j-1) > 0)
				neighbors.push(Point(i-1,j-1));
			if((i-1) > 0)
				neighbors.push(Point(i-1,j));
			if(((i-1) > 0) && ((j+1) < H))
				neighbors.push(Point(i-1,j+1));

			if((j-1) > 0)
				neighbors.push(Point(i,j-1));
			if((j+1) < H)
				neighbors.push(Point(i,j+1));

			if(((i+1) < W) && ((j-1) > 0))
				neighbors.push(Point(i+1,j-1));
			if((i+1) < W)
				neighbors.push(Point(i+1,j));
			if(((i+1) < W) && ((j+1) < H))
				neighbors.push(Point(i+1,j+1));

	}

// bool FindLines::check_neighborhood(int i, int j, Mat& img, Mat& localmax, Mat& visited,Mat & fieldConvexHull){
// 	push_back_neighbors(i,j);
// 
// 	while(!neighbors.empty()){
// 		Point2i pixel = neighbors.front();
// 		neighbors.pop();
// 
// 		//first check if there already is a maximum in neighborhood
// 		if(localmax.at<uchar>(pixel.x,pixel.y) == 255){
// // 			visited.at<uchar>(pixel.x,pixel.y) = 255;
// 			return true;
// 		}
// 		//else check neighborhood of pixel if equal to (i,j)
// // 		else if (img.at<uchar>(i,j) == img.at<uchar>(pixel.x,pixel.y) && visited.at<uchar>(pixel.x,pixel.y) == 0 && fieldConvexHull.at<uchar>(pixel.x,pixel.y) >0){
// 		else if (visited.at<uchar>(pixel.x,pixel.y) == 0 && fieldConvexHull.at<uchar>(pixel.x,pixel.y) >0
// // 		       && img.at<uchar>(pixel.x,pixel.y)  >  params.line.MinSkeletonValue->get() 
// 		       && img.at<uchar>(pixel.x,pixel.y)  == (img.at<uchar>(i,j))){
// 		       
// 		       
// // 			if(img.at<uchar>(pixel.x,pixel.y) > (img.at<uchar>(i,j))){// -params.line.LocalOptimalRange->get()
// // 			    if(check_neighborhood(pixel.x, pixel.y, img, localmax, visited, fieldConvexHull)==false){
// // 				localmax.at<uchar>(pixel.x,pixel.y) = img.at<uchar>(i,j); //try to weight local plateaus
// //   // 				     out.at<uchar>(i,j) = 255;
// // 				detectedPoins2.push_back(cv::Point(j,i));
// // 			      
// // 			    };
// // 
// // 			}
// 			
//                           visited.at<uchar>(pixel.x,pixel.y) = 255;
// // 			  localmax.at<uchar>(pixel.x,pixel.y) = 255;
// // 			  detectedPoins2.push_back( cv::Point(pixel.y,pixel.x));
// 			  push_back_neighbors(pixel.x, pixel.y);
// 		  
// 		}
// 	}
// 
// 	return false;
// }
// 
// void FindLines::local_maxima(Mat& in, Mat & fieldConvexHull, Mat& out){//at<uchar>(i,j) i: rows, j: cols
// 	if (in.type() != CV_8UC1)return;
// 	cols_ = W;
// 	rows_ = H;
// 	out = Mat::zeros(in.size(), CV_8UC1);
// 	Mat visited = Mat::zeros(in.size(), CV_8UC1);
// 	std::queue<Point2i> tovisit;
// 	Mat tmp = in.clone();
//         
// 	detectedPoins2.clear();
// 
// 	for(int i = 0; i < tmp.rows; i++){
// 		for (int j = 0; j < tmp.cols; j++){
// 
// 			//Compare to neighborhood
// 			//Case 1:  i < N8
// 			if ( 	visited.at<uchar>(i,j) == 0 && fieldConvexHull.at<uchar>(i,j) >0 && tmp.at<uchar>(i,j) > params.line.MinSkeletonValue->get() &&
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(std::max(0,i-1),std::max(0,j-1)) &&
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(std::max(0,i-1),j) &&
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(std::max(0,i-1),std::min(j+1,tmp.cols-1)) &&
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(i,std::max(0,j-1)) &&
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(i,std::min(j+1,tmp.cols-1)) &&
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(std::min(i+1,tmp.rows-1),std::max(0,j-1)) &&
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(std::min(i+1, tmp.rows-1),j) &&
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(std::min(i+1, tmp.rows-1),std::min(j+1,tmp.cols-1))
// 
// 			){
// 				visited.at<uchar>(i,j) = 255;
// 				out.at<uchar>(i,j) = 255;
// 				detectedPoins2.push_back( cv::Point(j,i));
// 			}
// 			else if(visited.at<uchar>(i,j) == 0  && fieldConvexHull.at<uchar>(i,j) >0 && tmp.at<uchar>(i,j) > params.line.MinSkeletonValue->get() &&
// 					tmp.at<uchar>(i,j) >= tmp.at<uchar>(std::max(0,i-1),std::max(0,j-1))&&
// 					tmp.at<uchar>(i,j) >= tmp.at<uchar>(std::max(0,i-1),j) &&
// 					tmp.at<uchar>(i,j) >= tmp.at<uchar>(std::max(0,i-1),std::min(j+1,tmp.cols-1)) &&
// 					tmp.at<uchar>(i,j) >= tmp.at<uchar>(i,std::max(0,j-1)) &&
// 					tmp.at<uchar>(i,j) >= tmp.at<uchar>(i,std::min(j+1,tmp.cols-1)) &&
// 					tmp.at<uchar>(i,j) >= tmp.at<uchar>(std::min(i+1,tmp.rows-1),std::max(0,j-1)) &&
// 					tmp.at<uchar>(i,j) >= tmp.at<uchar>(std::min(i+1, tmp.rows-1),j) &&
// 					tmp.at<uchar>(i,j) >= tmp.at<uchar>(std::min(i+1, tmp.rows-1),std::min(j+1,tmp.cols-1)) &&
// 
// 					(
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(std::max(0,i-1),std::max(0,j-1)) ||
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(std::max(0,i-1),j) ||
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(std::max(0,i-1),std::min(j+1,tmp.cols-1)) ||
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(i,std::max(0,j-1)) ||
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(i,std::min(j+1,tmp.cols-1)) ||
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(std::min(i+1,tmp.rows-1),std::max(0,j-1)) ||
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(std::min(i+1, tmp.rows-1),j) ||
// 					tmp.at<uchar>(i,j) > tmp.at<uchar>(std::min(i+1, tmp.rows-1),std::min(j+1,tmp.cols-1)))
// 
// 
// 			){
// 				visited.at<uchar>(i,j) = 255;
// 				if(check_neighborhood(i,j,tmp,out,visited, fieldConvexHull) == false){
// 					out.at<uchar>(i,j) = tmp.at<uchar>(i,j); //try to weight local plateaus
// // 				        out.at<uchar>(i,j) = 255;
// 				        detectedPoins2.push_back(cv::Point(j,i));
// 				  
// 				}
// 
// 			}
// 		}
// 
// 	}
// 
// // 	threshold(out,out,100,255,THRESH_TOZERO);
// 
// 
// }




	
	

// ****************************************************************************** //	
// ****************************************************************************** //	
//For Particles Filters
// ****************************************************************************** //
void FindLines::findLines(FrameGrabber & CamFrm){
  

    m_Top= CamFrm.m_Top;

    // apply line filters to detect lines in different orientations. Line point candidates would have higher respondses.
    applyLineFilter(/* in */CamFrm.Brightness_Channel, /* out */CamFrm.weightedWhiteValues);
    
    RetrieveSkeleton(/* in */CamFrm.fieldConvexHullMat, /* in */CamFrm.weightedWhiteValues, /* out */ detectedPoins );
//     removeObstacleBoarder (/* in */ CamFrm.binaryImgs[BLACK_C]  ,/* in_out */ detectedPoins );
   
    undistortedDetectedPoints.clear();

    distortionModel.UndistortP(detectedPoins,undistortedDetectedPoints);
        
    findhoughLines(undistortedDetectedPoints);
    
    MergeLinesOnImg(LinesOnImg, LinesOnImg_After_Merged);
    
    MeasConf = getMeasurementConfidence(LinesOnImg_After_Merged);

    
    if(debug_line_detector){
      
      	tools.cvshowImg(CamFrm.weightedWhiteValues, "weightedWhiteValues");
        //the output file will be saved in package "tmp_file"
	tools.writePGM(CamFrm.weightedWhiteValues, "weightedWhiteValues");
	tools.writeDAT(CamFrm.weightedWhiteValues, "weightedWhiteValues.dat");
	
        tools.cvshowImg(CamFrm.weightedWhiteValues, "smothWhiteValues");
    }
    
   

}
	
void FindLines::findhoughLines(std::vector<cv::Point> & undistortedPoints){
  
      LinesOnImg.clear();


      cv::Mat gray=Mat::zeros(Size(siX, siY), CV_8UC1);
      
      for ( unsigned int i = 0; i <undistortedPoints.size() ; i++ ) { 
	  gray.at<uchar>(undistortedPoints[i].y, undistortedPoints[i].x ) = 250;	
      }
      
      vector<cv::Vec4i> hough_lines;
      int rho = params.line.HoughLineRho->get(),
          threshold= params.line.HoughLineVoteThreshold->get(),
	  minLineLength= params.line.HoughLineMinLineLength->get(),
	  maxLineGap = params.line.HoughLinemaxLineGap->get();
      double theta = CV_PI * (params.line.HoughLineTheta->get())/180.0 ;
      
      cv::HoughLinesP(gray, hough_lines,  rho, theta, threshold, minLineLength, maxLineGap );
//       cout<<"Line Num: "<< lines.size()<<endl;
      
//       rho – Distance resolution of the accumulator in pixels.
//       theta – Angle resolution of the accumulator in radians.
//       threshold – Accumulator threshold parameter. Only those lines are returned that get enough votes ( >\texttt{threshold} ).
//       minLineLength – Minimum line length. Line segments shorter than that are rejected.
//       maxLineGap – Maximum allowed gap between points on the same line to link them.
      
      int id = 0;

      //storing lines
      for( size_t i = 0; i < hough_lines.size(); i++ )
	{

// 	    float l = sqrt(pow(hough_lines[i][3] - hough_lines[i][1],2) + pow(hough_lines[i][2] -hough_lines[i][0], 2) );
	    Vec2i  s(hough_lines[i][0],hough_lines[i][1] );
	    Vec2i  e(hough_lines[i][2],hough_lines[i][3] );
	    Line line0( id++, s, e) ;
	   
	    LinesOnImg.push_back(line0);
      }
  
}


Line FindLines::MergeTwoLinesOnImg(Line line1, Line line2, int id)
{    
      float ang = 0;
      Vec2f  line1MidP((line1.s[0] + line1.e[0])/2.0, (line1.s[1] + line1.e[1])/2.0);
      Vec2f  line2MidP((line2.s[0] + line2.e[0])/2.0, (line2.s[1] + line2.e[1])/2.0);
      float min_value = 0.000001;
      float r = line1.len/(std::max(line1.len + line2.len, min_value));
  
      Vec2f newMidP(r*line1MidP[0] + (1-r) * line2MidP[0] , r*line1MidP[1] + (1-r) * line2MidP[1] );

      //(-M_PI/2, M_PI/2)
      if(fabs(line1.ang -line2.ang)>M_PI/2.0 ){  
	if(line1.ang<0 ){ line1.ang = line1.ang +M_PI;}
	else{ line2.ang = line2.ang + M_PI;}
      }
      ang = r*line1.ang + (1-r) *line2.ang; 
      if(ang >=M_PI/2.0){ang = ang - M_PI; }
      
      
      
      float k = m_Math::getKValueFromAngle(ang);
      vector<Vec2f> pProject; 
      Vec2f pp;
      m_Math::GetProjectivePoint( Vec2f(line1.s[0],line1.s[1]), k, newMidP,  pp); pProject.push_back(pp);
      m_Math::GetProjectivePoint( Vec2f(line1.e[0],line1.e[1]), k, newMidP,  pp); pProject.push_back(pp);
      m_Math::GetProjectivePoint( Vec2f(line2.s[0],line2.s[1]), k, newMidP,  pp); pProject.push_back(pp);
      m_Math::GetProjectivePoint( Vec2f(line2.e[0],line2.e[1]), k, newMidP,  pp); pProject.push_back(pp);
      
      Vec2f startf = newMidP, endf = newMidP;
      
      for(unsigned int i = 0; i< pProject.size(); ++i){
	   
	    if(fabs(ang - M_PI/2.0)<0.000001 ){//vertical line
		if(pProject[i][1]< startf[1]){ startf[0] =pProject[i][0];startf[1] = pProject[i][1]; }
		if(pProject[i][1]>   endf[1]){   endf[0] =pProject[i][0];  endf[1] = pProject[i][1]; }

	    }
	    else
	    {  
	      if(pProject[i][0]< startf[0]){ startf[0] = pProject[i][0]; startf[1] = pProject[i][1]; }
	      if(pProject[i][0]>   endf[0]){   endf[0] = pProject[i][0];   endf[1] = pProject[i][1]; }
	      
	    }
        }  
       
        Vec2i start(startf[0],startf[1]), end (endf[0],endf[1]);
        float length = m_Math::TwoPointDistance(start, end);
      
      Line Merged_line(id, length, start, end, k, ang, 1.0) ;
  
      return Merged_line;
  
}
void FindLines::MergeLinesOnImg(std::vector< Line > m_LineBuffer_Before_Merge, std::vector< Line >& m_LineBuffer_After_Merge){
    
      if( m_LineBuffer_Before_Merge.size()<1){return;}
//    cv::Mat vis_houghLines(siY,siX,CV_8UC3,cv::Scalar(150, 150, 150));
      m_LineBuffer_After_Merge.clear();
      int AngleToMerge = params.line.AngleToMergeImg->get();
      int MinLineSegDistance = params.line.MinLineSegDistanceImg->get();
      int MinProjectedDistance =params.line.MinProjectedDistanceImg->get();
      
//       cout<<"Merge Line: ";
      std::vector< Line >  m_LineBuffer_Before_Merge_tmp = m_LineBuffer_Before_Merge ;
      std::vector< Line >  m_LineBuffer_After_Merge_tmp;
//       int loopNum=0;
      while (true){
//           loopNum++;
	  int id=0;
	  for(unsigned int i=0; i < m_LineBuffer_Before_Merge_tmp.size(); ++i ){
	      if(m_LineBuffer_Before_Merge_tmp[i].id !=-1){
		    Line a = m_LineBuffer_Before_Merge_tmp[i];
		    for(unsigned int j= i + 1; j < m_LineBuffer_Before_Merge_tmp.size(); ++j ){
			if(m_LineBuffer_Before_Merge_tmp[j].id != -1 ){
			  Line b = m_LineBuffer_Before_Merge_tmp[j];
			  Vec2i Midpoint_a((a.s[0]+a.e[0])/2.0, (a.s[1]+a.e[1])/2.0);
			  Vec2i Midpoint_b((b.s[0]+b.e[0])/2.0, (b.s[1]+b.e[1])/2.0);
			  if(m_Math::AngDif( a.ang, b.ang) < AngleToMerge
			    &&  m_Math::getShortestDistance(a, b) <MinLineSegDistance
			    &&  m_Math::GetProjectiveDistance(Midpoint_a, b) < MinProjectedDistance
			    &&  m_Math::GetProjectiveDistance(Midpoint_b, a) < MinProjectedDistance){
			     Line c = MergeTwoLinesOnImg(a, b, id++);
			     if( c.s != c.e ){
			      a = c;
			      m_LineBuffer_Before_Merge_tmp[j].id = -1;}
			    
//                             cout<<"Merge Line: "<<a.id<<" and "<< b.id<<", "<<endl;
// 			    cout<<"        "<< a.ang_w<<"  "<< b.ang_w<<"  "<< a.k_w<<"  "<< b.k_w <<"    "<<getShortestDistance_WorldCord(a, b)
// 			    <<"  " <<GetProjectiveDistance_WorldCord(Vec2f((a.s_w[0]+a.e_w[0])/2.0,(a.s_w[1]+a.e_w[1])/2.0), b)<<endl;
			  
			  }
			}
		      
		    }
		    m_LineBuffer_After_Merge_tmp.push_back(a);

	      }
	  
	  }
	  if(m_LineBuffer_After_Merge_tmp.size() == m_LineBuffer_Before_Merge_tmp.size() ){break;}
	  m_LineBuffer_Before_Merge_tmp = m_LineBuffer_After_Merge_tmp;
	  m_LineBuffer_After_Merge_tmp.clear();
      }
//      cout<<endl;
//      cout <<"loopNum  "<<loopNum<<endl;
//      cout<<endl;
     m_LineBuffer_After_Merge = m_LineBuffer_After_Merge_tmp;
     
//       cv::line( vis_houghLines, cv::Point(m_LineBuffer_tmp[i].s[0], m_LineBuffer_tmp[i].s[1] ), cv::Point(m_LineBuffer_tmp[i].e[0],m_LineBuffer_tmp[i].e[1]), 
// 		cv::Scalar( m_LineBuffer_tmp[i].r, m_LineBuffer_tmp[i].g, m_LineBuffer_tmp[i].b), 4, 0 );
	      
//       std::ostringstream ss0;
//       ss0 << i;
//       cv::putText(vis_houghLines,ss0.str(), cv::Point((m_LineBuffer_tmp[i].s[0]+m_LineBuffer_tmp[i].e[0])/2-20,
// 						      (m_LineBuffer_tmp[i].s[1]+m_LineBuffer_tmp[i].e[1])/2 -20),
// 					    cv::FONT_HERSHEY_TRIPLEX,1,cv::Scalar(0,0,0),2);
      //vis**************
//   	cv::imshow("houghLines" ,vis_houghLines);
// 	cv::waitKey(1);
     

}

float FindLines::getMeasurementConvexhullArea(std::vector< Line >& Lines){
    vector<cv::Point> LineEndPoints;
    vector<cv::Point> ConvexHullPoints;
    if(Lines.size() >0){
	for(unsigned int i=0; i < Lines.size(); ++i ){
	    LineEndPoints.push_back(Lines[i].s );
	    LineEndPoints.push_back(Lines[i].e );
	}
	
	cv::convexHull(LineEndPoints,ConvexHullPoints,false);
	float area =contourArea(ConvexHullPoints);
      
	return area;
	  
    }
    else{
        return 0;}
  
}

float FindLines::getMeasurementConfidence(std::vector< Line >& Lines){
   
    if(Lines.size()>0 ){

	float areaThreshold = params.line.MeasAreaThreshod->get();
	float lineNumThreshold = params.line.MeasLineNumThreshod->get();
	float lineLenThreshold = params.line.MeasLineLengthThreshod->get();
	float angThreshold = params.line.MeasLineMaxAngDifThreshod->get();
    
    

	float area = getMeasurementConvexhullArea( Lines);
	float lineNum = Lines.size();
	float sumOfLineLen =0;
	float maxAngDif = 0;
	
	for(unsigned int i=0; i < Lines.size(); ++i ){
	    sumOfLineLen+= Lines[i].len;
	}
	

	for(unsigned int i=0; i < Lines.size(); ++i ){
	    for(unsigned int j=i; j < Lines.size(); ++j ){
	      float angDif = m_Math::AngDif(Lines[i].ang, Lines[j].ang ) ;
	      if(angDif > maxAngDif){ maxAngDif =angDif; }
	      if( maxAngDif > angThreshold){break;}
	  
	   }
	   if( maxAngDif > angThreshold){break;}
	}
	
	
	area = std::min(area, areaThreshold);
	lineNum = std::min(lineNum, lineNumThreshold);
	sumOfLineLen = std::min(sumOfLineLen, lineLenThreshold);
	maxAngDif = std::min(maxAngDif, angThreshold);
// 	cout<< " " << area/areaThreshold<<" " <<lineNum/lineNumThreshold<<" " <<  sumOfLineLen/lineLenThreshold <<" " << maxAngDif/angThreshold<<endl;
	
	return ( area/areaThreshold + lineNum/lineNumThreshold + sumOfLineLen/lineLenThreshold + maxAngDif/angThreshold   )* 0.25;
    
    }
    else{
      return 0;
    }
}


