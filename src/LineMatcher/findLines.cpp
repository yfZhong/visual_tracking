/*

Authors: Yongfeng

*/

#include <visual_tracking/LineMatcher/findLines.h>

FindLines::FindLines()/*:filtered_data(H * W,0)*/{
    debug_line_detector = 0;
    m_Top = H-1;
    fpsData = 30;

   
    MeasConf = 0;
    count=0;
    num_change_min_ske = 100;
    delta1 =0;

    mean_b = 70;
    min_skele_shift=0;

    
}


FindLines::~FindLines(){     

  
}


void FindLines::findBoundingRects(FrameGrabber & CamFrm){
  
  
  m_Top= CamFrm.m_Top;
  
  applyLineFilter(/* in */CamFrm.Brightness_Channel,CamFrm.fieldConvexHullMat,CamFrm.GreenBinary, /* out */CamFrm.weightedWhiteValues);
  
  RetrieveSkeleton(/* in */CamFrm.fieldConvexHullMat, /* in */CamFrm.weightedWhiteValues, /* out */ detectedPoins );
  
//   regionGlow( /* in */CamFrm.weightedWhiteValues , skeletonMatrix_tmp, /* in */CamFrm.fieldConvexHullMat, detectedPoins );
  
//   removeObstacleBoarder (/* in */ CamFrm.ObstacleConvexHull  ,/* in_out */ detectedPoinsWithType);
  
  
  findSquares(CamFrm);
  

  findRectangle_Buffer();
  
}





void FindLines::applyLineFilter(/*in*/cv:: Mat & Brightness_Channel,/* in */cv:: Mat & fieldConvexHull, /*in*/cv:: Mat & greenBinary, /*out*/float ( & weightedWhiteValues )[ H ][ W ]){
  
     Scalar m = mean(Brightness_Channel(cv::Rect(150,250,640-300,480-250)));
//      cout<<  m.val[0]/2<<endl;
     mean_b= m.val[0];
     int shift=0;
//      if(mean_b>80){ shift+= mean_b-80;  }
     
     
    int min_brightness = params.line.MinBrightnessValue->get() + shift;  
    float div_pct1 = params.line.Div1->get();
    float div_pct2 = params.line.Div2->get();//from  top to bottom [0-div_pct1, div_pct1-div_pct2, div_pct2-1]
    
    float div_pct3 = 0.8;
   
    
    int offset_size_9 = 7; int half_size_9 = 3;
    int offset_v_9[7][2]  = {{0,-3},{0,-2},{0,-1},{0,0},{0,1},{0,2},{0,3}}; //offset for vertical and horizontal kernel
    int offset_h_9[7][2]  = {{-3,0},{-2,0},{-1,0},{0,0},{1,0},{2,0},{3,0}}; //offset for vertical and horizontal kernel
    int offset_d45_9[7][2]  = {{3,-3},{2,-2},{1,-1},{0,0},{-1,1},{-2,2},{-3,3}}; //offset for diagonal kernel
    int offset_d135_9[7][2]  = {{-3,-3},{-2,-2},{-1,-1},{0,0},{1,1},{2,2},{3,3}}; //offset for diagonal kernel
    
    int DOG_kernel_9[7] = {-1,-3,2,4,2,-3,-1}; //difference of gaussian kernel
 
    memset(weightedWhiteValues, 0, sizeof(float)*W*H); 
    
    
    
    double max_v =std::numeric_limits<double>::min();
    
    
    
     int kernel_type = 3; // 3 times larger than the first kernel 
     
     
	 //apply the dfg kernel in one direction and in its vertical direction apply the gassian smoth kernel(1,2,1)
	 ros::Time t = ros::Time::now();
	 
         for ( int i = 0; i< W; i++){  
	    for ( int j = m_Top ; j< H-20; j++){ 
	      
//            for t9 bagfile, skip the most bottom part because the white body parts may interrupt the line detection
// 	      if(j> H-100){continue;}
	      
	      
	      
// 	    for ( int j = 0; j< H; j++){   
		if(j<div_pct1 * H ){
		    kernel_type = 1; // small kernel for upper part image
		}
		else if(j<div_pct2 * H){
		    kernel_type = 2; // middle size kernel for middle part of the image
		}
		else if(j<div_pct3 * H){
		    kernel_type = 3; // middle size kernel for middle part of the image
		}
		else{
		    kernel_type = 4; // largest kernel for lower part of the image
		}


	        if(i<half_size_9*kernel_type+1 || i>= W -half_size_9*kernel_type-1 || j<( m_Top   + half_size_9*kernel_type+1) || j>= H -half_size_9*kernel_type-1
		    || Brightness_Channel.at<uchar>( j , i) < min_brightness /*|| greenV[0] == 255*/ ){		 
		     continue;}
		
		int too_close_to_boundary =0;
		while(true){     
		         
		    if(fieldConvexHull.at<uchar>(j+half_size_9*kernel_type, i ) ==0  || fieldConvexHull.at<uchar>(j-half_size_9*kernel_type, i ) ==0 ||  
		       fieldConvexHull.at<uchar>(j, i +half_size_9*kernel_type) ==0  || fieldConvexHull.at<uchar>(j, i-half_size_9*kernel_type ) ==0
		    ){ 
			if(kernel_type <1){too_close_to_boundary =1; break;}
			else{kernel_type--;}
		    }
		    else{break;}
		}
		
		if(too_close_to_boundary  ){continue;}
		
	
		int tmp_v = 0, tmp_h = 0, tmp_d45 =0 ,tmp_d135=0;
		
		for(int os =0; os < offset_size_9; os++){ //os: offset size
		      int pos_x,pos_y;
		  
		      pos_x= i;
		      pos_y= j + offset_v_9[os][1] *kernel_type;
		      tmp_h +=   DOG_kernel_9[os]* ((Brightness_Channel.at<uchar>( pos_y , pos_x)<<1) +Brightness_Channel.at<uchar>( pos_y , pos_x-1) +Brightness_Channel.at<uchar>( pos_y , pos_x +1) );
		      
		      
		      pos_x= i+ offset_h_9[os][0]*kernel_type;
		      pos_y= j;
		      tmp_v +=  DOG_kernel_9[os]* ((Brightness_Channel.at<uchar>( pos_y , pos_x)<<1) + Brightness_Channel.at<uchar>( pos_y+1 , pos_x) + Brightness_Channel.at<uchar>( pos_y -1, pos_x));
		      
		      pos_x= i+ offset_d135_9[os][0]*kernel_type;
		      pos_y= j+ offset_d135_9[os][1]*kernel_type;

		      tmp_d45 +=  DOG_kernel_9[os]* ((Brightness_Channel.at<uchar>( pos_y , pos_x)<<1) + Brightness_Channel.at<uchar>( pos_y+1 , pos_x-1) +Brightness_Channel.at<uchar>( pos_y-1 , pos_x+1));
		      
		      
		      pos_x= i+ offset_d45_9[os][0]*kernel_type;
		      pos_y= j+ offset_d45_9[os][1]*kernel_type;

		      tmp_d135 +=  DOG_kernel_9[os]* ((Brightness_Channel.at<uchar>( pos_y , pos_x)<<1) + Brightness_Channel.at<uchar>( pos_y-1 , pos_x-1) +Brightness_Channel.at<uchar>( pos_y +1, pos_x+1));  
		}

		tmp_h = std::max(0,tmp_h );
		tmp_v = std::max(0,tmp_v );
		tmp_d45 = std::max(0,tmp_d45 );
		tmp_d135 = std::max(0,tmp_d135 );

		weightedWhiteValues[(j)][  i] = sqrt( pow(tmp_h,2)+ pow(tmp_v,2)+ pow(tmp_d45,2)+ pow(tmp_d135,2));

	    
	    
		if(max_v <weightedWhiteValues[j][i]){max_v = weightedWhiteValues[(j)][ i];}
	    }
	   
	   
	}

        if(max_v>1500){ max_v=1500;  }
        

	 for ( int j =  m_Top ; j< H; j++){
	     for ( int i = 0; i< W; i++){
	             if(greenBinary.data[j * W + i ] == 255 ){ weightedWhiteValues[j][ i] = 0;}
	             else if( weightedWhiteValues[j][ i]>max_v ){  weightedWhiteValues[j][ i] =255; }
	             else{
		     weightedWhiteValues[j][ i] = (weightedWhiteValues[j][ i]-0.0)/(max_v-0.0) *255;}
	     }
	   
	}
	

}


void FindLines::RetrieveSkeleton (/* in */cv:: Mat & fieldConvexHull, /* in */ const float ( & matrix )[ H ][ W ] , /* out */  std::vector<cv::Point> &detectedPoins ){

// 	    skeletonMatrix_tmp = Mat::zeros(fieldConvexHull.size(), CV_8UC1);
          
            skeletonMatrix_tmp = Mat::zeros(cv::Size(W, H - m_Top ), CV_8UC1);
	    
	    int MinSkeletonValue = params.line.MinSkeletonValue->get() ;
// 	    int MinSkeletonValue = params.line.MinSkeletonValue->get() + min_skele_shift;
// 	    
// 
// 	    
// 	    if( heading_speed>=0.08){num_change_min_ske=0; delta1 = 14; }
// 	    else if( heading_speed>=0.05 && !( delta1 = 10&& num_change_min_ske <6)){num_change_min_ske=2; delta1 = 9; }
// 	    else if( heading_speed>=0.03 && !( delta1 > 6 && num_change_min_ske <6)){num_change_min_ske=3; delta1 = 5;}
// 	    
// 	   
// 	   if(abs(mean_b - 48) < 3 ){ MinSkeletonValue +=14; }
// 	   else{ if( num_change_min_ske <6){MinSkeletonValue += delta1;  num_change_min_ske++; }}
	    
	    
//            cout<<"MinSkeletonValue  "<<MinSkeletonValue<<endl;
	    
	    detectedPoins.clear();
	    detectedPoinsWithType.clear();
            // * Loop through rows. * //
            for ( int x = 1; x < W - 1; x++ ) { // [0]
                // * Loop through columns. * // for ( int x = 1; x < W - 1; x++ ) { // [0]
                // * Loop through columns. * //
                // * Process the pixels under m_FieldBoundary[x]-params.field.FieldBoarder->get()/2 for skeleton. * //
                for ( int y = m_Top; y < H; y++ ) {// [1]
// 		    if ( fieldConvexHull.at<uchar>(y,x ) == 0 ) { // [2]  //at<uchar>(i,j) i: rows, j: cols
//                        continue;
//                     }
                    register unsigned int vote = 0;
                    register int val = matrix[ y ][ x ];
		    if ( val < MinSkeletonValue ) { // [2]
                       continue;
                    }
                    else { // [2]
                        if ( val <= matrix[ y ][ x - 1 ] ) { // [3]
                            vote++;
                        }
                        if ( val <= matrix[ y ][ x + 1 ] ) { // [3]
                            vote++;
                        }
                        if ( val <= matrix[ (y - 1) ][ x ] ) { // [3]
                            vote++;
                        }
                        if ( val <= matrix[ (y + 1) ][ x ] ) { // [3]
                            vote++;
                        }
                        if ( vote <= 3 ) { // [3]
                            if ( val <= matrix[ (y - 1) ][ x - 1 ] ) { // [4]
                                vote++;
                            }
                            if ( val <= matrix[ (y + 1)][ x - 1 ] ) { // [4]
                                vote++;
                            }
                            if ( vote < 3 ) { // [4]
                                if ( val <= matrix[ (y - 1) ][ x + 1 ] ) { // [5]
                                    vote++;
                                }
                                if ( val <= matrix[ (y + 1) ][ x + 1 ] ) { // [5]
                                    vote++;
                                }
                                if ( vote < 3 ) { // [5]
				     detectedPoins.push_back(cv::Point(x , y)); 
				     detectedPoinsWithType.push_back(make_pair(cv::Point(x , y), vote)); 
				     skeletonMatrix_tmp.at<uchar>(y- m_Top,x) = 255;
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


void FindLines::regionGlow( const float ( & matrix )[ H ][ W ] , Mat & visited, cv:: Mat & _fieldConvexHull, std::vector<pair<cv::Point, int> > & _detectedPoinsWithType ){
  
  int size = _detectedPoinsWithType.size();
  int LocalOptimalRange = params.line.LocalOptimalRange->get();
  int MinSkeletonValue =params.line.MinSkeletonValue->get();
  
  
  
  for( int i =0; i< size; ++i){
//     int type = _detectedPoinsWithType[i].second;
     
        cv::Point seed = _detectedPoinsWithType[i].first;
        push_back_neighbors(seed.x, seed.y);
	while(!neighbors.empty()){
		Point pixel = neighbors.front();
		neighbors.pop();

		//first check if there already is a maximum in neighborhood
		if(visited.at<uchar>(pixel.y, pixel.x) >0){
                 continue;
		}
		//else check neighborhood of pixel if equal to (i,j)
		else if (  _fieldConvexHull.at<uchar>(pixel.y ,pixel.x) >0
		       && matrix[ pixel.y ][ pixel.x ] >= matrix[ seed.y ][ seed.x ] -LocalOptimalRange
		       && matrix[ pixel.y ][ pixel.x ] > MinSkeletonValue ){
                          visited.at<uchar>(pixel.y, pixel.x)= 255;
			  _detectedPoinsWithType.push_back(make_pair(cv::Point( pixel.x, pixel.y ) , 2) );
			  push_back_neighbors(pixel.x ,pixel.y);
		  
		}
    }
    
  }

}


void FindLines::regionGlow(const float ( & matrix )[ H ][ W ] , Mat & visited , cv:: Mat & _fieldConvexHull, std::vector<cv::Point> & detectedPoins){
  
  int size = detectedPoins.size();
  int LocalOptimalRange = params.line.LocalOptimalRange->get();
  int MinSkeletonValue =params.line.MinSkeletonValue->get();
  
  
  
  for( int i =0; i< size; ++i){
//     int type = _detectedPoinsWithType[i].second;
     
        cv::Point seed = detectedPoins[i];
        push_back_neighbors(seed.x, seed.y);
	while(!neighbors.empty()){
		Point pixel = neighbors.front();
		neighbors.pop();

		//first check if there already is a maximum in neighborhood
		if(visited.at<uchar>(pixel.y- m_Top, pixel.x) >0){
                 continue;
		}
		//else check neighborhood of pixel if equal to (i,j)
		else if (  _fieldConvexHull.at<uchar>(pixel.y ,pixel.x) >0
		       && matrix[ pixel.y ][ pixel.x ] >= matrix[ seed.y ][ seed.x ] -LocalOptimalRange
		       && matrix[ pixel.y ][ pixel.x ] > MinSkeletonValue ){
                          visited.at<uchar>(pixel.y- m_Top, pixel.x)= 255;
			  detectedPoins.push_back(cv::Point( pixel.x, pixel.y )  );
			  push_back_neighbors(pixel.x ,pixel.y);
		  
		}
    }
    
  }

}


void FindLines::findSquares(FrameGrabber & CamFrm){
     squars.clear();
     vector< pair<cv::Point, int> > squars_for_check;//0-H_tmp
     
//      std::sort(detectedPoins.begin(), detectedPoins.end(),  boost::bind(&sqrt(cv::Point::x * cv::Point::x+ cv::Point::y* cv::Point.y) , _1) > boost::bind(&sqrt(cv::Point::x * cv::Point::x+ cv::Point::y* cv::Point.y) , _2));
      std::sort(detectedPoins.begin(), detectedPoins.end(), bind(&FindLines::SortFuncDescending, this, _1, _2));

    int H_tmp = H - m_Top;
	  
    int k_max = params.square.Max_K->get();
    int k_max_border = params.square.Max_K_border->get();
    int MaxNeighborDist = params.square.MaxNeighborDist->get();
    
    Mat skeletonCount[k_max];
    for(int k=0; k<k_max;++k){ skeletonCount[k] = Mat::zeros(cv::Size(W,H_tmp ), CV_8UC1);} 
//      skeletonCount[0] = keletonMatrix_tmp;
   
     for ( int x = 0; x < W ; x++ ) { // [0]
                // * Loop through columns. * //
                for ( int y = 0; y < H_tmp ; y++ ) {// [1]
		  int count = 0;
			      if( skeletonMatrix_tmp.at<uchar>(y , x ) == 255){count++;}
		  skeletonCount[0].at<uchar>(y ,x) =count;
		}	
     }
     
      for(int k=1; k<k_max;++k){
	
	     for ( int x = 0; x < W  -(2*k); x++ ) { // [0]
                // * Loop through columns. * //
                // * Process the pixels under m_FieldBoundary[x]-params.field.FieldBoarder->get()/2 for skeleton. * //
                for ( int y = 0; y < H_tmp -(2*k); y++ ) {// [1]
		  
// 		   if( skeletonMatrix_tmp.at<uchar>(y , x ) == 0){continue;}
		  int count = 0;
		  for(int i= 2*(k-1) +1; i<=2* k ;++i){
		    	  for(int j= 0; j<= 2*(k-1);++j){
			      if( skeletonMatrix_tmp.at<uchar>(y + j , x + i) == 255){count++;}
		            }
		  }
		  
		  for(int i=0; i<=2*k; ++i){
		    	  for(int j= 2 *(k-1)+1; j<= 2*k;++j){
			      if( skeletonMatrix_tmp.at<uchar>(y + j , x + i) == 255){count++;}
		            }
		  }
		  
		  skeletonCount[k].at<uchar>(y  ,x) = count + skeletonCount[k-1].at<uchar>(y  ,x);
		}	
              }
      }
  

        int32_t dim = 2;
	int32_t M_num = detectedPoins.size();
      
          // kd tree of model points
	kdtree::KDTree*     M_tree;
	kdtree::KDTreeArray M_data;
	M_data.resize(boost::extents[M_num][dim]);
      
	for(int i = 0;i< M_num; i++){
	      M_data[i][0] = (float)detectedPoins[i].x;;
	      M_data[i][1] = (float)(detectedPoins[i].y- m_Top);
	}
	// build a kd tree from the model point cloud
	M_tree = new kdtree::KDTree(M_data);
	

	
	Mat visited_mat = Mat::zeros(cv::Size(W,H_tmp ), CV_8UC1);
	
	int offset_center[4][2]  = {{-1,-1},{1,-1},{-1,1},{1,1}};
	int offset_corner[4][4][2]  = {{{0,0},{2,0},{0,2},{2,2}}, {{-2,0},{0,0},{-2,2},{0,2}},{{0,-2},{2,-2},{0,0},{2,0}},{{-2,-2},{0,-2},{-2,0},{0,0}}};
	

        int k_max_tmp = k_max;
	
	for ( unsigned int i = 0; i < detectedPoins.size() ; i++ ) {//[1] 
	  
	  int x = detectedPoins[i].x;
	  int y = detectedPoins[i].y- m_Top;
	  squars_for_check.clear();
	  
	      
	      if(visited_mat.at<uchar>(y , x ) >0 ){continue;}// [2]
		
		     int corner_idx =-1, cx, cy;// cx--- Point_corner_x, ccx----Point center
		     int squareL_k=0;
		    
		     
		     //(1)find a corrner skeleton point
// 		     if(y< (0.3*H-m_Top) || (x<0.1*W && y< (0.8*H-m_Top)) || (x> 0.9*W && y< (0.8*H-m_Top))){k_max_tmp = k_max_border;
		     if(y< (0.3*H-m_Top) || (x<0.1*W && y< (0.6*H-m_Top)) || (x> 0.9*W && y< (0.6*H-m_Top))){k_max_tmp = k_max_border;
		       MaxNeighborDist = params.square.MaxNeighborDist->get();
		     }
		     else{k_max_tmp = k_max;
		        MaxNeighborDist = params.square.MaxNeighborDist->get() *2;}
		     
		     for(int k = 1; k<k_max_tmp;++k){ // [3.1]
		         
			    squareL_k = k;
			    if(x-k<0|| x+k>=W||y-k<0|| y+k>=H_tmp){break;}
			    
			    int meetOtherSquare=0;
			    
			    //check whether will overlap with previous squares
			    for(int x_i = x-k;x_i<=x+k; ++x_i){ 
			      if(visited_mat.at<uchar>(y-k, x_i ) >0  ){
				  squareL_k--;
				  meetOtherSquare=1; 
				  if(x_i<=x){corner_idx =0;}else{corner_idx =1;}
			      }
			       else if( visited_mat.at<uchar>(y+k, x_i ) >0 ){
				  squareL_k--;
				  meetOtherSquare=1; 
				  if(x_i<=x){corner_idx =2;}else{corner_idx =3;}
			      }
				  
			       if(meetOtherSquare ==1 ){break; }
			    }
				  
			    if(meetOtherSquare ==1 ){ 
			      cx = x + offset_center[corner_idx ][0] * squareL_k;
			      cy = y + offset_center[corner_idx ][1] * squareL_k; 
			      break;}
			      
			    for(int y_i = y-k +1;y_i<= y+k-1; ++y_i){ 
			      if(visited_mat.at<uchar>(y_i, x-k ) >0 ){
				  squareL_k--; 
				  meetOtherSquare=1;
				  if(y_i<=y){corner_idx =0;}else{corner_idx =2;}
			      }
			      else if(visited_mat.at<uchar>(y_i, x+k ) >0 ){
				  squareL_k--; 
				  meetOtherSquare=1;
				  if(y_i<=y){corner_idx =1;}else{corner_idx =3;}
			      }
			      if(meetOtherSquare ==1 ){break; }
			    }
			    if(meetOtherSquare ==1 ){  
			          cx = x + offset_center[corner_idx ][0] * squareL_k;
			          cy = y + offset_center[corner_idx ][1] * squareL_k;  break;}
			    
			    
			    
			    std::vector<float>         query(dim);
			    for(int c_i=0; c_i<4; c_i++){
				query[0]= x+ offset_center[c_i][0]*squareL_k; 
				query[1]= y+ offset_center[c_i][1]*squareL_k;

				kdtree::KDTreeResultVector neighbor;
				M_tree->n_nearest(query,1,neighbor);
				// check if it is an inlier
				  if (neighbor[0].dis>MaxNeighborDist){
				    corner_idx = c_i;  
				    squareL_k--; 
    // 				cx = M_tree->the_data[neighbor[0].idx][0];
    // 				cy = M_tree->the_data[neighbor[0].idx][1];
				    cx= x+ offset_center[c_i][0]*squareL_k; 
				    cy= y+ offset_center[c_i][1]*squareL_k;
				  }
				  if(corner_idx >=0 ){ break;}
			    }
			    if(corner_idx >=0 ){ break;}
		     }// [3.1]
		     
		     
		     //(2)find a optimal k starting from this corner pixel
		     int k_tmp= squareL_k;
// 		     int k_ref = -1; //for checking the square size. if it is too small then throw away
// 		     int neighbor_idx=-1;
		     //start from this corner and eapand the square
		      if(corner_idx <0){// [3.2]
// 			squars.push_back(make_pair(cv::Point(x, y) , squareL_k));
// 			 squareL_k = 0;
			cx = x -squareL_k;
			cy = y - squareL_k;
			corner_idx =0;

		      }
		      else{
			for(int k = k_tmp; k<k_max_tmp;++k){
			    squareL_k = k;
			    
			    if(cx + offset_corner[corner_idx][0][0]*k  <0|| cx+ offset_corner[corner_idx][3][0]*k>=W
			      ||cy+ offset_corner[corner_idx][0][1]*k  <0|| cy+ offset_corner[corner_idx][3][1]*k >=H_tmp){break;}
			      
			      int meetOtherSquare=0;
			      
			      int center_x= cx + 0.5*(offset_corner[corner_idx][0][0]*squareL_k +offset_corner[corner_idx][3][0]*squareL_k);
			      int center_y= cy + 0.5*(offset_corner[corner_idx][0][1]*squareL_k +offset_corner[corner_idx][3][1]*squareL_k);
			      
// 			      for(int x_i = center_x -squareL_k ;x_i<=center_x +squareL_k; ++x_i){
// 				  for(int y_i = center_y  -squareL_k ;y_i<= center_y +squareL_k; ++y_i){
// 				      if(visited_mat.at<uchar>(y_i, x_i ) >0){meetOtherSquare = 1;}
// 				      if(meetOtherSquare ==1 ){ squareL_k--;break;}
// 				  }
// 				  if(meetOtherSquare ==1 ){ break;}
// 			      }
// 			      if(meetOtherSquare ==1 ){ break;}
			      
			      if(corner_idx==0){			      
				  for(int x_i = center_x +squareL_k-1 ;x_i<=center_x +squareL_k; ++x_i){
				      for(int y_i = center_y  -squareL_k ;y_i<= center_y +squareL_k-2; ++y_i){
					  if(visited_mat.at<uchar>(y_i, x_i ) >0){meetOtherSquare = 1;}
					  if(meetOtherSquare ==1 ){ squareL_k--;break;}
				      }
				      if(meetOtherSquare ==1 ){ break;}
				  }
				 for(int x_i = center_x - squareL_k ;x_i<=center_x +squareL_k; ++x_i){
				      for(int y_i = center_y  + squareL_k-1 ;y_i<= center_y +squareL_k; ++y_i){
					  if(visited_mat.at<uchar>(y_i, x_i ) >0){meetOtherSquare = 1;}
					  if(meetOtherSquare ==1 ){ squareL_k--;break;}
				      }
				      if(meetOtherSquare ==1 ){ break;}
				  }
				
			      }
			      else if(corner_idx==1){			      
				  for(int x_i = center_x -squareL_k;x_i<=center_x -squareL_k+1; ++x_i){
				      for(int y_i = center_y  -squareL_k ;y_i<= center_y +squareL_k-2; ++y_i){
					  if(visited_mat.at<uchar>(y_i, x_i ) >0){meetOtherSquare = 1;}
					  if(meetOtherSquare ==1 ){ squareL_k--;break;}
				      }
				      if(meetOtherSquare ==1 ){ break;}
				  }
				 for(int x_i = center_x - squareL_k ;x_i<=center_x +squareL_k; ++x_i){
				      for(int y_i = center_y  + squareL_k-1 ;y_i<= center_y +squareL_k; ++y_i){
					  if(visited_mat.at<uchar>(y_i, x_i ) >0){meetOtherSquare = 1;}
					  if(meetOtherSquare ==1 ){ squareL_k--;break;}
				      }
				      if(meetOtherSquare ==1 ){ break;}
				  }
				
			      }
			      else if(corner_idx==2){			      
				  for(int x_i = center_x +squareL_k -1;x_i<=center_x +squareL_k; ++x_i){
				      for(int y_i = center_y  -squareL_k +2 ;y_i<= center_y +squareL_k; ++y_i){
					  if(visited_mat.at<uchar>(y_i, x_i ) >0){meetOtherSquare = 1;}
					  if(meetOtherSquare ==1 ){ squareL_k--;break;}
				      }
				      if(meetOtherSquare ==1 ){ break;}
				  }
				 for(int x_i = center_x - squareL_k ;x_i<=center_x +squareL_k; ++x_i){
				      for(int y_i = center_y  - squareL_k ;y_i<= center_y -squareL_k +1; ++y_i){
					  if(visited_mat.at<uchar>(y_i, x_i ) >0){meetOtherSquare = 1;}
					  if(meetOtherSquare ==1 ){ squareL_k--;break;}
				      }
				      if(meetOtherSquare ==1 ){ break;}
				  }
				
			      }
			       else if(corner_idx==3){			      
				  for(int x_i = center_x -squareL_k;x_i<=center_x -squareL_k+1; ++x_i){
				      for(int y_i = center_y  -squareL_k+2 ;y_i<= center_y +squareL_k; ++y_i){
					  if(visited_mat.at<uchar>(y_i, x_i ) >0){meetOtherSquare = 1;}
					  if(meetOtherSquare ==1 ){ squareL_k--;break;}
				      }
				      if(meetOtherSquare ==1 ){ break;}
				  }
				 for(int x_i = center_x - squareL_k ;x_i<=center_x +squareL_k; ++x_i){
				      for(int y_i = center_y  - squareL_k ;y_i<= center_y -squareL_k +1; ++y_i){
					  if(visited_mat.at<uchar>(y_i, x_i ) >0){meetOtherSquare = 1;}
					  if(meetOtherSquare ==1 ){ squareL_k--;break;}
				      }
				      if(meetOtherSquare ==1 ){ break;}
				  }
				
			      }
			          if(meetOtherSquare ==1 ){ break;}
			      
			      
			      
			      
			      
			
			      std::vector<float>   query(dim);
			      int corner_num=0;  
			      for(int c_i=0; c_i<4; c_i++){
  // 			        if(c_i == corner_idx){continue;}
				  query[0]= cx + offset_corner[corner_idx][c_i][0]*k; 
				  query[1]= cy + offset_corner[corner_idx][c_i][1]*k;
				  
				  kdtree::KDTreeResultVector neighbor;
				  M_tree->n_nearest(query,1,neighbor);
				  // check if it is an inlier
	  
				  if (neighbor[0].dis>MaxNeighborDist){
					corner_num++;
				  }
				  if(corner_num >=3 ){squareL_k--; break;}
			      }
			      if(corner_num >=3 ){ break;}

			}
		     }// [3.2]
		     
// 		     if(squareL_k< 0.5*k_ref+1 ){
		     if(squareL_k< 0 ){  //// [3.3]
		       
// 		       	  for(int si = -squareL_k; si<=squareL_k; ++si){
// 				for(int sj = -squareL_k; sj<=squareL_k; ++sj){
// 				    visited_mat.at<uchar>(y +sj ,x + si ) = neighbor_idx+1;
// 				}
// 			  }
		       
		    }
		     else{
		     
			  //(3)find a optimal center
			    int value_o =-1;
			    int o_x, o_y;
			    
			    int center_x= cx + 0.5*(offset_corner[corner_idx][0][0]*squareL_k +offset_corner[corner_idx][3][0]*squareL_k);
			    int center_y= cy + 0.5*(offset_corner[corner_idx][0][1]*squareL_k +offset_corner[corner_idx][3][1]*squareL_k);
			    
			    for( int xi = -squareL_k; xi<=squareL_k; ++xi ){
				for( int yj = -squareL_k; yj<=squareL_k; ++yj ){
				     			      
				      if(( center_x+ xi-squareL_k) <0 || (center_x + xi+squareL_k)>=W ||(center_y + yj-squareL_k) <0 || (center_y + yj+squareL_k) >=H_tmp
					|| (x<(center_x+ xi-squareL_k)) ||  (x > ( center_x+ xi+squareL_k)) || (y<( center_y+ yj-squareL_k)) || (y> (center_y + yj+squareL_k)) 
				      ){continue;}
				      
				      
				      int meetOtherSquare=0;
				      for(int x_i = center_x +xi-squareL_k ;x_i<=center_x +xi +squareL_k; ++x_i){
					  for(int y_j = center_y + yj -squareL_k ;y_j<= center_y+ yj +squareL_k; ++y_j){
					      if(visited_mat.at<uchar>(y_j, x_i ) >0){meetOtherSquare = 1;}
                                              if(meetOtherSquare ==1 ){ break;}
					  }
                                          if(meetOtherSquare ==1 ){ break;}
				      }
				      if(meetOtherSquare ==1 ){ continue;}
				      
				      //up left corner
				      int c_x= center_x + xi + offset_center[0][0]*squareL_k; 
				      int c_y= center_y + yj + offset_center[0][1]*squareL_k;
					     
				      if( value_o < skeletonCount[squareL_k].at<uchar>(c_y, c_x )){ 
					  value_o = skeletonCount[squareL_k].at<uchar>(c_y, c_x ) ; 
					  o_x = center_x + xi;
					  o_y = center_y + yj;
				      }

				  }
			    }
			  
			  //store this optimal square

			  squars.push_back(make_pair(cv::Point(o_x, o_y+ m_Top) , squareL_k));
			  squars_for_check.push_back(make_pair(cv::Point(o_x, o_y) , squareL_k));
			  
// 			  cv::Rect rect(o_x-squareL_k,o_y - squareL_k, 2*squareL_k+1 ,2*squareL_k+1  );
// 			  cv::Mat croppedImage = skeletonMatrix_tmp(rect);
			 
			 
			  for(int si = -squareL_k; si<=squareL_k; ++si){
				for(int sj = -squareL_k; sj<=squareL_k; ++sj){
// 				  if((o_x -si )>0&&(o_x +si )<W&&(o_y -sj)>0&& (o_y +sj)<H_tmp){
				      visited_mat.at<uchar>(o_y +sj , o_x +si ) =  squars.size();  
// 				  }
				}
			  }

			  
			  
			  
		     }// [3.3]
		     
		     
		     
		     
		     int iter_num =0;
		     //check neighbors, find best neighbour
		     while(squars_for_check.size()>0 &&iter_num <1000000){ //// [3.4]
			  iter_num ++;
		          cv::Point maxP;int value =-1;
			  
			  int s_x = squars_for_check.back().first.x;
			  int s_y = squars_for_check.back().first.y;
			  int s_K = squars_for_check.back().second;
			  squars_for_check.pop_back();
			  int fix_corner_x, fix_corner_y,fix_corner_idx;
                          

			   if(y< 0.3*(H-m_Top) || x<0.1*W || x> 0.9*W){k_max_tmp = k_max_border; }
			   else {k_max_tmp = k_max;  }
			  
			  for(int _x = s_x-3*s_K-1;  _x <=s_x+s_K; _x++){
			      int _y = s_y-3*s_K-1;
			      if(_x>=0&&_x<W &&_y>=0 && skeletonCount[s_K].at<uchar>(_y, _x ) > value){
				  maxP = cv::Point( _x, _y );
				  value = skeletonCount[s_K].at<uchar>(_y, _x );
				  if( (_y+2*s_K+1)<H_tmp &&visited_mat.at<uchar>(_y+2*s_K+1 ,_x ) >0 ){fix_corner_x=_x; fix_corner_y=_y+2*s_K;  fix_corner_idx=2; }
				  else{  fix_corner_x=_x+2*s_K; fix_corner_y=_y+2*s_K;  fix_corner_idx=3;  }
			      }   
			  }
			  
			  for(int _x = s_x-3*s_K-1;  _x <=s_x+s_K; _x++){
			      int _y = s_y + s_K +1;
			      if(_x>=0 &&_x<W &&_y<H_tmp && skeletonCount[s_K].at<uchar>(_y, _x ) > value ){
				
				  maxP = cv::Point( _x, _y);
				  value = skeletonCount[s_K].at<uchar>(_y, _x );
				  
				  if( (_y-1)>0&&visited_mat.at<uchar>(_y-1 ,_x ) >0 ){fix_corner_x=_x; fix_corner_y=_y;  fix_corner_idx=0; }
				  else{  fix_corner_x=_x+2*s_K; fix_corner_y=_y;  fix_corner_idx=1;  }
			      }   
			  }
			  
			  for(int _y = s_y-3*s_K;  _y <=s_y+s_K; _y++){
			      int _x = s_x-3*s_K-1;
			      if(_x>=0&&_y<H_tmp&&_y>=0 && skeletonCount[s_K].at<uchar>(_y, _x ) > value ){

				  maxP = cv::Point( _x, _y);
				  value = skeletonCount[s_K].at<uchar>(_y, _x );
				  if( _x+2*s_K+1<W&& visited_mat.at<uchar>(_y ,_x+2*s_K+1 ) >0 ){fix_corner_x=_x+2*s_K; fix_corner_y=_y;  fix_corner_idx=1; }
				  else{  fix_corner_x=_x+2*s_K; fix_corner_y=_y+2*s_K;  fix_corner_idx=3;  }
			      }   
			  }
		       			  
			  for(int _y = s_y-3*s_K;  _y <s_y+s_K; _y++){
			      int _x = s_x+s_K;
			      if(_x>=0&&_y<H_tmp&&_y>=0 && skeletonCount[s_K].at<uchar>(_y, _x ) > value ){

				  maxP = cv::Point( _x, _y);
				  value = skeletonCount[s_K].at<uchar>(_y, _x );
				  if((_x-1)>0 && visited_mat.at<uchar>(_y ,_x-1 ) >0 ){fix_corner_x=_x; fix_corner_y=_y;  fix_corner_idx=0; }
				  else{  fix_corner_x=_x; fix_corner_y=_y+2*s_K;  fix_corner_idx=2;  }
			      }   
			  }
			  
			  if(value >0){//find a best neighbor
			    
				std::vector<float>   query(dim);
	
// 				int closest_corner_idx =-1, cx, cy; double dist = 1000;
// 			 
// 				    //find a fix corner
// 				    for(int c_i=0; c_i<4; c_i++){
// 					query[0]= maxP.x  + offset_corner[0][c_i][0]*s_K; 
// 					query[1]= maxP.y  + offset_corner[0][c_i][1]*s_K;
// 
// 					kdtree::KDTreeResultVector neighbor;
// 					M_tree->n_nearest(query,1,neighbor);
// 					// check if it is an inlier
// 				      
// 					if(dist > neighbor[0].dis){
// 					      dist= neighbor[0].dis;
// 					      closest_corner_idx = c_i; 
//     // 					  cx = M_tree->the_data[neighbor[0].idx][0];
//     // 				          cy = M_tree->the_data[neighbor[0].idx][1];
// 					      cx = query[0];
// 					      cy = query[1];
// 					}
// 				    }
				      
				    
				    int meetOtherSquare=0;
				    int _x =fix_corner_x + offset_corner[fix_corner_idx][0][0]*s_K;
				    int _y =fix_corner_y + offset_corner[fix_corner_idx][0][1]*s_K;
					  for(int x_i = _x ;x_i<=_x + 2*s_K; ++x_i){
					      for(int y_i = _y ;y_i<= _y + 2*s_K; ++y_i){
						  if(visited_mat.at<uchar>(y_i, x_i ) >0){meetOtherSquare = 1;}
						  if(meetOtherSquare ==1 ){ break;}
					      }
					      if(meetOtherSquare ==1 ){ break;}
					  }
				
				
				  if(meetOtherSquare==0){
					  //count the corner number  
					  int corner_num=1;
					  for(int c_i=0; c_i<4; c_i++){
					    
						if(c_i ==fix_corner_idx){continue;}
						query[0]= fix_corner_x + offset_corner[fix_corner_idx][c_i][0]*s_K; 
						query[1]= fix_corner_y + offset_corner[fix_corner_idx][c_i][1]*s_K;
						
// 						if(visited_mat.at<uchar>(query[1] ,query[0] )>0){
// 						  corner_num =4;break;
// 						}
						kdtree::KDTreeResultVector neighbor;
						M_tree->n_nearest(query,1,neighbor);
						// check if it is an inlier
					      
						if (neighbor[0].dis>MaxNeighborDist ){
						    corner_num++;
						}

					  }
					    
					    
					    if(corner_num <=2){//too small k
						
						int s_K_tmp = s_K;
						for(int new_k = s_K_tmp +1;new_k<k_max_tmp; new_k++ ){
							    corner_num =1;
							    s_K = new_k;
							    
							    int meetOtherSquare3=0;
							    int ulx =fix_corner_x + offset_corner[fix_corner_idx][0][0]*s_K;
							    int uly =fix_corner_y + offset_corner[fix_corner_idx][0][1]*s_K;
								  for(int x_i = ulx ;x_i<=ulx + 2*s_K; ++x_i){
								      for(int y_i = uly ;y_i<= uly + 2*s_K; ++y_i){
									  if(visited_mat.at<uchar>(y_i, x_i ) >0){meetOtherSquare3 = 1;}
									  if(meetOtherSquare3 ==1 ){ break;}
								      }
								      if(meetOtherSquare3 ==1 ){ break;}
								  }
							    
							    if(meetOtherSquare3 ==1 ){s_K--; break;}
							    
							    for(int c_i=0; c_i<4; c_i++){
								if(c_i ==fix_corner_idx){continue;}
								query[0]= fix_corner_x + offset_corner[fix_corner_idx][c_i][0]*new_k; 
								query[1]= fix_corner_y + offset_corner[fix_corner_idx][c_i][1]*new_k;
							      
// 								if(visited_mat.at<uchar>(query[1] ,query[0] )>0){
// 								  corner_num =4;s_K--;break;
// 								}
								
								kdtree::KDTreeResultVector neighbor;
								M_tree->n_nearest(query,1,neighbor);
								// check if it is an inlier
							      
								if (neighbor[0].dis>MaxNeighborDist){
								    corner_num++;
								}
								if(corner_num>=3){s_K--;break;}
							  
							    }
							    if(corner_num>=3){break;}
						  }
					    }
					    else{//too large K
						int s_K_tmp = s_K;
						for(int new_k = s_K_tmp-1;new_k>=0; new_k-- ){
							    
							    s_K = new_k;
							    
							    
							    
/*							     int meetOtherSquare2=0;
							     int ulx =fix_corner_x + offset_corner[fix_corner_idx][0][0]*s_K;
							     int uly =fix_corner_y + offset_corner[fix_corner_idx][0][1]*s_K;
							     
							     for(int x_i = ulx ;x_i<=ulx + 2*s_K; ++x_i){
								for(int y_i = uly ;y_i<= uly + 2*s_K; ++y_i){
								    if(visited_mat.at<uchar>(y_i, x_i ) >0){meetOtherSquare2 = 1;}
								    if(meetOtherSquare2 ==1 ){ s_K--;break;}
								}
								if(meetOtherSquare2 ==1 ){ break;}
							     }
							    if(meetOtherSquare2 ==1 ){ continue;} */  
							    
							    
							    
							    corner_num =1;
							    for(int c_i=0; c_i<4; c_i++){
								if(c_i ==fix_corner_idx){continue;}
								query[0]= fix_corner_x + offset_corner[fix_corner_idx][c_i][0]*new_k; 
								query[1]= fix_corner_y + offset_corner[fix_corner_idx][c_i][1]*new_k;
							      
// 								if(visited_mat.at<uchar>(query[1] ,query[0] )>0){
// 								  corner_num =4;break;
// 								}
								
								kdtree::KDTreeResultVector neighbor;
								M_tree->n_nearest(query,1,neighbor);
								// check if it is an inlier
							      
								if (neighbor[0].dis>MaxNeighborDist){
								    corner_num++;
								}
							    }
							    if(corner_num<3){break;}
						  }
					    }
					    
	
	
					  
					  int center_x= fix_corner_x + 0.5*(offset_corner[fix_corner_idx][0][0]*s_K +offset_corner[fix_corner_idx][3][0]*s_K);
					  int center_y= fix_corner_y + 0.5*(offset_corner[fix_corner_idx][0][1]*s_K +offset_corner[fix_corner_idx][3][1]*s_K);

					    
					  squars.push_back(make_pair(cv::Point(center_x, center_y+ m_Top) , s_K));
					  squars_for_check.push_back(make_pair(cv::Point(center_x, center_y) , s_K));
					  
					  

					  for(int si = -s_K; si<=s_K; ++si){
					    for(int sj = -s_K; sj<=s_K; ++sj){
					      if((center_x -si )>0&&(center_x +si )<W&&(center_y - sj)>0&& (center_y +sj)<H_tmp){
						
						       visited_mat.at<uchar>(center_y +sj , center_x +si ) = squars.size();}
					    }
					  }
					  
	
					  
					  
					  

				    }
				
		           }
			    
		    }// [3.4]


	}//end of [1] 
	
	
	
	  if (M_tree) delete M_tree;
	
	
	  
// 	   dim = 2;
// 	   M_num = squars_rect.size();
// 	
// // 	    kd tree of model points
// 	  kdtree::KDTree*     M_tree2;
// 	  kdtree::KDTreeArray M_data2;
// 	  
// 	  double* M2 = (double*)calloc(dim*M_num,sizeof(double));
	
	  
	    
//  cv::Mat nodeGraph(cv::Size(W,H-m_Top),CV_8UC3,cv::Scalar(150, 150, 150));
// 	 pure_detectedPoins.clear();
// 	 for(unsigned int s_i=0; s_i<  squars_rect.size(); s_i++){
// // 	   	      Node node;
// // 		      node.x_pos = squars_rect[s_i].x + 0.5*squars_rect[s_i].width;
// // 		      node.y_pos = squars_rect[s_i].y + 0.5*squars_rect[s_i].height;
// // 		      node.num_connected = 0;
// // 		      node.num_close = 0;
// // 		      node.crossing_type = Node::CT_Unknown;
// // 		      m_NodeBuffer->push_back(node);
// 		      pure_detectedPoins.push_back(cv::Point( squars_rect[s_i].x + 0.5*squars_rect[s_i].width, squars_rect[s_i].y + 0.5*squars_rect[s_i].height));
// 
// 		      cv::circle(nodeGraph,cv::Point( squars_rect[s_i].x + 0.5*squars_rect[s_i].width, squars_rect[s_i].y + 0.5*squars_rect[s_i].height), 2, cv::Scalar(0, 0, 0),-1);
// // 		      M2[s_i*dim+0] = squars_rect[s_i].x + 0.5*squars_rect[s_i].width;
// // 		      M2[s_i*dim+1] = squars_rect[s_i].y + 0.5*squars_rect[s_i].height;
// 		      
// 
// 	 }
	
/*	
	M_data2.resize(boost::extents[M_num][dim]);
	for (int32_t m=0; m<M_num; m++)
	for (int32_t n=0; n<dim; n++)
	M_data2[m][n] = (float)M2[m*dim+n];

	// build a kd tree from the model point cloud
	M_tree2 = new kdtree::KDTree(M_data2);*/
	

		
// 	 for(unsigned int ni=0; ni<  m_NodeBuffer->size(); ni++){
// 	          
// 	          cv::circle(nodeGraph,cv::Point( ( * m_NodeBuffer )[ ni ].x_pos, ( * m_NodeBuffer )[ ni ].y_pos), 2, cv::Scalar(0, 0, 0),-1);
// 	   
// 
// 	           std::vector<float>   query(dim);
// 			    
// 		   query[0]=  ( * m_NodeBuffer )[ ni ].x_pos; 
// 		   query[1]=  ( * m_NodeBuffer )[ ni ].y_pos; 
// 				
//  
// 		   kdtree::KDTreeResultVector neighbor;
// 		   M_tree2->n_nearest(query,2,neighbor);
// 
// 		   
// 		   if( M_tree2->the_data[neighbor[0].idx][0] ==  query[0]&&  M_tree2->the_data[neighbor[0].idx][1] ==  query[1]  
// 		       &&neighbor[1].dis <20){
// 	  
// 			if ( !is_connected( ni, neighbor[1].idx ) ){
// 			     connect_nodes( ni, neighbor[1].idx );
// 			     cv::line( nodeGraph, cv::Point(  query[0] ,   query[1]),
// 		                         cv::Point(M_tree2->the_data[neighbor[1].idx][0],M_tree2->the_data[neighbor[1].idx][1]),  cv::Scalar(200,0,0), 1, 8 );
// 			     
// 			     
// 			}
// 		     
// 		  }
// 
// 		      
// 
// 	 }
// 	
	
// 	distortionModel.UndistortP(pure_detectedPoins, pure_undistortedDetectedPoints);
  
	

    
    
}

void FindLines::findRectangle_Buffer(){
	Rectangles.clear();
// 	pure_detectedPoins.clear();
// 	pure_undistortedDetectedPoints.clear();
	
	int MinPointNum = params.square.MinPointNum->get();
	for(unsigned int i=0; i<  squars.size(); i++){
	  

	  
		vector<cv::Point> pintsInSquare;
		int s_k = squars[i].second;
		int x = squars[i].first.x;
		int y = squars[i].first.y;
		
		
// 		    int repeat1 =0;
// 		    for(unsigned int s=0;s<  i; s++){
// 		      if( x== squars[s].first.x && y == squars[s].first.y){repeat1 =1;cout<<"s_k  "<< s_k << " 2k "<<squars[s].second<<endl; break;}
// 		    }
// 		    
// 		    if(repeat1==0){
// 		      
// 		      
// 		    }
// 		
// 		
// 		
// 		
// 		
		
		
		
		
		int avg_x=0, avg_y=0;
		
		for(int si = -s_k; si<=s_k; ++si){
		      for(int sj = -s_k; sj<=s_k; ++sj){
		 
			    if(skeletonMatrix_tmp.at<uchar>(y + sj -m_Top , x + si) == 255){
				  pintsInSquare.push_back(cv::Point(x + si ,y + sj));
			          avg_x +=x + si;
				  avg_y += y + sj;
			    }
		      }
		}
		if( pintsInSquare.size()>MinPointNum ){
			cv::Rect rect = cv::boundingRect(pintsInSquare);
// 			cv::Rect rect(x-s_k, y-s_k, s_k *2+1, s_k *2+1);
			
// 			rect.y += m_Top;
// 			
			M_rect m_rect;
			m_rect.rect = rect;
			m_rect.center_x = rect.x + 0.5* rect.width;
			m_rect.center_y = rect.y + 0.5* rect.height ;
			m_rect.area = rect.width * rect.height;
			m_rect.num_points =  pintsInSquare.size();
			m_rect.avg_x = avg_x / m_rect.num_points ;
			m_rect.avg_y = avg_y / m_rect.num_points ;
			
			m_rect.avg_x = m_rect.center_x ;
			m_rect.avg_y = m_rect.center_y ;
			
			
			cv::Point pIn(m_rect.avg_x , m_rect.avg_y);
			cv::Point pOut;
			
			if( distortionModel.UndistortP( pIn, pOut) ){
			      m_rect.undistorted_x_pos = pOut.x;
			      m_rect.undistorted_y_pos = pOut.y;
			      
			      int repeat=0;
			      for(unsigned int k=0; k<  Rectangles.size(); k++){
				if( m_rect.avg_x == Rectangles[k].avg_x && m_rect.avg_y == Rectangles[k].avg_y){repeat =1;
// 				  cout<<"repeat  "<<endl;
				  break;}
			      }
			      
			      if(repeat==0){
			          Rectangles.push_back(m_rect);}
			}
// 			pure_detectedPoins.push_back(cv::Point(m_rect.avg_x , m_rect.avg_y));
		
		}
	
	
	}
	
// 	distortionModel.UndistortP(pure_detectedPoins, pure_undistortedDetectedPoints);
	
// 	for(unsigned int i=0; i<  Rectangles.size(); i++){
// 	  
// 	  Rectangles[i].undistorted_x_pos = pure_undistortedDetectedPoints[i].x;
// 	  Rectangles[i].undistorted_y_pos = pure_undistortedDetectedPoints[i].y;
// 
// 	}
	
//      std::sort(Rectangles.begin(), Rectangles.end(),  boost::bind(&M_rect::num_points, _1) > boost::bind(&M_rect::num_points, _2));
	
 
}



// ****************************************************************************** //
void FindLines::Smooth ( /* in_out */float ( & matrix )[ H ][ W ]   ){
  
           float tmp[H][W];
	   memset(tmp, 0, sizeof(int)*W*H);  
  
 
	    // * Low-pass 1st Time. * // 
	    for ( int y = H-1; y > m_Top -1; y-- ) {
		for ( int x = 1; x < W - 1; x++ ) {
		    tmp[ y ][  x ] = ( matrix[ y ][  x ] *2 ) + ( matrix[ y ][  x - 1 ] + matrix[ y ][  x + 1 ] );
		}
	    }

	    for ( int y =  H-1  ; y > m_Top; y-- ) {
		for ( int x = 0; x < W; x++ ) {
		    matrix[ y ][  x ] = ( ( tmp[ y ][  x ] * 2 ) + ( tmp[ (y - 1) ][ x ] + tmp[ (y + 1) ][  x ] ) ) /16.0;
		}
	    }
	    
	    // * Low-pass 2nd Time. * //
	    for ( int y = H-1; y > m_Top -1; y-- ) {
		for ( int x = 1; x < W - 1; x++ ) {
		    tmp[ y ][  x ] = ( matrix[ y ][  x ] * 2 ) + ( matrix[ y ][  x - 1 ] + matrix[ y ][  x + 1 ] );
		}
	    }

	    for ( int y =  H-1  ; y > m_Top; y-- ) {
		for ( int x = 0; x < W; x++ ) {

		    matrix[ y ][  x ] = ( ( tmp[ y ][  x ] *2  ) + ( tmp[ (y - 1) ][  x ] + tmp[ (y + 1) ][  x ] ) ) / 16;
		}
	    }

	    return;

	} // END of Smooth METHOD
	
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
	
bool FindLines::SortFuncDescending(cv::Point i, cv::Point j){
	return sqrt(pow( i.x ,2)+ pow(i.y - (H -1),2)) >sqrt( pow(j.x, 2)+ pow(j.y - (H -1),2));
}

// ****************************************************************************** //



