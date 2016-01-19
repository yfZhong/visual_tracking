// #include "findLines.h"

#include <visual_tracking/LineMatcher/findLines.h>

FindLines::FindLines()/*:filtered_data(H * W,0)*/{
    debug_line_detector = 0;
    m_Top = H-1;
    fpsData = 30;
    
//     H_data = new int[W*H]; 
//     V_data = new int[W*H]; 
//     diagonal_data45 = new int[W*H]; 
//     diagonal_data135 = new int[W*H]; 
   
    MeasConf = 0;

}


FindLines::~FindLines(){     

  
}

		
void FindLines::findSkeletons(FrameGrabber & CamFrm){
  

    m_Top= CamFrm.m_Top;
   
   
//     boost::timer::cpu_timer timer;
    // apply line filters to detect lines in different orientations. Line point candidates would have higher respondses.
    applyLineFilter(/* in */CamFrm.Brightness_Channel, /* out */CamFrm.weightedWhiteValues);
     
//      fpsData = (0.9 * fpsData) + (0.1 * (1000000000l / timer.elapsed().wall));
//      cout<< fpsData<<endl;
 
//       return;
    
    RetrieveSkeleton(/* in */CamFrm.fieldConvexHullMat, /* in */CamFrm.weightedWhiteValues, /* out */ detectedPoins );
    

/*	      cv::Mat skeletonPixels1(cv::Size(W,H),CV_8UC3,cv::Scalar(150, 150, 150));

	      //draw detected points
	      for ( unsigned int i = 0; i < detectedPoinsWithType.size() ; i++ ) { 
		  cv::circle(skeletonPixels1, detectedPoinsWithType[i].first, 2, cv::Scalar(0, 10, 200),-1);
	      }
	      
		 cv::imshow("skeletonPixels1" ,skeletonPixels1);
	          cv::waitKey(1);  */   
    
    
    removeObstacleBoarder (/* in */ CamFrm.ObstacleConvexHull  ,/* in_out */ detectedPoinsWithType );
//     
//     
// 	    cv::Mat skeletonPixels2(cv::Size(W,H),CV_8UC3,cv::Scalar(150, 150, 150));

	      //draw detected points
// 	      for ( unsigned int i = 0; i < detectedPoinsWithType.size() ; i++ ) { 
// 		if(detectedPoinsWithType[i].second ==0   ){
// 		  cv::circle(skeletonPixels2, detectedPoinsWithType[i].first, 2, cv::Scalar(0, 0, 200),-1);
// 		}
// 		else if(detectedPoinsWithType[i].second ==1   ){
// 		  cv::circle(skeletonPixels2, detectedPoinsWithType[i].first, 2, cv::Scalar(200, 0, 0),-1);
// 		}
// 		else if(detectedPoinsWithType[i].second == 2  ){
// 		  cv::circle(skeletonPixels2, detectedPoinsWithType[i].first, 2, cv::Scalar(0, 200, 0),-1);
// 		}
// 	      }
//     
//      
//         regionGlow( /* in */CamFrm.weightedWhiteValues , skeletonMatrix_tmp, /* in */CamFrm.fieldConvexHullMat, detectedPoinsWithType );
//      removeObstacleBoarder (/* in */ CamFrm.ObstacleConvexHull  ,/* in_out */ detectedPoinsWithType );
    
	      
// 	erode(skeletonMatrix_tmp, skeletonMatrix_tmp , Mat(), Point(-1, -1), params.goal.erode->get());
	
	
//       RetrieveSkeleton2 ( skeletonMatrix_tmp, /* out */ detectedPoins );
//       
//       	      cv::Mat skeletonPixels4(cv::Size(W,H),CV_8UC3,cv::Scalar(150, 150, 150));
// 
// 	      //draw detected points
// 	      for ( unsigned int i = 0; i < detectedPoinsWithType.size() ; i++ ) { 
// 		  cv::circle(skeletonPixels4, detectedPoinsWithType[i].first, 2, cv::Scalar(0, 10, 200),-1);
// 	      }
      
     
//      	      for ( unsigned int i = 0; i < detectedPoinsWithType.size() ; i++ ) { 
// 		if(detectedPoinsWithType[i].second ==0   ){
// 		  cv::circle(skeletonPixels2, detectedPoinsWithType[i].first, 2, cv::Scalar(0, 0, 200),-1);
// 		}
// 		else if(detectedPoinsWithType[i].second ==1   ){
// 		  cv::circle(skeletonPixels2, detectedPoinsWithType[i].first, 2, cv::Scalar(200, 0, 0),-1);
// 		}
// 		else if(detectedPoinsWithType[i].second == 2  ){
// 		  cv::circle(skeletonPixels2, detectedPoinsWithType[i].first, 2, cv::Scalar(0, 200, 0),-1);
// 		}
// 	      }
//      
//       
// 
// 	 cv::imshow("skeletonPixels2" ,skeletonPixels2);
// 	 cv::waitKey(1);     
	      
	      
    
//       for(int j=0;j<siY;j++)
//       {
// 	memset(CamFrm.skeletonPixelMatrix[j],0, siX*sizeof(int)); 
//       }
    
      undistortedDetectedPoints.clear();
      distortionModel.UndistortP(detectedPoins, undistortedDetectedPoints);
      CamFrm.MaximunNodeNum = undistortedDetectedPoints.size();
//       for(unsigned int i =0; i< undistortedDetectedPoints.size(); ++i ){
// 	  cv::Point p = undistortedDetectedPoints[i];
// 	  CamFrm.skeletonPixelMatrix[p.y ] [ p.x ] = 3-detectedPoinsWithType[i].second;
//       }
    
       
//       cv::Mat skeletonPixels4(cv::Size(SUB_SAMPLING_WIDTH,SUB_SAMPLING_HEIGHT),CV_8UC3,cv::Scalar(150, 150, 150));
      

      for(int j=0;j<SUB_SAMPLING_HEIGHT;j++)
      {
	memset(CamFrm.skeletonPixelMatrix_sub[j],0, SUB_SAMPLING_WIDTH*sizeof(int)); 
      }
      
      
      for(unsigned int i =0; i< undistortedDetectedPoints.size(); ++i ){
	  cv::Point p = undistortedDetectedPoints[i];
	  CamFrm.skeletonPixelMatrix_sub[p.y/SUB_SAMPLING_PARAMETER ] [ p.x/SUB_SAMPLING_PARAMETER ] += (3-detectedPoinsWithType[i].second);
      }
      
      
      undistortedDetectedPoints.clear();
      for(int i =0; i< SUB_SAMPLING_HEIGHT; ++i){
	      for(int j =0; j< SUB_SAMPLING_WIDTH; ++j){

		int count =  CamFrm.skeletonPixelMatrix_sub[i][j];
		
		if(count<2){  CamFrm.skeletonPixelMatrix_sub[i][j] =0;  continue;}
		if(count >=6){ CamFrm.skeletonPixelMatrix_sub[i][j] = CamFrm.MaximunNodeNum; 
		}
		else if(count >=4){CamFrm.skeletonPixelMatrix_sub[i][j] = CamFrm.MaximunNodeNum +1; 
		  
		}
		else if(count >=2){CamFrm.skeletonPixelMatrix_sub[i][j] = CamFrm.MaximunNodeNum +2; }
		
		undistortedDetectedPoints.push_back(cv::Point(j,i));
 		
              }
      }
      
      
/*    	 cv::imshow("skeletonPixels4" ,skeletonPixels4);
	 cv::waitKey(1);   */ 
    
    
    
   
    if(debug_line_detector){
      
//      tools.cvshowImg(CamFrm.weightedWhiteValues, "weightedWhiteValues");
//      //the output file will be saved in package "tmp_file"
// 	tools.writePGM(CamFrm.weightedWhiteValues, "weightedWhiteValues");
// 	tools.writeDAT(CamFrm.weightedWhiteValues, "weightedWhiteValues.dat");
/*	
        tools.cvshowImg(CamFrm.weightedWhiteValues, "smothWhiteValues");*/
    }
    
   

}

void FindLines::findBoundingRects(FrameGrabber & CamFrm){
  
  
  m_Top= CamFrm.m_Top;
   
  applyLineFilter(/* in */CamFrm.Brightness_Channel, /* out */CamFrm.weightedWhiteValues);
    
  RetrieveSkeleton(/* in */CamFrm.fieldConvexHullMat, /* in */CamFrm.weightedWhiteValues, /* out */ detectedPoins );
    
   //detectedPoins[i].y: 0 -H
  //skeletonMatrix_tmp: W * (H -m_Top)
  
  findSquares(CamFrm);
  //squars: 0-H uars::
  findRectangle_Buffer();
  //Rectangles: 0 -H
  

//    cv::Mat rectangles(cv::Size(W*2,(H -m_Top)*2),CV_8UC3,cv::Scalar(150, 150, 150));
//    
//    for ( unsigned int i = 0; i < detectedPoins.size() ; i++ ) { 
//       cv::circle(rectangles,cv::Point( detectedPoins[i].x *2, (detectedPoins[i].y- m_Top)*2), 1, cv::Scalar(0, 10, 200),-1);
//    }
// 
//    
//   for(unsigned int i=0; i<Rectangles.size(); i++){
//     cv::line( rectangles, cv::Point((Rectangles[i].rect.x)*2,                           (Rectangles[i].rect.y -m_Top )*2), 
// 			  cv::Point((Rectangles[i].rect.x+ Rectangles[i].rect.width)*2, (Rectangles[i].rect.y  -m_Top )*2),   cv::Scalar(0,0,0), 1, 8 );
//     cv::line( rectangles, cv::Point((Rectangles[i].rect.x+ Rectangles[i].rect.width)*2, (Rectangles[i].rect.y  -m_Top )*2),   
// 			  cv::Point((Rectangles[i].rect.x+ Rectangles[i].rect.width)*2, (Rectangles[i].rect.y + Rectangles[i].rect.height  -m_Top )*2),   cv::Scalar(0,0,0), 1, 8 );
//     cv::line( rectangles, cv::Point((Rectangles[i].rect.x+ Rectangles[i].rect.width)*2, (Rectangles[i].rect.y + Rectangles[i].rect.height  -m_Top )*2),  
// 			  cv::Point((Rectangles[i].rect.x)*2,                           (Rectangles[i].rect.y + Rectangles[i].rect.height  -m_Top )*2),    cv::Scalar(0,0,0), 1, 8 );
//     cv::line( rectangles, cv::Point((Rectangles[i].rect.x)*2,                           (Rectangles[i].rect.y + Rectangles[i].rect.height  -m_Top )*2),
// 			  cv::Point((Rectangles[i].rect.x)*2,                           (Rectangles[i].rect.y -m_Top )*2),  cv::Scalar(0,0,0), 1, 8 ); 
//   }
// 
//   
//   
//     cv::imshow("rectangles" ,rectangles);
//     cv::waitKey(1);
//       	
  
  
  
}


bool FindLines::SortFuncDescending(cv::Point i, cv::Point j){
	return sqrt(pow( i.x ,2)+ pow(i.y - (H -1),2)) >sqrt( pow(j.x, 2)+ pow(j.y - (H -1),2));
}

void FindLines::findSquares(FrameGrabber & CamFrm){
     squars.clear();
     vector< pair<cv::Point, int> > squars_for_check;//0-H_tmp
     
//      std::sort(detectedPoins.begin(), detectedPoins.end(),  boost::bind(&sqrt(cv::Point::x * cv::Point::x+ cv::Point::y* cv::Point.y) , _1) > boost::bind(&sqrt(cv::Point::x * cv::Point::x+ cv::Point::y* cv::Point.y) , _2));
      std::sort(detectedPoins.begin(), detectedPoins.end(), bind(&FindLines::SortFuncDescending, this, _1, _2));

    int H_tmp = H - m_Top;
	  
    int k_max = params.square.Max_K->get();
    int MaxNeighborDist = params.square.MaxNeighborDist->get();
    
    Mat skeletonCount[k_max];
    for(int k=0; k<k_max;++k){ skeletonCount[k] = Mat::zeros(cv::Size(W,H_tmp ), CV_8UC1);} 
//      skeletonCount[0] = skeletonMatrix_tmp;
   
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
	


	
	for ( unsigned int i = 0; i < detectedPoins.size() ; i++ ) {//[1] 
	  
	  int x = detectedPoins[i].x;
	  int y = detectedPoins[i].y- m_Top;
	  squars_for_check.clear();
	      
	      if(visited_mat.at<uchar>(y , x ) >0 ){continue;}// [2]
		
		     int corner_idx =-1, cx, cy;// cx--- Point_corner_x, ccx----Point center
		     int squareL_k=0;
		    
		     
		     //(1)find a corrner skeleton point
		     for(int k = 1; k<k_max;++k){ // [3.1]
		         
			    squareL_k = k;
			    if(x-k<0|| x+k>=W||y-k<0|| y+k>=H_tmp){break;}
			    
			    int meetOtherSquare=0;
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
			for(int k = k_tmp; k<k_max;++k){
			    squareL_k = k;
			    
			    if(cx + offset_corner[corner_idx][0][0]*k  <0|| cx+ offset_corner[corner_idx][3][0]*k>=W
			      ||cy+ offset_corner[corner_idx][0][1]*k  <0|| cy+ offset_corner[corner_idx][3][1]*k >=H_tmp){break;}
			      
			      int meetOtherSquare=0;
			      
			      int center_x= cx + 0.5*(offset_corner[corner_idx][0][0]*squareL_k +offset_corner[corner_idx][3][0]*squareL_k);
			      int center_y= cy + 0.5*(offset_corner[corner_idx][0][1]*squareL_k +offset_corner[corner_idx][3][1]*squareL_k);
			      
			      for(int x_i = center_x -squareL_k ;x_i<=center_x +squareL_k; ++x_i){
				  for(int y_i = center_y  -squareL_k ;y_i<= center_y +squareL_k; ++y_i){
				      if(visited_mat.at<uchar>(y_i, x_i ) >0){meetOtherSquare = 1;}
				      if(meetOtherSquare ==1 ){ squareL_k--;break;}
				  }
				  if(meetOtherSquare ==1 ){ break;}
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
						for(int new_k = s_K_tmp +1;new_k<k_max; new_k++ ){
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
// 			cv::Rect rect = cv::boundingRect(pintsInSquare);
			cv::Rect rect(x-s_k, y-s_k, s_k *2+1, s_k *2+1);
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
			
			
			cv::Point pIn(m_rect.avg_x , m_rect.avg_y);
			cv::Point pOut;
			
			if( distortionModel.UndistortP( pIn, pOut) ){
			      m_rect.undistorted_x_pos = pOut.x;
			      m_rect.undistorted_y_pos = pOut.y;
			      Rectangles.push_back(m_rect);
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



void FindLines::applyLineFilter(/*in*/cv:: Mat Brightness_Channel,/*out*/float ( & weightedWhiteValues )[ H ][ W ]){
    float div_pct1 = params.line.Div1->get();
    float div_pct2 = params.line.Div2->get();//from  top to bottom [0-div_pct1, div_pct1-div_pct2, div_pct2-1]
    float div_pct3 = 0.9;
   
    int offset_size_9 = 9; int half_size_9 = 4;
    int offset_v_9[9][2]  = {{0,-4},{0,-3},{0,-2},{0,-1},{0,0},{0,1},{0,2},{0,3},{0,4}}; //offset for vertical and horizontal kernel
    int offset_h_9[9][2]  = {{-4,0},{-3,0},{-2,0},{-1,0},{0,0},{1,0},{2,0},{3,0},{4,0}}; //offset for vertical and horizontal kernel
    int offset_d45_9[9][2]  = {{4,-4},{3,-3},{2,-2},{1,-1},{0,0},{-1,1},{-2,2},{-3,3},{-4,4}}; //offset for diagonal kernel
    int offset_d135_9[9][2]  = {{-4,-4},{-3,-3},{-2,-2},{-1,-1},{0,0},{1,1},{2,2},{3,3},{4,4}}; //offset for diagonal kernel
    
    int DOG_kernel_9[9] = {-3,-5,1,4,6,4,1,-5,-3}; //difference of gaussian kernel
    
//     int offset_size_9 = 7; int half_size_9 = 3;
//     int offset_v_9[7][2]  = {{0,-3},{0,-2},{0,-1},{0,0},{0,1},{0,2},{0,3}}; //offset for vertical and horizontal kernel
//     int offset_h_9[7][2]  = {{-3,0},{-2,0},{-1,0},{0,0},{1,0},{2,0},{3,0}}; //offset for vertical and horizontal kernel
//     int offset_d45_9[7][2]  = {{3,-3},{2,-2},{1,-1},{0,0},{-1,1},{-2,2},{-3,3}}; //offset for diagonal kernel
//     int offset_d135_9[7][2]  = {{-3,-3},{-2,-2},{-1,-1},{0,0},{1,1},{2,2},{3,3}}; //offset for diagonal kernel
//     
//     int DOG_kernel_9[7] = {-1,-3,2,4,2,-3,-1}; //difference of gaussian kernel
    
 

//     memset(H_data, 0, sizeof(int)*W*H);  
//     memset(V_data, 0, sizeof(int)*W*H);   
//     memset(diagonal_data45, 0, sizeof(int)*W*H);   
//     memset(diagonal_data135, 0, sizeof(int)*W*H); 
    memset(weightedWhiteValues, 0, sizeof(float)*W*H); 
    
    
    
    double max_v =std::numeric_limits<double>::min();
//     min_v =std::numeric_limits<double>::max();
    
    
    
     int kernel_type = 3; // 3 times larger than the first kernel 
	 //apply the dfg kernel
         for ( int i = 0; i< W; i++){  
	    for ( int j = m_Top ; j< H; j++){   
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
		    || Brightness_Channel.at<uchar>( j , i) < params.line.MinBrightnessValue->get()){ continue;}
	
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
		
// 		    H_data[j ][ i] = std::max(0,tmp_h );
// 		    V_data[j ][ i] = std::max(0,tmp_v );
// 		    diagonal_data45[j ][ i] = std::max(0,tmp_d45 );
// 		    diagonal_data135[j ][ i] = std::max(0,tmp_d135 );
		
		weightedWhiteValues[(j)][  i] = sqrt( pow(tmp_h,2)+ pow(tmp_v,2)+ pow(tmp_d45,2)+ pow(tmp_d135,2));

	    
	    
		if(max_v <weightedWhiteValues[j][i]){max_v = weightedWhiteValues[(j)][ i];}
		
	    }
	   
	   
	}
	  




// 	 for ( int j =0; j< H; j++){
// 	     for ( int i = 0; i< W; i++){
// 	            if(j <  m_Top  ){
// 		       weightedWhiteValues[j][  i] = 0;
// 		      
// 		    }
// 		    else{
// 		     weightedWhiteValues[j][  i] = (weightedWhiteValues[j][ i]-0.0)/(max_v-0.0) *255;  
// 		
// 		    }
// 	     }
// 	   
// 	}

	 for ( int j =  m_Top ; j< H; j++){
	     for ( int i = 0; i< W; i++){
		     weightedWhiteValues[j][ i] = (weightedWhiteValues[j][ i]-0.0)/(max_v-0.0) *255;  

	     }
	   
	}


    
//     std::vector<float > test(H * W,0);
//       for ( int i = 0; i< W; i++){  
//       for ( int j =0 ; j< H; j++){  
// 	  test[(j)*W + i] =  weightedWhiteValues[j][ i] ;
// 	
//       }
// 	
//      }
//       tools.cvshowImg(test, "test");
}




void FindLines::RetrieveSkeleton (/* in */cv:: Mat & fieldConvexHull, /* in */ const float ( & matrix )[ H ][ W ] , /* out */  std::vector<cv::Point> &detectedPoins ){

// 	    skeletonMatrix_tmp = Mat::zeros(fieldConvexHull.size(), CV_8UC1);
            skeletonMatrix_tmp = Mat::zeros(cv::Size(W, H - m_Top ), CV_8UC1);
	    
	    detectedPoins.clear();
	    detectedPoinsWithType.clear();
            // * Loop through rows. * //
            for ( int x = 1; x < W - 1; x++ ) { // [0]
                // * Loop through columns. * //
                // * Process the pixels under m_FieldBoundary[x]-params.field.FieldBoarder->get()/2 for skeleton. * //
                for ( int y = m_Top; y < H; y++ ) {// [1]
		    if ( fieldConvexHull.at<uchar>(y,x ) == 0 ) { // [2]  //at<uchar>(i,j) i: rows, j: cols
                       continue;
                    }
                    register unsigned int vote = 0;
                    register int val = matrix[ y ][ x ];
		    if ( val < params.line.MinSkeletonValue->get() ) { // [2]
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

void FindLines::RetrieveSkeleton2 ( /* in */ const int ( & matrix )[ H ][ W ] , /* out */  std::vector<cv::Point> &detectedPoins ){

	    detectedPoins.clear();
	    detectedPoinsWithType.clear();
            // * Loop through rows. * //
            for ( int x = 1; x < W - 1; x++ ) { // [0]
                // * Loop through columns. * //
                // * Process the pixels under m_FieldBoundary[x]-params.field.FieldBoarder->get()/2 for skeleton. * //
                for ( int y = m_Top; y < H; y++ ) {// [1]

                    register unsigned int vote = 0;
                    register int val = matrix[ y ][ x ];
		    if ( val == 0 ) { // [2]
                       continue;
                    }
                    else { // [2]
                        if ( val < matrix[ y ][ x - 1 ] ) { // [3]
                            vote++;
                        }
                        if ( val < matrix[ y ][ x + 1 ] ) { // [3]
                            vote++;
                        }
                        if ( val < matrix[ (y - 1) ][ x ] ) { // [3]
                            vote++;
                        }
                        if ( val < matrix[ (y + 1) ][ x ] ) { // [3]
                            vote++;
                        }
                        if ( vote <= 3 ) { // [3]
                            if ( val < matrix[ (y - 1) ][ x - 1 ] ) { // [4]
                                vote++;
                            }
                            if ( val < matrix[ (y + 1)][ x - 1 ] ) { // [4]
                                vote++;
                            }
                            if ( vote < 3 ) { // [4]
                                if ( val < matrix[ (y - 1) ][ x + 1 ] ) { // [5]
                                    vote++;
                                }
                                if ( val < matrix[ (y + 1) ][ x + 1 ] ) { // [5]
                                    vote++;
                                }
                                if ( vote < 3 ) { // [5]
				     detectedPoins.push_back(cv::Point(x , y)); 
				     detectedPoinsWithType.push_back(make_pair(cv::Point(x , y), vote)); 
                                }// [5]
                            }// [4]

                        }// [3]
                    } // [2]
                } // [1]
            } // [0]
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
      
//       	tools.cvshowImg(CamFrm.weightedWhiteValues, "weightedWhiteValues");
//         //the output file will be saved in package "tmp_file"
// 	tools.writePGM(CamFrm.weightedWhiteValues, "weightedWhiteValues");
// 	tools.writeDAT(CamFrm.weightedWhiteValues, "weightedWhiteValues.dat");
	
//         tools.cvshowImg(CamFrm.weightedWhiteValues, "smothWhiteValues");
    }
    
   

}

//by Hafez
// bool FindLines::GetLines(Mat &rawHSV, Mat & fieldMask,
// 		Mat &guiImg, bool SHOWGUI, const Mat &lineBinary,
// 		vector<LineSegment> &resLines)
// {
// 	int MIN_LINE_DOUBLE_VOTE = params.line.lineVoteDouble->get();
// 	int MIN_LINE_VOTE = params.line.HoughLineVoteThreshold->get();
// 	int MIN_LINE_COLOR_VOTE = params.line.lineVoteColor->get();
// 	
// 	int rho = params.line.HoughLineRho->get(),
//           threshold= params.line.HoughLineVoteThreshold->get(),
// 	  minLineLength= params.line.HoughLineMinLineLength->get(),
// 	  maxLineGap = params.line.HoughLinemaxLineGap->get();
// 	
// 	
// 	vector<Vec4i> linesFromHoughP;
// 	HoughLinesP(lineBinary, linesFromHoughP, 1, M_PI / 45, 20,
// 			params.line.MinLineLength->get(), 20);
// 
// 	for (size_t i = 0; i < linesFromHoughP.size(); i++)
// 	{
// 		Vec4i lP = linesFromHoughP[i];
// 		LineSegment tmpLine(Point2d(lP[0], lP[1]), Point2d(lP[2], lP[3]));
// 
// 		vector<cv::Point2d> midds = tmpLine.GetMidPoints(3); //2^3+1 = 16
// 		int lineVoter = 0;
// 		int vote_for_double = 0;
// 		int vote_for_color = 0;
// 		uchar *dataImg = fieldMask.data;
// 		//	printf("size= %d\n", midds.size());
// 		for (size_t j = 0; j < midds.size(); j++)
// 		{
// 
// 			int jumpMin = params.line.jumpMin->get(); //pixel
// 			int jumpMax = params.line.jumpMax->get(); //pixel
// 			double distanceToZero = GetDistance(
// 					cv::Point2d((params.camera.width->get() / 2),
// 							(params.camera.height->get())), midds[j]);
// 			LineInterpolation interP(
// 					LineSegment(cv::Point2d(0, jumpMax),
// 							cv::Point2d(params.camera.height->get(), jumpMin)));
// 			double jump;
// 			if (!interP.GetValue(distanceToZero, jump))
// 			{
// 				//printh(CRed, "Error In Programming!");
// 				continue;
// 			}
// 			LineSegment tocheck = tmpLine.PerpendicularLineSegment(jump,
// 					midds[j]);
// 			cv::LineIterator it(lineBinary, tocheck.P1, tocheck.P2, 8);
// 
// 			vector<uchar> buf(it.count);
// 
// 			int currentCounter = 0; //to make sure that each point can insrease just 2 times
// 			for (int k = 0; k < it.count; k++, ++it)
// 			{
// 				uchar val=*(*it);
// 				if ( val > 10)
// 				{
// 					vote_for_double++;
// 					currentCounter++;
// 				}
// 				if (currentCounter >= 2) //to make sure that each point can insrease just 2 times
// 					break;
// 			}
// 
// 			cv::LineIterator itHSV(rawHSV, tocheck.P1, tocheck.P2, 8);
// 
// 			vector<uchar> bufHSV(itHSV.count);
// 
// 			for (int k = 0; k < itHSV.count; k++, ++itHSV)
// 			{
// 				cv::Vec3b hsvC = (cv::Vec3b) *itHSV;
// 				if (hsvC[0] >= params.line.h0->get()
// 						&& hsvC[0] <= params.line.h1->get()
// 						&& hsvC[1] >= params.line.s0->get()
// 						&& hsvC[1] <= params.line.s1->get()
// 						&& hsvC[2] >= params.line.v0->get()
// 						&& hsvC[2] <= params.line.v1->get())
// 				{
// 					vote_for_color++;
// 					break; //to make sure that each point can insrease just 1 times
// 				}
// 			}
// 
// 			int safeToShow = 0;
// 			if (tocheck.P1.x >= 0 && tocheck.P1.y >= 0
// 					&& tocheck.P1.x < params.camera.width->get()
// 					&& tocheck.P1.y < params.camera.height->get())
// 			{
// 				safeToShow++;
// 				uchar* pixelP = dataImg
// 						+ ((((int) tocheck.P1.y * params.camera.width->get())
// 								+ (int) tocheck.P1.x) * 1);
// 				if (*pixelP > 50)
// 					lineVoter++;
// 			}
// 			if (tocheck.P2.x >= 0 && tocheck.P2.y >= 0
// 					&& tocheck.P2.x < params.camera.width->get()
// 					&& tocheck.P2.y < params.camera.height->get())
// 			{
// 				safeToShow++;
// 				uchar* pixelP = dataImg
// 						+ ((((int) tocheck.P2.y * params.camera.width->get())
// 								+ (int) tocheck.P2.x) * 1);
// 				if (*pixelP > 50)
// 					lineVoter++;
// 			}
// 
// 			if (safeToShow >= 2)
// 			{
// 				if (SHOWGUI && params.line.showDebug2->get())
// 				{
// 					cv::line(guiImg, tocheck.P1, tocheck.P2, yellowColor(), 1);
// 				}
// 			}
// 		}
// 		if (lineVoter > MIN_LINE_VOTE && vote_for_double > MIN_LINE_DOUBLE_VOTE
// 				&& vote_for_color > MIN_LINE_COLOR_VOTE)
// 			resLines.push_back(tmpLine);
// 
// 	}
// 	return resLines.size() > 0;
// 
// }



	
void FindLines::findhoughLines(std::vector<cv::Point> & undistortedPoints){
  
      LinesOnImg.clear();

      int y_min = boundingRect(undistortedPoints).y;

      cv::Mat gray=Mat::zeros(Size(siX, siY-y_min), CV_8UC1);
      
      
      for ( unsigned int i = 0; i <undistortedPoints.size() ; i++ ) { 
	  gray.at<uchar>(undistortedPoints[i].y - y_min, undistortedPoints[i].x ) = 250;	
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
	    Vec2i  s(hough_lines[i][0],hough_lines[i][1] + y_min );
	    Vec2i  e(hough_lines[i][2],hough_lines[i][3] + y_min);
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
   cv::Mat vis_houghLines(siY,siX,CV_8UC3,cv::Scalar(150, 150, 150));
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
      
      for(unsigned int i =0; i<m_LineBuffer_After_Merge.size(); ++i){
     
	  cv::line( vis_houghLines, cv::Point(m_LineBuffer_After_Merge[i].s[0], m_LineBuffer_After_Merge[i].s[1] ), 
		    cv::Point(m_LineBuffer_After_Merge[i].e[0],m_LineBuffer_After_Merge[i].e[1]), 
		    cv::Scalar(0, 100, 200), 4, 0 );
          }
//       std::ostringstream ss0;
//       ss0 << i;
//       cv::putText(vis_houghLines,ss0.str(), cv::Point((m_LineBuffer_tmp[i].s[0]+m_LineBuffer_tmp[i].e[0])/2-20,
// 						      (m_LineBuffer_tmp[i].s[1]+m_LineBuffer_tmp[i].e[1])/2 -20),
// 					    cv::FONT_HERSHEY_TRIPLEX,1,cv::Scalar(0,0,0),2);
      //vis**************
  	cv::imshow("houghLines" ,vis_houghLines);
	cv::waitKey(1);
     

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


