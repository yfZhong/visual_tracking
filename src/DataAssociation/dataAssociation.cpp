
#include <visual_tracking/DataAssociation/dataAssociation.h>


 DataAssociation::DataAssociation(){
   
}


//  void DataAssociation::generateModel(geometry_msgs::Pose &p, FrameGrabber & CamFrm ){
//         CamFrm.forw_Proj.setRobotPose(p);
// 	CamFrm.forw_Proj. GetModelLineComps();
// }


 void DataAssociation::AssociateDetectionToModel(FrameGrabber & CamFrm, FindNodes &NodeFinder, float &error){
   
     DetectionToModelCorrespondences.clear();
     DetectionToModelOutliers.clear();
     
     LinearGraph_Buffer  * m_LinearGraph_Buffer = NodeFinder.m_LinearGraph_Buffer;
     ModelLine_Buffer  * m_ModelLine_Buffer= CamFrm.forw_Proj.m_ModelLine_Buffer;
     
     int MaxAngDiff = params.dataAssociation.MaxAngDiff->get();
     float MaxErrorForInlier = params.dataAssociation.MaxErrorForInlier->get();
     float MinCompLength = params.graphNode._MinCmpLineLength->get();
     float MaxAngleChange= params.dataAssociation._MaxAngleChange->get();
     
      LinearGraph_Buffer::iterator it_1;
      ModelLine_Buffer::iterator it_2;
      
      
      error = 0;

      int i=-1;int j=-1;
      
      
      //for each comp, find the closest comp
      for ( it_1 = m_LinearGraph_Buffer->begin( ); it_1 != m_LinearGraph_Buffer->end( ); it_1++ ) { // 
	  
	
	  ++i;
	  if((*it_1).sumOfLength < MinCompLength){continue;}
	  if((*it_1).Points.size() < 2){continue;}

	  vector<Vec2i> &points = it_1->Points;
	  float undistortedAngleAvg =  it_1->undistortedAngleAvg;
	  float undistortedAngleChangeAvg=  it_1->undistortedAngleChangeAvg;
	  
	  float minCompError =  std::numeric_limits<float>::max();
	  float minComp_j = -1;
	  j=-1;
	  
	  // fine the closest model comp
	  for ( it_2 =  m_ModelLine_Buffer->begin( ); it_2 != m_ModelLine_Buffer->end( ); it_2++) {
	      ++j;
// 	      cout<<"(*it_2).id        "<<(*it_2).id<<endl;
	      
// 	       cout<<"(*it_2).sumOfLengthUndistorted  "<<(*it_2).sumOfLengthUndistorted<< "  "<<j<<endl;
	      if((*it_2).sumOfLengthUndistorted < MinCompLength){continue;}
	      if((*it_2).UndistortedPoints.size() < 2){continue;}
	      float compError =0;
	      
	      if( (*it_2).id >=0 ){

		  Line &LongLine = (*it_2).LongLine;
// 		
		  if(m_Math::AngDif( undistortedAngleAvg , LongLine.ang) > MaxAngDiff ){continue;}
		  
		  // get the shortest distance of each point to this line segment. 
		  for( int n = 0; n< points.size(); n++){
		    compError += m_Math::Point2LineSegDistance( points[n], LongLine);
 
		  }
		  
// 		  cout<< "j "<<j<<"    "<<compError<<endl;
		  if( compError < minCompError ){
			minCompError = compError; 
			minComp_j = j;
		  }
	      }
	      else{
// 		 if((*it_1).undistortedAngleChangeAvg <MaxAngleChange && points.size()>5 ){ continue;}
		
		  vector<Line>  &UndistortedLines = (*it_2).UndistortedLines;
		  // for each point, project to the line seg with shotest Point2LineSegDistance(
		  int num_anglediff_outside_range=0;
		  for(unsigned int n = 0; n< points.size(); n++){
// 		    compError += m_Math::Point2LineSegDistance( points[n], LongLine);
		         Vec2i p = points[n];
		          
			  int min_l=-1; float mindist =  std::numeric_limits<float>::max();
			  for(unsigned int l = 0; l< UndistortedLines.size(); l++){
			    
			      Line &line = UndistortedLines[l];
			     
			      
			      float dist_tmp =  m_Math::Point2LineSegDistance( p,line );
			      
			      if( dist_tmp < mindist ){
				      mindist = dist_tmp; 
				      min_l = l;
				      if(m_Math::AngDif( undistortedAngleAvg , line.ang) > MaxAngDiff ){
				      num_anglediff_outside_range++;}
				
			      } 
	
			  }
			  
			  if( min_l >=0 ){//nth point project to min_l_th line
			     compError+= mindist;
			  }

		  }
		  
		  float num_anglediff_outside_range_Pct = float(num_anglediff_outside_range)/float(points.size());
		
	          if( compError < minCompError  && num_anglediff_outside_range_Pct<= 0.5){
			minCompError = compError; 
			minComp_j = j;
		  }
	      }
	    
	  }
	  
          float maxerror_tmp= MaxErrorForInlier;
	  
	  if(points.size()>20 &&  undistortedAngleChangeAvg <10){maxerror_tmp += 3*maxerror_tmp; }
	  else if (points.size()>=10){maxerror_tmp += 2*maxerror_tmp ;}
	  else if (points.size()>=5){maxerror_tmp += maxerror_tmp ;}
	  
	
	 
	  if(minComp_j >=0 && minCompError / float(points.size()) > maxerror_tmp ){//This is an outlier comp
	         for( unsigned int n = 0; n< points.size(); n++){  
		      DetectionToModelOutliers.push_back( points[n] );
		  }

	  }
	  else if( minComp_j >=0 ){//copm i project to minComp_j;
	      error+= minCompError * minCompError;
	      
	      //generate the correspondence

	      if(  (*m_ModelLine_Buffer)[minComp_j].id >=0 ){
		  Line &LongLine =  (*m_ModelLine_Buffer)[minComp_j].LongLine;
		  for(unsigned int n = 0; n< points.size(); n++){
		    Vec2i closestP = m_Math::getClosestPointOnLine( points[n], LongLine);
		    DetectionToModelCorrespondences.push_back( make_pair(points[n] , closestP));
		  }
	      }
	      else{
		  vector<Line>  &UndistortedLines =  (*m_ModelLine_Buffer)[minComp_j].UndistortedLines;
//                   cout<<"UndistortedLines.size() "<<UndistortedLines.size()<<endl;
		  // for each point, project to the line seg with shotest Point2LineSegDistance(
		  for(unsigned int n = 0; n< points.size(); n++){
// 		    compError += m_Math::Point2LineSegDistance( points[n], LongLine);
		         Vec2i p = points[n];
		          
			  int min_l=-1; float mindist =  std::numeric_limits<float>::max();
			  
			  
			  
			  for( unsigned int l = 0; l< UndistortedLines.size(); l++){
			    
			      Line &line = UndistortedLines[l];
// 			      float len = line.len;
// 			      cout<< len<<endl;
			      float dist_tmp =  m_Math::Point2LineSegDistance( p,line );
// 			      cout<< "dist_tmp "<<dist_tmp<<endl;
			      if( dist_tmp < mindist ){
				      mindist = dist_tmp; 
				      min_l = l;
			      } 
			  }
			  
			  if( min_l >=0 ){//nth point project to min_l_th line
			     Vec2i closestP = m_Math::getClosestPointOnLine(p, UndistortedLines[min_l]);
		             DetectionToModelCorrespondences.push_back( make_pair(p , closestP));
			  }
 
		  }
	       }

          }
          
   
      }
    
//      cout<<"DetectionToModelCorrespondences.size   "<<DetectionToModelCorrespondences.size()<<endl;
     
     
      float num_points =float(DetectionToModelCorrespondences.size()) +  DetectionToModelOutliers.size();
      DTMInlierPct = float(DetectionToModelCorrespondences.size()) / ( num_points +1 ) ;
      
//       error = error/float(DetectionToModelCorrespondences.size() +1) ;
      
      
      
      DTMError = error;
      
 }

 
 
 
  
  void DataAssociation::AssociateModelToDetection(FrameGrabber & CamFrm, FindNodes &NodeFinder, float& error){
  
     ModelToDetectionCorrespondences.clear();
     ModelToDetectionOutliers.clear();
     
     LinearGraph_Buffer  * m_LinearGraph_Buffer = NodeFinder.m_LinearGraph_Buffer;
     ModelLine_Buffer  * m_ModelLine_Buffer= CamFrm.forw_Proj.m_ModelLine_Buffer;
    
     float MinLength_Model = params.graphNode._MinCmpLineLength->get();
     int MaxAngDiff = params.dataAssociation.MaxAngDiff->get();
     float MaxErrorForInlier = params.dataAssociation.MaxErrorForInlier->get();
     float MinLength_ref = params.dataAssociation._MinCompLengthForModelError->get();
     float MaxAngleChange= params.dataAssociation._MaxAngleChange->get();
     
     
      ModelLine_Buffer::iterator it_1;
      LinearGraph_Buffer::iterator it_2;
      
      error = 0;
 

      int i=-1;int j=-1;
      
      //for each comp, find the closest comp
      for ( it_1 = m_ModelLine_Buffer->begin( ); it_1 != m_ModelLine_Buffer->end( ); it_1++ ) { // 
// 	  cout<<"i  "<<endl;
	      ++i;
	      if((*it_1).sumOfLengthUndistorted < MinLength_Model){continue;}
	      if((*it_1).UndistortedPoints.size() < 2){continue;}

	      vector<Vec2i>  & UndistortedPoints= (*it_1).UndistortedPoints;
	      vector<Line>  & Angles= (*it_1).UndistortedLines;

	     
	       for(unsigned int  pi = 0; pi<UndistortedPoints.size(); pi++) {
		  float point_ang;
		 if (pi== UndistortedPoints.size()-1){ point_ang = Angles[ pi-1].ang; }
		 else{ point_ang =Angles[ pi].ang; }

		 
		  float min_dist=std::numeric_limits<float>::max();
	          int min_j=-1, min_l = -1;
		  j=-1;
		  // fine the closest model comp
		  for ( it_2 =  m_LinearGraph_Buffer->begin( ); it_2 != m_LinearGraph_Buffer->end( ); it_2++) {
			j++;
			
			
			//stright line
			if((*it_1).id>=0 && (*it_2).undistortedAngleChangeAvg >MaxAngleChange ){continue;}
			if((*it_1).id>=0 &&  m_Math::AngDif( (*it_1).LongLine.ang, (*it_2).undistortedAngleAvg )>MaxAngDiff ) {continue;}

		        if((*it_2).sumOfLengthUndistorted  < MinLength_ref){continue;}
			

			vector<Line>  & lines = (*it_2).UndistortedLines;
			
			
			for(unsigned int  l = 0; l<lines.size(); l++) {
			      
			      if( m_Math::AngDif(point_ang,  lines[l].ang)>MaxAngDiff){continue;}
			    
			      float dist_tmp= m_Math::Point2LineSegDistance(UndistortedPoints[pi], lines[l]);
			
			      if( dist_tmp < min_dist ){
				      min_dist = dist_tmp; 
				      min_j = j;
				      min_l = l;
			      }
	  
			}
			   
		   
		  }
		      
		if(min_j >=0  && min_dist > MaxErrorForInlier ){//This is an outlier comp
			    ModelToDetectionOutliers.push_back( UndistortedPoints[pi] );

		}
		else if( min_j >=0 && min_l >=0 ){//copm i project to minComp_j;
		    error += min_dist *min_dist;
		    
		    Line l = (*m_LinearGraph_Buffer)[min_j].UndistortedLines[min_l];
		    Vec2i closestP =  m_Math::getClosestPointOnLine( UndistortedPoints[pi] ,l );
		    ModelToDetectionCorrespondences.push_back( make_pair(UndistortedPoints[pi] , closestP));  
	      }

	   }
	  
	  
      }
	          
      
      float num_points =float(ModelToDetectionCorrespondences.size()) +  ModelToDetectionOutliers.size();
      MTDInlierPct = float(ModelToDetectionCorrespondences.size()) / ( num_points +1 ) ;
      
//       error = error/float(DetectionToModelCorrespondences.size() +1) ;
      
      MTDError = error;
}
 
 
 
 
 
 
/* 
  void DataAssociation::AssociateModelToDetection(FrameGrabber & CamFrm, FindNodes &NodeFinder, float& error){
  
     ModelToDetectionCorrespondences.clear();
     ModelToDetectionOutliers.clear();
     
     LinearGraph_Buffer  * m_LinearGraph_Buffer = NodeFinder.m_LinearGraph_Buffer;
     ModelLine_Buffer  * m_ModelLine_Buffer= CamFrm.forw_Proj.m_ModelLine_Buffer;
    
     float MinLength_Model = params.graphNode._MinCmpLineLength->get();
     int MaxAngDiff = params.dataAssociation.MaxAngDiff->get();
     float MaxErrorForInlier = params.dataAssociation.MaxErrorForInlier->get();
     float MinLength_ref = params.dataAssociation._MinCompLengthForModelError->get();
     float MaxAngleChange= params.dataAssociation._MaxAngleChange->get();
     
     
      ModelLine_Buffer::iterator it_1;
      LinearGraph_Buffer::iterator it_2;
      
      error = 0;
 

      int i=-1;int j=-1;
      
      //for each comp, find the closest comp
      for ( it_1 = m_ModelLine_Buffer->begin( ); it_1 != m_ModelLine_Buffer->end( ); it_1++ ) { // 
	  
	      ++i;
	      if((*it_1).sumOfLengthUndistorted < MinLength_Model){continue;}
	      if((*it_1).UndistortedPoints.size() < 2){continue;}

	      vector<Vec2i>  & UndistortedPoints= (*it_1).UndistortedPoints;
	      vector<Line>  & UndistortedLines= (*it_1).UndistortedLines;

	      float minCompError =  std::numeric_limits<float>::max();
	      float minComp_j = -1;
	      j=-1;
              
		  // fine the closest model comp
	      for ( it_2 =  m_LinearGraph_Buffer->begin( ); it_2 != m_LinearGraph_Buffer->end( ); it_2++) {
		j++;
		
		
		 //stright line
		 if((*it_1).id>=0 && (*it_2).undistortedAngleChangeAvg >MaxAngleChange ){continue;}
		 if((*it_1).id>=0 &&  m_Math::AngDif( (*it_1).LongLine.ang, (*it_2).undistortedAngleAvg )>MaxAngDiff ) {continue;}
		 if((*it_2).sumOfLengthUndistorted  < MinLength_ref){continue;}
		 
		 float compError =0;
		 
		 vector<Line>  & lines = (*it_2).UndistortedLines;
		 
		 for(unsigned int  pi = 0; pi<UndistortedPoints.size(); pi++) {
	            
	
// 		    compError += m_Math::GetProjectiveDistance(UndistortedPoints[pi], line);

		      float min_dist=std::numeric_limits<float>::max();
		      int min_l=-1;
		      for(unsigned int  l = 0; l<lines.size(); l++) {
			  float dist_tmp= m_Math::GetProjectiveDistance(UndistortedPoints[pi], lines[l]);
		    
			  if( dist_tmp < min_dist ){
				  min_dist = dist_tmp; 
				  min_l = l;
			  }
	
			}
		       if( min_l >=0 ){//nth point project to min_l_th line
			     compError  += min_dist;
		        }
		   
		 }
		 
		 
				
	          if( compError < minCompError ){
			minCompError = compError; 
			minComp_j = j;
		  }
	    
             }
	  
	  
	  



	  if(minComp_j >=0 && minCompError / float(UndistortedPoints.size()) > MaxErrorForInlier ){//This is an outlier comp
	         for( int n = 0; n< UndistortedPoints.size(); n++){  
		      ModelToDetectionOutliers.push_back( UndistortedPoints[n] );
		  }

	  }
	  else if( minComp_j >=0 ){//copm i project to minComp_j;
	      error += minCompError;
	      
	      
// 	      	    int x = 0, y=0;
// 		    
// 		    vector<Vec2i>  & refPs = (*m_LinearGraph_Buffer)[minComp_j].Points;
// 		    for(unsigned int  n = 0; n<refPs.size(); n++) {
// 		         x+= refPs[n][0];    y += refPs[n][1];
// 		    }
// 		    x /=refPs.size();  y /=refPs.size();
// 		    float ang =  (*m_LinearGraph_Buffer)[minComp_j].undistortedAngleAvg;
// 		    Vec2i s(x + cos(ang)*10 ,  y +  (sin(ang)*10 ));
// 		    Vec2i e(x - cos(ang)*10 ,  y -  (sin(ang)*10 ));
// 		    Line line( 0 , s, e);
// 		 
// 
// 	            for(unsigned int  pi = 0; pi<UndistortedPoints.size(); pi++) {
// 		             Vec2i  pProject;
// 			     m_Math::GetProjectivePoint( UndistortedPoints[pi] , line, pProject);
// 		             ModelToDetectionCorrespondences.push_back( make_pair(UndistortedPoints[pi] , pProject));   
// 
// 		      }
// 	      
	      
	      
	      
	         for(unsigned int  pi = 0; pi<UndistortedPoints.size(); pi++) {
		     
		      float min_dist=std::numeric_limits<float>::max();
		      int min_l=-1;
		      vector<Line>  & lines = (*m_LinearGraph_Buffer)[minComp_j].UndistortedLines;
		    
		      
		      for(unsigned int  l = 0; l<lines.size(); l++) {
			  
			  float dist_tmp= m_Math::GetProjectiveDistance(UndistortedPoints[pi], lines[l]);
		    
			  if( dist_tmp < min_dist ){
				  min_dist = dist_tmp; 
				  min_l = l;
			  }
			
		      }
		      
		       if( min_l >=0 ){//nth point project to min_l_th line
			     Vec2i  pProject;
			     m_Math::GetProjectivePoint( UndistortedPoints[pi] , lines[min_l], pProject);
		             ModelToDetectionCorrespondences.push_back( make_pair(UndistortedPoints[pi] , pProject));    
		        }
		   
		 }
	      
	  } 
	  
	  
	  
    }      
	          
      
      float num_points =float(ModelToDetectionCorrespondences.size()) +  ModelToDetectionOutliers.size();
      MTDInlierPct = float(ModelToDetectionCorrespondences.size()) / ( num_points +1 ) ;
      
      error = error/float(DetectionToModelCorrespondences.size() +1) ;
      
      MTDError = error;
}
 
 */
 
 
 
 

//  void DataAssociation::AssociateModelToDetection(FrameGrabber & CamFrm, FindNodes &NodeFinder, float& error){
//   
//      ModelToDetectionCorrespondences.clear();
//      ModelToDetectionOutliers.clear();
//      
//      LinearGraph_Buffer  * m_LinearGraph_Buffer = NodeFinder.m_LinearGraph_Buffer;
//      ModelLine_Buffer  * m_ModelLine_Buffer= CamFrm.forw_Proj.m_ModelLine_Buffer;
//     
//      float MinLength_Model = params.graphNode._MinCmpLineLength->get();
//      int MaxAngDiff = params.dataAssociation.MaxAngDiff->get();
//      float MaxErrorForInlier = params.dataAssociation.MaxErrorForInlier->get();
//      float MinLength_ref = params.dataAssociation._MinCompLengthForModelError->get();
//      float MaxAngleChange= params.dataAssociation._MaxAngleChange->get();
//      
//      
//       ModelLine_Buffer::iterator it_1;
//       LinearGraph_Buffer::iterator it_2;
//       
//       error = 0;
//  
// 
//       int i=-1;int j=-1;
//       
//       //for each comp, find the closest comp
//       for ( it_1 = m_ModelLine_Buffer->begin( ); it_1 != m_ModelLine_Buffer->end( ); it_1++ ) { // 
// 	  
// 	      ++i;
// 	      if((*it_1).sumOfLengthUndistorted < MinLength_Model){continue;}
// 	      if((*it_1).UndistortedPoints.size() < 2){continue;}
// 
// 	      vector<Vec2i>  & UndistortedPoints= (*it_1).UndistortedPoints;
// 	      vector<Line>  & UndistortedLines= (*it_1).UndistortedLines;
// 
// 	      float minCompError =  std::numeric_limits<float>::max();
// 	      float minComp_j = -1;
// 	      j=-1;
//               
// 		  // fine the closest model comp
// 	      for ( it_2 =  m_LinearGraph_Buffer->begin( ); it_2 != m_LinearGraph_Buffer->end( ); it_2++) {
// 		j++;
// 		
// 		
// 		 //stright line
// 		 if((*it_1).id>=0 && (*it_2).undistortedAngleChangeAvg >MaxAngleChange ){continue;}
// 		 if((*it_1).id>=0 &&  m_Math::AngDif( (*it_1).LongLine.ang, (*it_2).undistortedAngleAvg )>MaxAngDiff ) {continue;}
// 		 if((*it_2).sumOfLengthUndistorted  < MinLength_ref){continue;}
// 		 
// 		 float compError =0;
// 		 
// 		 vector<Line>  & TangentLines = (*it_2).TangentLines;
// 		 vector<Vec2i>  & refPs = (*it_2).Points;
// 		 if(refPs.size()<2){continue;}
// 		 
// 		 
// 		 //avg line, center in the avg pose with avg ang
// 		    int x = 0, y=0;
// 		    for(unsigned int  n = 0; n<refPs.size(); n++) {
// 		         x+= refPs[n][0];    y += refPs[n][1];
// 		    }
// 		    x /=refPs.size();  y /=refPs.size();
// 		    float ang = (*it_2).undistortedAngleAvg;
// 		    Vec2i s(x + cos(ang)*10 ,  y +  (sin(ang)*10 ));
// 		    Vec2i e(x - cos(ang)*10 ,  y -  (sin(ang)*10 ));
// 		    Line line( 0 , s, e);
// 		 
// 		 
// 		 
// 		 for(unsigned int  pi = 0; pi<UndistortedPoints.size(); pi++) {
// 	            
// 	
// 		    compError += m_Math::GetProjectiveDistance(UndistortedPoints[pi], line);
// 		   
// 		   
// /*		   
// 		      float min_dist=std::numeric_limits<float>::max();
// 		      int min_l=-1;
// 		      for(unsigned int  l = 0; l<TangentLines.size(); l++) {
// 			  float dist_tmp= m_Math::GetProjectiveDistance(UndistortedPoints[pi], TangentLines[l]);
// 		    
// 			  if( dist_tmp < min_dist ){
// 				  min_dist = dist_tmp; 
// 				  min_l = l;
// 			  }
// 	
// 			}
// 		       if( min_l >=0 ){//nth point project to min_l_th line
// 			     compError  += min_dist;
// 		        }*/
// 		   
// 		 }
// 		 
// 		 
// 				
// 	          if( compError < minCompError ){
// 			minCompError = compError; 
// 			minComp_j = j;
// 		  }
// 	    
//              }
// 	  
// 	  
// 	  
// 
// 
// 
// 	  if(minComp_j >=0 && minCompError / float(UndistortedPoints.size()) > MaxErrorForInlier ){//This is an outlier comp
// 	         for( int n = 0; n< UndistortedPoints.size(); n++){  
// 		      ModelToDetectionOutliers.push_back( UndistortedPoints[n] );
// 		  }
// 
// 	  }
// 	  else if( minComp_j >=0 ){//copm i project to minComp_j;
// 	      error += minCompError;
// 	      
// 	      
// 	      	    int x = 0, y=0;
// 		    
// 		    vector<Vec2i>  & refPs = (*m_LinearGraph_Buffer)[minComp_j].Points;
// 		    for(unsigned int  n = 0; n<refPs.size(); n++) {
// 		         x+= refPs[n][0];    y += refPs[n][1];
// 		    }
// 		    x /=refPs.size();  y /=refPs.size();
// 		    float ang =  (*m_LinearGraph_Buffer)[minComp_j].undistortedAngleAvg;
// 		    Vec2i s(x + cos(ang)*10 ,  y +  (sin(ang)*10 ));
// 		    Vec2i e(x - cos(ang)*10 ,  y -  (sin(ang)*10 ));
// 		    Line line( 0 , s, e);
// 		 
// 
// 	            for(unsigned int  pi = 0; pi<UndistortedPoints.size(); pi++) {
// 		             Vec2i  pProject;
// 			     m_Math::GetProjectivePoint( UndistortedPoints[pi] , line, pProject);
// 		             ModelToDetectionCorrespondences.push_back( make_pair(UndistortedPoints[pi] , pProject));   
// 
// 		      }
// 	      
// 	      
// 	      
// 	      
// // 	       for(unsigned int  pi = 0; pi<UndistortedPoints.size(); pi++) {
// // 		     
// // 		      float min_dist=std::numeric_limits<float>::max();
// // 		      int min_l=-1;
// // 		      vector<Line>  & TangentLines = (*m_LinearGraph_Buffer)[minComp_j].TangentLines;
// // 		      
// // 		      
// // 		      for(unsigned int  l = 0; l<TangentLines.size(); l++) {
// // 			  
// // 			  float dist_tmp= m_Math::GetProjectiveDistance(UndistortedPoints[pi], TangentLines[l]);
// // 		    
// // 			  if( dist_tmp < min_dist ){
// // 				  min_dist = dist_tmp; 
// // 				  min_l = l;
// // 			  }
// // 			
// // 		      }
// // 		      
// // 		       if( min_l >=0 ){//nth point project to min_l_th line
// // 			     Vec2i  pProject;
// // 			     m_Math::GetProjectivePoint( UndistortedPoints[pi] , TangentLines[min_l], pProject);
// // 		             ModelToDetectionCorrespondences.push_back( make_pair(UndistortedPoints[pi] , pProject));    
// // 		        }
// // 		   
// // 		 }
// 	      
// 	  } 
// 	  
// 	  
// 	  
//     }      
// 	          
//       
//       float num_points =float(ModelToDetectionCorrespondences.size()) +  ModelToDetectionOutliers.size();
//       MTDInlierPct = float(ModelToDetectionCorrespondences.size()) / ( num_points +1 ) ;
//       
//       error = error/float(DetectionToModelCorrespondences.size() +1) ;
//       
//       MTDError = error;
// }
//  
 
 
// void DataAssociation::AssociateModelToDetection(FrameGrabber & CamFrm, FindNodes &NodeFinder, float& error){
//   
//      ModelToDetectionCorrespondences.clear();
//      ModelToDetectionOutliers.clear();
//      
//      LinearGraph_Buffer  * m_LinearGraph_Buffer = NodeFinder.m_LinearGraph_Buffer;
//      ModelLine_Buffer  * m_ModelLine_Buffer= CamFrm.forw_Proj.m_ModelLine_Buffer;
//     
//      float MinLength_Model = params.graphNode._MinCmpLineLength->get();
//      int MaxAngDiff = params.dataAssociation.MaxAngDiff->get();
//      float MaxErrorForInlier = params.dataAssociation.MaxErrorForInlier->get();
//      float MinLength_ref = params.dataAssociation._MinCompLengthForModelError->get();
//      float MaxAngleChange= params.dataAssociation._MaxAngleChange->get();
//      
//      
//       ModelLine_Buffer::iterator it_1;
//       LinearGraph_Buffer::iterator it_2;
//       
//       error = 0;
//  
// 
//       int i=-1;int j=-1;
//       
//       //for each comp, find the closest comp
//       for ( it_1 = m_ModelLine_Buffer->begin( ); it_1 != m_ModelLine_Buffer->end( ); it_1++ ) { // 
// 	  
// 	      ++i;
// 	      if((*it_1).sumOfLengthUndistorted < MinLength_Model){continue;}
// 	      if((*it_1).UndistortedPoints.size() < 2){continue;}
// 
// 	      vector<Vec2i>  & UndistortedPoints= (*it_1).UndistortedPoints;
// 	      vector<Line>  & UndistortedLines= (*it_1).UndistortedLines;
// 
// 	      float minCompError =  std::numeric_limits<float>::max();
// 	      float minComp_j = -1;
// 	      j=-1;
//               
// 		  // fine the closest model comp
// 	      for ( it_2 =  m_LinearGraph_Buffer->begin( ); it_2 != m_LinearGraph_Buffer->end( ); it_2++) {
// 		j++;
// 		
// 		
// 		 //stright line
// 		 if((*it_1).id>=0 && (*it_2).undistortedAngleChangeAvg >MaxAngleChange ){continue;}
// 		 if((*it_1).id>=0 &&  m_Math::AngDif( (*it_1).LongLine.ang, (*it_2).undistortedAngleAvg )>MaxAngDiff ) {continue;}
// 		 if((*it_2).sumOfLengthUndistorted  < MinLength_ref){continue;}
// 		 
// 		 float compError =0;
// 		 
// 		 vector<Line>  & TangentLines = (*it_2).TangentLines;
// 		 for(unsigned int  pi = 0; pi<UndistortedPoints.size(); pi++) {
// 	
// 		      float min_dist=std::numeric_limits<float>::max();
// 		      int min_l=-1;
// 		      for(unsigned int  l = 0; l<TangentLines.size(); l++) {
// 			  float dist_tmp= m_Math::GetProjectiveDistance(UndistortedPoints[pi], TangentLines[l]);
// 		    
// 			  if( dist_tmp < min_dist ){
// 				  min_dist = dist_tmp; 
// 				  min_l = l;
// 			  }
// 	
// 			}
// 		       if( min_l >=0 ){//nth point project to min_l_th line
// 			     compError  += min_dist;
// 		        }
// 		   
// 		 }
// 		 
// 		 
// 				
// 	          if( compError < minCompError ){
// 			minCompError = compError; 
// 			minComp_j = j;
// 		  }
// 	    
//              }
// 	  
// 	  
// 	  
// 
// 
// 
// 	  if(minComp_j >=0 && minCompError / float(UndistortedPoints.size()) > MaxErrorForInlier ){//This is an outlier comp
// 	         for( int n = 0; n< UndistortedPoints.size(); n++){  
// 		      ModelToDetectionOutliers.push_back( UndistortedPoints[n] );
// 		  }
// 
// 	  }
// 	  else if( minComp_j >=0 ){//copm i project to minComp_j;
// 	      error += minCompError;
// 	      
// 	       for(unsigned int  pi = 0; pi<UndistortedPoints.size(); pi++) {
// 		     
// 		      float min_dist=std::numeric_limits<float>::max();
// 		      int min_l=-1;
// 		      vector<Line>  & TangentLines = (*m_LinearGraph_Buffer)[minComp_j].TangentLines;
// 		      
// 		      for(unsigned int  l = 0; l<TangentLines.size(); l++) {
// 			  
// 			  float dist_tmp= m_Math::GetProjectiveDistance(UndistortedPoints[pi], TangentLines[l]);
// 		    
// 			  if( dist_tmp < min_dist ){
// 				  min_dist = dist_tmp; 
// 				  min_l = l;
// 			  }
// 			
// 		      }
// 		      
// 		       if( min_l >=0 ){//nth point project to min_l_th line
// 			     Vec2i  pProject;
// 			     m_Math::GetProjectivePoint( UndistortedPoints[pi] , TangentLines[min_l], pProject);
// 		             ModelToDetectionCorrespondences.push_back( make_pair(UndistortedPoints[pi] , pProject));    
// 		        }
// 		   
// 		 }
// 	      
// 	  } 
// 	  
// 	  
// 	  
//     }      
// 	          
//       
//       float num_points =float(ModelToDetectionCorrespondences.size()) +  ModelToDetectionOutliers.size();
//       ModelToDetectionInlierPct = float(ModelToDetectionCorrespondences.size()) / ( num_points +1 ) ;
//       
//       error = error/float(DetectionToModelCorrespondences.size() +1) ;
//       
//       MTDError = error;
// }