/******* by Yongfeng  2015.01*******/
#include <visual_tracking/DataAssociation/dataAssociation.h>


 DataAssociation::DataAssociation(){
   
}



 void DataAssociation::AssociateDetectionToModel(FrameGrabber & CamFrm, FindNodes &NodeFinder, float &error){
   
     DetectionToModelCorrespondences.clear();
     DetectionToModelOutliers.clear();

     
     
     LinearGraph_Buffer  * m_LinearGraph_Buffer = NodeFinder.m_LinearGraph_Buffer;
     ModelLine_Buffer  * m_ModelLine_Buffer= CamFrm.forw_Proj.m_ModelLine_Buffer;
     
          if( m_LinearGraph_Buffer->size()==0 ||  m_ModelLine_Buffer->size() ==0){return;}
     
     int MaxAngDiff = params.dataAssociation.MaxAngDiff->get();
     float MaxErrorForInlier = params.dataAssociation.MaxErrorForInlier->get();
     float MinCompLength = params.graphNode._MinCmpLineLength->get();
     float MaxAngleAvgChange= params.dataAssociation._MaxAngleChange->get();
     
      LinearGraph_Buffer::iterator it_1;
      ModelLine_Buffer::iterator it_2;
      
      
      error = 0;

      int i=-1;int j=-1;
      
       float sumOfWeights =0;
      //for each comp, find the closest comp
      for ( it_1 = m_LinearGraph_Buffer->begin( ); it_1 != m_LinearGraph_Buffer->end( ); it_1++ ) { // 
	  
	
	  ++i;
	  if((*it_1).sumOfLength < MinCompLength){continue;}
	  if((*it_1).Points.size() < 2){continue;}
	  

	  

	  vector<Vec2i> &points = it_1->Points;
	  vector<float> &weights = it_1->NodeWeights;
	  
	  float sumOfCmpWeights =0;
	  for( int n = 0; n< weights.size(); n++){
	    sumOfCmpWeights+= weights[n];  
	  }

	  float undistortedAngleAvg =  it_1->undistortedAngleAvg;
	  float undistortedAngleChangeAvg=  it_1->undistortedAngleChangeAvg;
	  
	  float minCompError =  std::numeric_limits<float>::max();
	  float minComp_j = -1;
	  j=-1;
	  
	  // fine the closest model comp
	  for ( it_2 =  m_ModelLine_Buffer->begin( ); it_2 != m_ModelLine_Buffer->end( ); it_2++) {
	      ++j;

	      if((*it_2).sumOfLengthUndistorted < MinCompLength){continue;}
	      if((*it_2).UndistortedPoints.size() < 2){continue;}
	      float compError =0;
	      
	      if( (*it_2).id >=0 ){

		  Line &LongLine = (*it_2).LongLine;
// 		
		  if(m_Math::AngDif( undistortedAngleAvg , LongLine.ang) > MaxAngDiff ){continue;}
		  
		  // get the shortest distance of each point to this line segment. 
		  for( int n = 0; n< points.size(); n++){
		    compError += m_Math::Point2LineSegDistance( points[n], LongLine) * weights[n];
 
		  }
		  
		  if( compError < minCompError ){
			minCompError = compError; 
			minComp_j = j;
		  }
	      }
	      else{
// 		 if((*it_1).undistortedAngleChangeAvg <MaxAngleAvgChange && points.size()>15 ){ continue;}
		
		  vector<Line>  &UndistortedLines = (*it_2).UndistortedLines;
		  // for each point, project to the line seg with shotest Point2LineSegDistance(
		  int num_anglediff_outside_range=0;
		  for(unsigned int n = 0; n< points.size(); n++){
// 		    compError += m_Math::Point2LineSegDistance( points[n], LongLine);
		         Vec2i p = points[n];
		          
			  int min_l=-1; float mindist =  std::numeric_limits<float>::max();
			  for(unsigned int l = 0; l< UndistortedLines.size(); l++){
			    
			      Line &line = UndistortedLines[l];
			     
			      
			      float dist_tmp =  m_Math::Point2LineSegDistance( p,line ) ;
			      
			      if( dist_tmp < mindist ){
				      mindist = dist_tmp; 
				      min_l = l;
				      if(m_Math::AngDif( undistortedAngleAvg , line.ang) > MaxAngDiff ){
				      num_anglediff_outside_range++;}
				
			      } 
	
			  }
			  
			  if( min_l >=0 ){//nth point project to min_l_th line
			     compError+= mindist * weights[n];
			  }

		  }
		  
		  float num_anglediff_outside_range_Pct = float(num_anglediff_outside_range)/float(points.size());
		
	          if( compError < minCompError  /*&& num_anglediff_outside_range_Pct<= 0.5*/){
			minCompError = compError; 
			minComp_j = j;
		  }
	      }
	    
	  }
	  
          float maxerror_tmp= MaxErrorForInlier;
	  
	  if(points.size()>20 ){maxerror_tmp += 100*maxerror_tmp; }
	  else if (points.size()>=8){maxerror_tmp += 70*maxerror_tmp ;} 
	  else if (points.size()>=5 &&  undistortedAngleChangeAvg <8){maxerror_tmp += 4* maxerror_tmp ;}
	  else if (points.size()>=5){ maxerror_tmp += 2* maxerror_tmp;} 
	  
	
	 
	  if(minComp_j >=0 && minCompError/sumOfCmpWeights/*/ float(points.size())*/ > maxerror_tmp ){//This is an outlier comp
	         for( unsigned int n = 0; n< points.size(); n++){  
		      DetectionToModelOutliers.push_back( points[n] );
		  }

	  }
	  else if( minComp_j >=0 ){//copm i project to minComp_j;
// 	      error+= minCompError * minCompError;
	      
	      //generate the correspondence
            sumOfWeights += sumOfCmpWeights;
	      if(  (*m_ModelLine_Buffer)[minComp_j].id >=0 ){
		  Line &LongLine =  (*m_ModelLine_Buffer)[minComp_j].LongLine;
		  for(unsigned int n = 0; n< points.size(); n++){
		    Vec2i closestP = m_Math::getClosestPointOnLine( points[n], LongLine);
		    error+= pow(m_Math::TwoPointDistance(points[n], closestP), 2 ) * weights[n];
		    DetectionToModelCorrespondences.push_back( make_pair(points[n] , closestP));
		  }
	      }
	      else{
		  vector<Line>  &UndistortedLines =  (*m_ModelLine_Buffer)[minComp_j].UndistortedLines;
//                   cout<<"UndistortedLines.size() "<<UndistortedLines.size()<<endl;
		  // for each point, project to the line seg with shotest Point2LineSegDistance(
		  for(unsigned int n = 0; n< points.size(); n++){
// 		         compError += m_Math::Point2LineSegDistance( points[n], LongLine);
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
			     error+= pow(m_Math::TwoPointDistance(p, closestP), 2 ) * weights[n];
		             DetectionToModelCorrespondences.push_back( make_pair(p , closestP));
			  }
 
		  }
	       }

          }
          
   
      }

     
      error = error/ sumOfWeights * DetectionToModelCorrespondences.size();
      float num_points =float(DetectionToModelCorrespondences.size()) +  DetectionToModelOutliers.size();
      DTMInlierPct = float(DetectionToModelCorrespondences.size()) / ( num_points +1 ) ;
      
      
      
      DTMError = error;
      
 }

 
 
 
  
  void DataAssociation::AssociateModelToDetection(FrameGrabber & CamFrm, FindNodes &NodeFinder, float& error){
  
     ModelToDetectionCorrespondences.clear();
     ModelToDetectionOutliers.clear();
     
     LinearGraph_Buffer  * m_LinearGraph_Buffer = NodeFinder.m_LinearGraph_Buffer;
     ModelLine_Buffer  * m_ModelLine_Buffer= CamFrm.forw_Proj.m_ModelLine_Buffer;
     
      if( m_LinearGraph_Buffer->size()==0 ||  m_ModelLine_Buffer->size() ==0){return;}
    
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
			

		        if((*it_2).sumOfLengthUndistorted  < MinLength_ref){continue;}
			

			vector<Line>  & lines = (*it_2).UndistortedLines;
			
			
			for(unsigned int  l = 0; l<lines.size(); l++) {
			      
// 			      if( m_Math::AngDif(point_ang,  lines[l].ang)>MaxAngDiff){continue;}
			    
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
		    
// 		    error+= pow(m_Math::TwoPointDistance(p, closestP), 2 ) * weights[n];
		    ModelToDetectionCorrespondences.push_back( make_pair(UndistortedPoints[pi] , closestP));  
	      }

	   }
	  
	  
      }
	          
      
      float num_points =float(ModelToDetectionCorrespondences.size()) +  ModelToDetectionOutliers.size();
      MTDInlierPct = float(ModelToDetectionCorrespondences.size()) / ( num_points +1 ) ;
      
//       error = error/float(DetectionToModelCorrespondences.size() +1) ;
      
      MTDError = error;
}
 
 

