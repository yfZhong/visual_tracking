// by Yongfeng
// #include "lineMatcher.h"
#include <visual_tracking/LineMatcher/lineMatcher.h>

LineMatcher::LineMatcher(){
  
  if(params.field.BonnField->get()==true){
    
    fieldInfo.A = 5.45;
    fieldInfo.B = 4.10;
    fieldInfo.E = 0.60;
    fieldInfo.F = 3.40;
    fieldInfo.G = 1.30;
    fieldInfo.H = 1.20;
    fieldInfo.D = 2.6;
    
    
  }
  else{
    fieldInfo.A = 9;
    fieldInfo.B = 6;
    fieldInfo.E = 1;
    fieldInfo.F = 5;
    fieldInfo.G = 2.1;
    fieldInfo.H = 1.50;
    fieldInfo.D = 2.6;
  }
  
  generateFieldLines();
  NumProjectedToDifferentLines=0;
  sumofLineLength=0;

}


 void LineMatcher::generateFieldLines(){
        Field_Lines.clear();
      int id = 0;
      //2 long boarder lines
      Vec3f s_w(-fieldInfo.A/2, -fieldInfo.B/2 , 0), e_w(fieldInfo.A/2, -fieldInfo.B/2 ,0);
      Field_Lines.push_back(Line_w(id ++, s_w, e_w));
      
      
      s_w = Vec3f(-fieldInfo.A/2, fieldInfo.B/2 , 0), e_w = Vec3f(fieldInfo.A/2, fieldInfo.B/2 ,0);
      Field_Lines.push_back(Line_w(id ++, s_w, e_w));
      
      //4 short lines
      s_w = Vec3f(-fieldInfo.A/2, -fieldInfo.F/2 , 0), e_w = Vec3f(-fieldInfo.A/2 + fieldInfo.E, -fieldInfo.F/2 ,0);
      Field_Lines.push_back(Line_w(id ++, s_w, e_w));
      
   
      s_w = Vec3f(-fieldInfo.A/2, fieldInfo.F/2 , 0), e_w = Vec3f(-fieldInfo.A/2 + fieldInfo.E, fieldInfo.F/2 ,0);
      Field_Lines.push_back(Line_w(id ++, s_w, e_w));
      
   
      s_w = Vec3f(fieldInfo.A/2- fieldInfo.E, -fieldInfo.F/2 , 0), e_w = Vec3f(fieldInfo.A/2, -fieldInfo.F/2 ,0);
      Field_Lines.push_back(Line_w(id ++, s_w, e_w));
      
 
      s_w = Vec3f(fieldInfo.A/2- fieldInfo.E, fieldInfo.F/2 , 0), e_w = Vec3f(fieldInfo.A/2, fieldInfo.F/2 ,0);
      Field_Lines.push_back(Line_w(id ++, s_w, e_w));
      
      
      
      //short boarder lines
      s_w = Vec3f(-fieldInfo.A/2, -fieldInfo.B/2 , 0), e_w = Vec3f(-fieldInfo.A/2, fieldInfo.B/2 ,0);
      Field_Lines.push_back(Line_w(id ++, s_w, e_w));
      
   
      s_w = Vec3f(0, -fieldInfo.B/2 , 0), e_w = Vec3f(0, fieldInfo.B/2 ,0);
      Field_Lines.push_back(Line_w(id ++, s_w, e_w));
  
      s_w = Vec3f(fieldInfo.A/2, -fieldInfo.B/2 , 0), e_w = Vec3f(fieldInfo.A/2, fieldInfo.B/2 ,0);
      Field_Lines.push_back(Line_w(id ++, s_w, e_w));
      
      
      
      //2 short vertical lines
      s_w = Vec3f(-fieldInfo.A/2 + fieldInfo.E, -fieldInfo.F/2 , 0), e_w = Vec3f(-fieldInfo.A/2 + fieldInfo.E, fieldInfo.F/2 ,0);
      Field_Lines.push_back(Line_w(id ++, s_w, e_w));

      
      s_w = Vec3f(fieldInfo.A/2 - fieldInfo.E, -fieldInfo.F/2 , 0), e_w = Vec3f(fieldInfo.A/2 - fieldInfo.E, fieldInfo.F/2 ,0);
      Field_Lines.push_back(Line_w(id ++, s_w, e_w));
      
      
      id=0;
      //circle lines for vis
      for(int i=0 ; i <100 -1; i++){// center circle
	  double theta = i * 2* M_PI /100.; double theta2 = (i+1) * 2* M_PI /100.;
	  s_w = Vec3f(  fieldInfo.H/2*cos(theta),  fieldInfo.H/2 *sin(theta) , 0), e_w =  Vec3f(  fieldInfo.H/2*cos(theta2),  fieldInfo.H/2 *sin(theta2) , 0);
	  Field_Circle_Lines_Vis.push_back(Line_w(id ++, s_w, e_w));
      } 
      double theta3 = 99. * 2* M_PI /100;
      s_w = Vec3f(  fieldInfo.H/2*cos(theta3),  fieldInfo.H/2 *sin(theta3) , 0), e_w =  Vec3f(  fieldInfo.H/2*cos(0),  fieldInfo.H/2 *sin(0) , 0);
      Field_Circle_Lines_Vis.push_back(Line_w(id ++, s_w, e_w));
      
}


void LineMatcher::setMeasurement(  std::vector< Line > LinesOnImg_ ){
   LinesOnImg = LinesOnImg_;
   
  
//     m_LineBuffer = m_LineBuffer_;
    
//       	backw_Proj.setCurTime(CameraFrame.header.stamp);
// 	backw_Proj.setRobotPose(robotPose);
//     backw_Proj.setTransform();
    
}

void LineMatcher::ProjectLinesToWorld(std::vector< Line > LinesOnImg, std::vector< Line_w >& m_LineBuffer_Before_Merge ){ 
   m_LineBuffer_Before_Merge.clear();
//  backw_Proj.onInit(CameraFrame.header.stamp , robotPose);
  int id = 0;
  for(unsigned int i = 0;i < LinesOnImg.size(); i++){
  
          cv::Point p1(LinesOnImg[i].s[0],LinesOnImg[i].s[1]),p2(LinesOnImg[i].e[0],LinesOnImg[i].e[1]);
	  cv::Point3f  s_w0, e_w0;
	  backw_Proj.projectImgPoint2WorldCord(p1, s_w0);
	  backw_Proj.projectImgPoint2WorldCord(p2, e_w0);
	  
	  Vec3f  s_w(s_w0.x, s_w0.y, s_w0.z);
	  Vec3f  e_w(e_w0.x, e_w0.y, e_w0.z);
	  
	  Line_w line(id++ , s_w, e_w) ;
	  m_LineBuffer_Before_Merge.push_back(line);
  }
    
}


Line_w LineMatcher::MergeTwoLines(Line_w line1, Line_w line2, int id)
{
  
  
      float ang_w = 0;
      Vec2f  line1MidP((line1.s_w[0] + line1.e_w[0])/2.0, (line1.s_w[1] + line1.e_w[1])/2.0);
      Vec2f  line2MidP((line2.s_w[0] + line2.e_w[0])/2.0, (line2.s_w[1] + line2.e_w[1])/2.0);
      float min_value = 0.000001;
      float r = line1.len_w/(std::max(line1.len_w + line2.len_w, min_value));
  
      Vec3f newMidP(r*line1MidP[0] + (1-r) * line2MidP[0] , r*line1MidP[1] + (1-r) * line2MidP[1], 0 );
      
       //(-M_PI/2, M_PI/2)
      if(fabs(line1.ang_w -line2.ang_w)>M_PI/2.0 ){  
	if(line1.ang_w<0 ){ line1.ang_w = line1.ang_w +M_PI;}
	else{ line2.ang_w = line2.ang_w + M_PI;}
      }
      ang_w = r*line1.ang_w + (1-r) *line2.ang_w; 
      if(ang_w >=M_PI/2.0){ang_w = ang_w - M_PI; }
      
      
      float k_w = m_Math::getKValueFromAngle(ang_w);
      vector<Vec3f> pProject; 
      Vec3f pp;
      m_Math::GetProjectivePoint( line1.s_w, k_w, newMidP,  pp); pProject.push_back(pp);
      m_Math::GetProjectivePoint( line1.e_w, k_w, newMidP,  pp); pProject.push_back(pp);
      m_Math::GetProjectivePoint( line2.s_w, k_w, newMidP,  pp); pProject.push_back(pp);
      m_Math::GetProjectivePoint( line2.e_w, k_w, newMidP,  pp); pProject.push_back(pp);
      Vec3f start_w = newMidP, end_w = newMidP;
      
      for(unsigned int i = 0; i< pProject.size(); ++i){
	  
	    if(ang_w == M_PI/2.0 ){//vertical line
		if(pProject[i][1]< start_w[1]){ start_w[0] =pProject[i][0];start_w[1] = pProject[i][1]; }
		if(pProject[i][1]> end_w[1]){ end_w[0] =pProject[i][0];end_w[1] = pProject[i][1]; }

	    }
	    else
	    {  
	      if(pProject[i][0]< start_w[0]){ start_w[0] = pProject[i][0]; start_w[1] = pProject[i][1]; }
	      if(pProject[i][0]> end_w[0]){ end_w[0] = pProject[i][0]; end_w[1] = pProject[i][1]; }
	      
	    }
        }  
        float length_w = m_Math::TwoPointDistance(start_w, end_w);
      
      Line_w Merged_line(id, length_w , start_w, end_w, k_w,ang_w,1.0) ;
  
      return Merged_line;
  
}
void LineMatcher::MergeLines(std::vector< Line_w > m_LineBuffer_Before_Merge, std::vector< Line_w >& m_LineBuffer_After_Merge){
      if( m_LineBuffer_Before_Merge.size()<1){return;}
//    cv::Mat vis_houghLines(siY,siX,CV_8UC3,cv::Scalar(150, 150, 150));
      m_LineBuffer_After_Merge.clear();

      float AngleToMerge = params.line.AngleToMerge->get();
      float MinLineSegDistance = params.line.MinLineSegDistance->get();
      float MinProjectedDistance =params.line.MinProjectedDistance->get();
      
      
      
//       cout<<"Merge Line: ";
      std::vector< Line_w >  m_LineBuffer_Before_Merge_tmp = m_LineBuffer_Before_Merge ;
      std::vector< Line_w >  m_LineBuffer_After_Merge_tmp;
//       int loopNum=0;
      while (true){
//           loopNum++;
	  int id=0;
	  for(unsigned int i=0; i < m_LineBuffer_Before_Merge_tmp.size(); ++i ){
	      if(m_LineBuffer_Before_Merge_tmp[i].id !=-1){
		    Line_w a = m_LineBuffer_Before_Merge_tmp[i];
		    for(unsigned int j= i + 1; j < m_LineBuffer_Before_Merge_tmp.size(); ++j ){
			if(m_LineBuffer_Before_Merge_tmp[j].id != -1 ){
			  Line_w b = m_LineBuffer_Before_Merge_tmp[j];
			  Vec3f Midpoint_a((a.s_w[0]+a.e_w[0])/2.0, (a.s_w[1]+a.e_w[1])/2.0, 0);
			  Vec3f Midpoint_b((b.s_w[0]+b.e_w[0])/2.0, (b.s_w[1]+b.e_w[1])/2.0, 0);
			  if(m_Math::AngDif( a.ang_w, b.ang_w) < AngleToMerge
			    &&  m_Math::getShortestDistance(a, b) <MinLineSegDistance
			    &&  m_Math::GetProjectiveDistance(Midpoint_a, b) < MinProjectedDistance
			     && m_Math::GetProjectiveDistance(Midpoint_b, a) < MinProjectedDistance){
			    
			      a = MergeTwoLines(a, b, id++);
			      m_LineBuffer_Before_Merge_tmp[j].id = -1;
			    
			  
			  }
			}
		      
		    }
		    m_LineBuffer_After_Merge_tmp.push_back(a);

	      }
	  
	  }
	  if(m_LineBuffer_After_Merge_tmp.size() ==m_LineBuffer_Before_Merge_tmp.size() ){break;}
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


 void LineMatcher::matchLines(){
    AssociationLines.clear();
    int ass_id = 0;
    NumProjectedToDifferentLines = 0;
    sumofLineLength = 0.0;
    vector<int> ProjectedToFieldLineId;
    
    for(unsigned int i =0; i<m_LineBuffer_After_Merge.size(); ++i){
      Vec3f s_w;
      Vec3f e_w;
      
       Vec3f middleP((m_LineBuffer_After_Merge[i].s_w[0] + m_LineBuffer_After_Merge[i].e_w[0])/2.0, 
		     (m_LineBuffer_After_Merge[i].s_w[1] + m_LineBuffer_After_Merge[i].e_w[1])/2.0);
      
      float dist= std::numeric_limits<float>::max(); 
      int id = Field_Lines.size();
     
      
      for(unsigned int j =0; j< Field_Lines.size(); ++j){
	//project to lines with similar angle
// 	if(  (fabs(m_LineBuffer_After_Merge[i].ang_w) < 0.25*M_PI && fabs(Field_Lines[j].ang_w) < 0.25*M_PI)
// 	  || (fabs(m_LineBuffer_After_Merge[i].ang_w) > 0.25*M_PI && fabs(Field_Lines[j].ang_w) > 0.25*M_PI)
// 	  || (fabs(m_LineBuffer_After_Merge[i].ang_w) == 0.25*M_PI)){
	if(1){
	    // at least one of the end point of the line is projected incide the target line segment
// 	    if(isProjectedInsideLineSeg( m_LineBuffer_After_Merge[i].s_w,Field_Lines[j]) || 
// 	       isProjectedInsideLineSeg( m_LineBuffer_After_Merge[i].e_w,Field_Lines[j]) ){
	    if(1){
// 	        float distS=GetProjectiveDistance(m_LineBuffer_After_Merge[i].s_w, Field_Lines[j]);
// 		float distM=GetProjectiveDistance(middleP, Field_Lines[j]);
// 		float distE=GetProjectiveDistance(m_LineBuffer_After_Merge[i].e_w, Field_Lines[j]);
	      
		float distS= m_Math::PointToLineSegDistance(m_LineBuffer_After_Merge[i].s_w, Field_Lines[j]);
		float distM= m_Math::PointToLineSegDistance(middleP, Field_Lines[j]);
		float distE= m_Math::PointToLineSegDistance(m_LineBuffer_After_Merge[i].e_w, Field_Lines[j]);
		float distP = (distS + distM + distE)/3.0;
		Line_w  ProjectedLineIncide;
// 		if( getLineProjectToLineSeg(m_LineBuffer_After_Merge[i], Field_Lines[j],ProjectedLineIncide)
// 		  && ProjectedLineIncide.len_w!=0  && m_LineBuffer_After_Merge[i].len_w!=0){
		  
// 		  distP = distP/( ProjectedLineIncide.len_w / m_LineBuffer_After_Merge[i].len_w);

		  if(distP < dist){ 
		      dist = distP; 
		      id = j;
// 		       cout<<i<< " to "<< j<< "  dist: "<<dist<<endl;
		    }
		  
// 		}
		

	    }
	  
	}
      }
      
//       //The line is not able to project to any line
//       if(id == int(Field_Lines.size())) {
// 	      for(unsigned int j =0; j< Field_Lines.size(); ++j){
// // 		float distS=GetProjectiveDistance(m_LineBuffer_After_Merge[i].s_w, Field_Lines[j]);
// // 		float distM=GetProjectiveDistance(middleP, Field_Lines[j]);
// // 		float distE=GetProjectiveDistance(m_LineBuffer_After_Merge[i].e_w, Field_Lines[j]);
// 		
// 		float distS=PointToLineSegDistance(m_LineBuffer_After_Merge[i].s_w, Field_Lines[j]);
// 		float distM=PointToLineSegDistance(middleP, Field_Lines[j]);
// 		float distE=PointToLineSegDistance(m_LineBuffer_After_Merge[i].e_w, Field_Lines[j]);
// 		float distP = (distS + distM  + distE)/3.0;
// 		
// 		if(distP < dist){ 
// 		  dist = distP; 
// 		  id = j;
// 		}
//               }
//       }
      
      //TODO::maybe we need to sample more points on each line, then calculate the average ProjectiveDistanceOnCircle
      // check the distance when projecting to the center circle
      if(m_LineBuffer_After_Merge[i].len_w < fieldInfo.H/2){
	  // project to circle
	  float distS_C = m_Math::GetProjectiveDistanceOnCircle(m_LineBuffer_After_Merge[i].s_w, float(fieldInfo.H/2.0));
	  float distM_C = m_Math::GetProjectiveDistanceOnCircle(middleP, float(fieldInfo.H/2.0));
	  float distE_C = m_Math::GetProjectiveDistanceOnCircle(m_LineBuffer_After_Merge[i].e_w, float(fieldInfo.H/2.0));
	  float distP_C = (distS_C + distM_C + distE_C)/3.0;
	  if(distP_C < dist){ 
	    dist = distP_C; 
	    id = -1;;
	  }
    //       cout<<i<< " to circle  dist: "<<distP_C<<endl;
      }
      Vec3f p1, p2;
      
      if(id == -1) {
// 	cout<<i <<"  angle:   "<<m_LineBuffer_After_Merge[i].ang_w<<"   project to circle. "<<endl;
	Vec3f  pProjec;
	m_Math::GetProjectivePointOnCircle( middleP, float(fieldInfo.H/2.0) , pProjec);
	s_w = middleP; e_w = pProjec;
	AssociationLines.push_back(Line_w(ass_id++, s_w, e_w));
	
	s_w = m_LineBuffer_After_Merge[i].s_w;
	m_Math::GetProjectivePointOnCircle( m_LineBuffer_After_Merge[i].s_w, float(fieldInfo.H/2.0) , e_w);
	AssociationLines.push_back(Line_w( ass_id++, s_w, e_w));
	p1 = e_w;
	
	s_w = m_LineBuffer_After_Merge[i].e_w;
	m_Math::GetProjectivePointOnCircle( m_LineBuffer_After_Merge[i].e_w, float(fieldInfo.H/2.0) , e_w);
	AssociationLines.push_back(Line_w( ass_id++, s_w, e_w));
	p2 = e_w;
	
	
	sumofLineLength += m_Math::TwoPointDistance(p1,p2 );
      }
      else if(id <int( Field_Lines.size()) ) {
// 	cout<<i <<"  angle:   "<<m_LineBuffer_After_Merge[i].ang_w<<"   project to line  "<< id <<"  angle:   "<<Field_Lines[id].ang_w<<endl;
	
	    s_w = middleP;
// 	    GetProjectivePoint( s_w, Field_Lines[id], e_w);
	    m_Math::getClosestPointOnLineSeg(s_w, Field_Lines[id], e_w);
	    AssociationLines.push_back(Line_w(ass_id++, s_w, e_w));
	
// 	    GetProjectivePoint( m_LineBuffer_After_Merge[i].s_w, Field_Lines[id], s_w);
// 	    GetProjectivePoint( m_LineBuffer_After_Merge[i].e_w, Field_Lines[id], e_w);
	    m_Math::getClosestPointOnLineSeg( m_LineBuffer_After_Merge[i].s_w, Field_Lines[id], s_w);
	    m_Math::getClosestPointOnLineSeg( m_LineBuffer_After_Merge[i].e_w, Field_Lines[id], e_w);
	    AssociationLines.push_back(Line_w( ass_id++, s_w, e_w));
	    p1 = s_w; p2 = e_w;
	    sumofLineLength += m_Math::TwoPointDistance(p1, p2 );

	    
      }
      
       
       ProjectedToFieldLineId.push_back(id);
       m_LineBuffer_After_Merge[i].conf = dist;

  }


       std::sort (ProjectedToFieldLineId.begin(), ProjectedToFieldLineId.end()); 
       vector<int>::iterator iter = unique(ProjectedToFieldLineId.begin(),ProjectedToFieldLineId.end());
       ProjectedToFieldLineId.erase(iter,ProjectedToFieldLineId.end());
       NumProjectedToDifferentLines = int(ProjectedToFieldLineId.size());
}

void LineMatcher::run(std::vector< Line > LinesOnImg_){
    setMeasurement( LinesOnImg_ );
    
    ProjectLinesToWorld(LinesOnImg, m_LineBuffer_Before_Merge );
    
    MergeLines( m_LineBuffer_Before_Merge, m_LineBuffer_After_Merge );
    
    matchLines();
}



  
  
