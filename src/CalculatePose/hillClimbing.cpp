#include <visual_tracking/CalculatePose/hillClimbing.h>

  HillClimbing::HillClimbing(){
    

       MovingStepInit();
       for(int i=0; i<6; ++i){
	 min_moving_step[i]= 0.01 * moving_step[i];
       }
        m_rng = allocate_rng();
	
	
       n=0;
       first_10_error_avg=0;
       conf_error_lost =0;
       m=0;
       lost_frame_num=0;
       
       
       
        headingOffset = params.hillclimbing.HeadingOffset->get();
	heading_sub_robotstate = nodeHandle.subscribe("/robotmodel/robot_heading", 1, &HillClimbing::handleHeadingData, this);
	SeqNum =0;
	hypothesisArray.header.frame_id = "world";
	hypothesisArray.header.seq = SeqNum++;
	currentError=0;
    
  };
 void HillClimbing::MovingStepInit(){
   
      float MovingStepCoefficient = params.hillclimbing.MovingStepCoefficient->get();
       
       moving_step[0] = 1.0 * MovingStepCoefficient;
       moving_step[1] = 1.0 * MovingStepCoefficient;
       moving_step[2] = 0.1 * MovingStepCoefficient;
       moving_step[3] = 0.1 * MovingStepCoefficient;
       moving_step[4] = 0.1 * MovingStepCoefficient;
       moving_step[5] = 1.0 * MovingStepCoefficient;
    
  };
  
  //from Hafez
double HillClimbing::getHeading()
{       headingOffset = params.hillclimbing.HeadingOffset->get() ;
        return m_Math::CorrectAngleRadian360(headingData.heading + headingOffset );
}
void HillClimbing::getFieldInfo( FieldInfo &_fieldInfo ){
      fieldA= _fieldInfo.A;
      fieldB =_fieldInfo.B;
}
  
  
  HillClimbing::~HillClimbing(){ 
    
    
    gsl_rng_free(m_rng);
    
    
  }
    
    
    
gsl_rng* HillClimbing::allocate_rng()
{
	gsl_rng_env_setup();
	const gsl_rng_type* T = gsl_rng_default;
	gsl_rng* rng = gsl_rng_alloc(T);
	gsl_rng_set(rng, (unsigned long)time(NULL));

	return rng;
}




 void HillClimbing::getNeighbors(geometry_msgs::Pose pose, int idx, vector< geometry_msgs::Pose >& poseArray){
      poseArray.clear();
      poseArray.push_back(pose);
      geometry_msgs::Pose pose_tmp;
      double p[6];
      
      p[0]= pose.position.x;
      p[1]= pose.position.y;
      p[2]= pose.position.z;
      //calculate roll pitch yaw from quaternion
      tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w);
      q.normalized();
      tf::Matrix3x3 m(q);
      m.getRPY(p[3], p[4], p[5]);
      
      double dev = moving_step[idx];
      
//       if(idx<2){dev= position_eta;}
//       else if(idx ==2){dev= position_eta *0.1;}
//       else if(idx == 5){dev= orientation_eta;}
//       else{
//        { dev = orientation_eta *0.1;}}
      
      p[idx] += dev;
      pose_tmp.position.x =  p[0];
      pose_tmp.position.y =  p[1];
      pose_tmp.position.z =  p[2];
      pose_tmp.orientation = tf::createQuaternionMsgFromRollPitchYaw ( p[3], p[4], p[5]);  
      poseArray.push_back( pose_tmp );
      
      
      
      p[idx] -= dev;
      p[idx] -= dev;
      pose_tmp.position.x =  p[0];
      pose_tmp.position.y =  p[1];
      pose_tmp.position.z =  p[2];
      pose_tmp.orientation = tf::createQuaternionMsgFromRollPitchYaw ( p[3], p[4], p[5]);  
      poseArray.push_back( pose_tmp );
    
}




void HillClimbing::robotPoseUpdate(geometry_msgs::Pose &p,FrameGrabber & CamFrm, PointMatcher &pointMatcher,FindNodes &NodeFinder, double &error_){
  
    int Max_iter_Num = 6;
    MovingStepInit();
    error_ = 100000;

    int id[6];
    id[0]= 5; id[1] = 0; id[2] = 1; id[3]= 2; id[4] = 3; id[5] = 4; 
    
//     for(int idx =0; idx<6; idx++){
    for(int j =0; j<6; j++){
        int idx = id[j];
	
        int iter_num=0;
	bool update_to_neighbor = 1;
	
	while(update_to_neighbor==1 && iter_num<Max_iter_Num ){
	      iter_num++;
	      vector< geometry_msgs::Pose > poseArray;
	      
	      
	      getNeighbors(p, idx, poseArray);
	      
// 	      double error_tmp[3][2];
	      float error[poseArray.size() ];
	      
	      int min_i, min_error;
	      for(unsigned int i=0; i<poseArray.size(); ++i){
// 	              calculateError(poseArray[i],CamFrm,error_tmp[i],pointMatcher,NodeFinder );  
		     error[i] = calculateError(poseArray[i],CamFrm,NodeFinder);
		     if(  error[i] < min_error ){
		         min_error = error[i];
			 min_i =i;
		     }
     
	      }
	      
	      if(min_i == 0 && moving_step[idx] <= min_moving_step[idx]){update_to_neighbor =0; 
		                  error_ = error[0]; }
	      else if( min_i == 0 ){
		    moving_step[idx] = 0.1 * moving_step[idx] ; 
		    p= poseArray[0]; 
		    //cout<<"idx: "<< idx<<"moving_step "<< moving_step[idx]<<endl;
		    }
	      else{
		    if( min_i == 1 ){
		      p = poseArray[1];

		    }
		    else{ p = poseArray[2];}
	      }
	
	      
	      
	      
	      
//               double error1_max = std::max( std::max(error_tmp[0][0],  error_tmp[1][0]), error_tmp[2][0] );
// 	      double error12_max = std::max( std::max(error_tmp[0][1],  error_tmp[1][1]), error_tmp[2][1] );
// 	     
// 	      if(error1_max == 0){update_to_neighbor =0;}
// 	      else if(error12_max ==0) { update_to_neighbor =0; }
// 	      else{
// 	           double errora = error_tmp[0][0]/error1_max  + error_tmp[0][1]/error12_max;
// 		   double errorb = error_tmp[1][0]/error1_max  + error_tmp[1][1]/error12_max;
// 		   double errorc = error_tmp[2][0]/error1_max  + error_tmp[2][1]/error12_max;
// 		   
// 		   double min_bc= std::min(errorb,  errorc);
// 		   
// 		   if(errora <= min_bc && moving_step[idx] <= min_moving_step[idx]){update_to_neighbor =0; 
// 		                  error = /*error_tmp[0][0]  +*/ error_tmp[0][1]; }
// 		   else if( errora <= min_bc ){
// 			  moving_step[idx] = 0.1 * moving_step[idx] ; 
// 			  p= poseArray[0]; 
// 			  //cout<<"idx: "<< idx<<"moving_step "<< moving_step[idx]<<endl;
// 			  }
// 		   else{
// 			  if( errorb <=errorc ){
// 			    p = poseArray[1];
// 
// 			  }
// 			  else{ p = poseArray[2];}
// 		   }
// 		   
// 	      }
	}//end of while
//         cout<<"iter_num "<<iter_num<<endl;
      
    }//end of for

  
}


 void HillClimbing:: mainLoop(geometry_msgs::Pose &pose,FrameGrabber & CamFrm, PointMatcher &pointMatcher,FindNodes &NodeFinder){
     geometry_msgs::Pose p[3];
     p[0]= pose, p[1] =pose, p[2]=pose;

     double roll, pitch, yaw;
     tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w);
     q.normalized();
     tf::Matrix3x3 m(q);
     m.getRPY(roll, pitch, yaw);
     
     double deltaYaw = M_PI/12.0 ;
     p[1].orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll, pitch, yaw + deltaYaw);
     p[2].orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll, pitch, yaw - deltaYaw);
     
     double error[3], minError=100000;
     
     int j =1;
     if(lost_frame_num >=5){
         j =3; cout<<"lost----------------------------------"<<endl;;
     }
     
//      cout<< "Best Init    ";
     for(int i =0; i<j; ++i){
       robotPoseUpdate(p[i],CamFrm, pointMatcher,NodeFinder, error[i]);
       if(minError  > error[i]){ 
	  minError  = error[i];  pose = p[i]; 
// 	  cout<< i<<"   ";
        }
     }
//      cout<<endl;

     calculateCov(pose, CamFrm, pointMatcher,NodeFinder);

 }

  void HillClimbing::calculateCov(geometry_msgs::Pose &pose, FrameGrabber & CamFrm, PointMatcher &pointMatcher,FindNodes &NodeFinder){
    
          
      geometry_msgs::Pose pose_tmp;
      double p[6];
      p[0]= pose.position.x;
      p[1]= pose.position.y;
      p[2]= pose.position.z;
      //calculate roll pitch yaw from quaternion
      tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w);
      q.normalized();
      tf::Matrix3x3 m(q);
      m.getRPY(p[3], p[4], p[5]);
      
      
      double error1, error, error2;   
      double K=0,k1=0,k2=0;
      calculateErrorForCov(pose,CamFrm,error,K,pointMatcher,NodeFinder);  
      double delta = 0.001;
      
      if(K>0){
	  double e = sqrt(error)/K;
	  if(n<10){
	    first_10_error[n] = e;
	    first_10_error_avg += e;
	    n+=1;
	  }
	  else if(n==10){ first_10_error_avg = std::max(first_10_error_avg/10.0, 0.00001); n++;}
	  else { conf_error_lost =  (e > params.kalmanFilter.ErrorThreshod->get() * first_10_error_avg )? 1.0: e/(params.kalmanFilter.ErrorThreshod->get() * first_10_error_avg);
	    
// 	    cout<<"error "<<e<< ",  avg "<<first_10_error_avg<<", conf  "<<conf_error_lost<<endl;
	  }
      }
      
      if(conf_error_lost>0.9){lost_frame_num++;}
      else{ lost_frame_num=0;}
      
      
      for(int idx =0; idx<6; idx++){
	
	p[idx] += delta;
	pose_tmp.position.x =  p[0];
	pose_tmp.position.y =  p[1];
	pose_tmp.position.z =  p[2];
	pose_tmp.orientation = tf::createQuaternionMsgFromRollPitchYaw ( p[3], p[4], p[5]);  
	calculateErrorForCov(pose_tmp,CamFrm,error1,k1,pointMatcher,NodeFinder);  
	 
	p[idx] -= 2*delta;
	pose_tmp.position.x =  p[0];
	pose_tmp.position.y =  p[1];
	pose_tmp.position.z =  p[2];
	pose_tmp.orientation = tf::createQuaternionMsgFromRollPitchYaw ( p[3], p[4], p[5]);
	calculateErrorForCov(pose_tmp,CamFrm,error2,k2,pointMatcher,NodeFinder);  

	if(K<=3 || (error1 + error2 == 2* error)){ cov[idx]= 99999;}
	else{ cov[idx] = error/(K-3) *( delta*delta/(error1 + error2 - 2* error));}
	 
	p[idx] += delta;
	
      }
      
//       cout<<"cov::  ";
//       for(int idx =0; idx<6; idx++){
// 	cout << cov[idx]<<",  ";
//       }
//       
//       cout<<endl;
      
 
  }
 
 void HillClimbing::calculateErrorForCov(geometry_msgs::Pose &p,FrameGrabber & CamFrm, double &error, double& K, PointMatcher &pointMatcher,FindNodes &NodeFinder){
  
        CamFrm.forw_Proj.setRobotPose(p);
	CamFrm.forw_Proj.mainLoop();
	
	pointMatcher.matcher(NodeFinder.undistortedNodeSamplePoins, CamFrm.forw_Proj.ModelPointsInImg);
	K= pointMatcher.inlierNum;
	
	
	if(pointMatcher.inlierNum ==0 ){ error= std::numeric_limits<double>::max();}
	else{
              error =0;
	      for ( int i = 0; i<pointMatcher.inlierNum; ++i ){
		
		error += (pow(pointMatcher.correspondences[i*8 + 6] - pointMatcher.correspondences[i*8 + 0], 2) + 
				 pow(pointMatcher.correspondences[i*8 + 7] - pointMatcher.correspondences[i*8 + 1], 2));
	      }
	}

}
 

void HillClimbing::calculateError(geometry_msgs::Pose &p,FrameGrabber & CamFrm, double(&error)[2], PointMatcher &pointMatcher,FindNodes &NodeFinder){
  
        CamFrm.forw_Proj.setRobotPose(p);
	CamFrm.forw_Proj.mainLoop();
	
        
        pointMatcher.matcher(CamFrm.forw_Proj.ModelPointsInImg,NodeFinder.undistortedNodeSamplePoins);
	
	if(pointMatcher.inlierNum ==0 ){ error[0] = std::numeric_limits<double>::max();}
	else{
              error[0] =0;
	      for ( int i = 0; i<pointMatcher.inlierNum; ++i ){
// 		error[0] += sqrt(pow(pointMatcher.correspondences[i*8 + 6] - pointMatcher.correspondences[i*8 + 0], 2) + 
// 				 pow(pointMatcher.correspondences[i*8 + 7] - pointMatcher.correspondences[i*8 + 1], 2));
		
		error[0] += (pow(pointMatcher.correspondences[i*8 + 6] - pointMatcher.correspondences[i*8 + 0], 2) + 
				 pow(pointMatcher.correspondences[i*8 + 7] - pointMatcher.correspondences[i*8 + 1], 2));
	      }
	      error[0]/= double(pointMatcher.inlierNum );
	}
	
	
	
	
	pointMatcher.matcher(NodeFinder.undistortedNodeSamplePoins, CamFrm.forw_Proj.ModelPointsInImg);
	
	if(pointMatcher.inlierNum ==0 ){ error[1] = std::numeric_limits<double>::max();}
	else{
              error[1] =0;
	      for ( int i = 0; i<pointMatcher.inlierNum; ++i ){
// 		error[1] += sqrt(pow(pointMatcher.correspondences[i*8 + 6] - pointMatcher.correspondences[i*8 + 0], 2) + 
// 				 pow(pointMatcher.correspondences[i*8 + 7] - pointMatcher.correspondences[i*8 + 1], 2));
		
		error[1] += (pow(pointMatcher.correspondences[i*8 + 6] - pointMatcher.correspondences[i*8 + 0], 2) + 
				 pow(pointMatcher.correspondences[i*8 + 7] - pointMatcher.correspondences[i*8 + 1], 2));
	      }
	      error[1]/= double(pointMatcher.inlierNum );
	}

}








//new approach


float HillClimbing::calculateError(geometry_msgs::Pose &p,FrameGrabber & CamFrm,FindNodes &NodeFinder){
  

	
        CamFrm.forw_Proj.setRobotPose( p);
	CamFrm.forw_Proj.GetModelLineComps();
	float errorDTM;float errorMTD;
	float DTMInlierPct; float MTDInlierPct;;
        AssociateData.AssociateDetectionToModel( CamFrm, NodeFinder, errorDTM);
	AssociateData.AssociateModelToDetection( CamFrm, NodeFinder, errorMTD);
	
	return   errorDTM+ errorMTD;
	
	
// 	DTMInlierPct = AssociateData.getDTMInlierPct();
// 	MTDInlierPct = AssociateData.getMTDInlierPct();
// 
//         return   errorDTM/(DTMInlierPct+0.00001) + errorMTD /(MTDInlierPct + 0.000001);
	
	


}

float HillClimbing::calculateAvgError(geometry_msgs::Pose &p,FrameGrabber & CamFrm,FindNodes &NodeFinder){
  

        CamFrm.forw_Proj.setRobotPose( p);
	CamFrm.forw_Proj.GetModelLineComps();
	float errorDTM;float errorMTD;
	float DTMInlierNum; float MTDInlierNum;;
        AssociateData.AssociateDetectionToModel( CamFrm, NodeFinder, errorDTM);
	AssociateData.AssociateModelToDetection( CamFrm, NodeFinder, errorMTD);
	DTMInlierNum = AssociateData.getDTMInlierNum();
	MTDInlierNum = AssociateData.getMTDInlierNum();
	
	return   sqrt( errorDTM+ errorMTD)/float(DTMInlierNum +MTDInlierNum );


}

 float HillClimbing::calculateErrorForCov(geometry_msgs::Pose &p,FrameGrabber & CamFrm, FindNodes &NodeFinder, float &K ){// K ----num of correspondence

	CamFrm.forw_Proj.setRobotPose( p);
	CamFrm.forw_Proj.GetModelLineComps();
	float errorDTM;float errorMTD;
// 	float DTMInlierPct; float MTDInlierPct;;
        AssociateData.AssociateDetectionToModel( CamFrm, NodeFinder, errorDTM);
	AssociateData.AssociateModelToDetection( CamFrm, NodeFinder, errorMTD);

	int k1 = AssociateData.getDTMInlierNum();
	int k2 = AssociateData.getMTDInlierNum();
// 	K= k1+k2;
	 
	return  errorDTM   + errorMTD ;
// 	return  errorDTM *(k1+1)  + errorMTD*(k2+1) ;


}


void HillClimbing::calculateCov(geometry_msgs::Pose &pose, FrameGrabber & CamFrm, FindNodes &NodeFinder){
    
          
      geometry_msgs::Pose pose_tmp;
      double p[6];
      p[0]= pose.position.x;
      p[1]= pose.position.y;
      p[2]= pose.position.z;
      //calculate roll pitch yaw from quaternion
      tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w);
      q.normalized();
      tf::Matrix3x3 m(q);
      m.getRPY(p[3], p[4], p[5]);
      
      
      float error1, error, error2;   
      float K=0,k1=0,k2=0;
      error =  calculateErrorForCov(pose, CamFrm, NodeFinder, K);  
      double delta = 0.00001;
      
      if(K>0){
	  double e = sqrt(error)/K;
	  if(n<10){
	    first_10_error[n] = e;
	    first_10_error_avg += e;
	    n+=1;
	  }
	  else if(n==10){ first_10_error_avg = std::max(first_10_error_avg/10.0, 0.00001); n++;}
	  else { conf_error_lost =  (e > params.kalmanFilter.ErrorThreshod->get() * first_10_error_avg )? 1.0: e/(params.kalmanFilter.ErrorThreshod->get() * first_10_error_avg);
	    
// 	    cout<<"error "<<e<< ",  avg "<<first_10_error_avg<<", conf  "<<conf_error_lost<<endl;
	  }
      }
      
      if(conf_error_lost>0.9){lost_frame_num++;}
      else{ lost_frame_num=0;}
      
      
      for(int idx =0; idx<6; idx++){
	
	p[idx] += delta;
	pose_tmp.position.x =  p[0];
	pose_tmp.position.y =  p[1];
	pose_tmp.position.z =  p[2];
	pose_tmp.orientation = tf::createQuaternionMsgFromRollPitchYaw ( p[3], p[4], p[5]);  
// 	calculateErrorForCov(pose_tmp,CamFrm,error1,k1,pointMatcher,NodeFinder);  
        error1 =  calculateErrorForCov(pose_tmp, CamFrm, NodeFinder, k1);  
	
	p[idx] -= 2*delta;
	pose_tmp.position.x =  p[0];
	pose_tmp.position.y =  p[1];
	pose_tmp.position.z =  p[2];
	pose_tmp.orientation = tf::createQuaternionMsgFromRollPitchYaw ( p[3], p[4], p[5]);
// 	calculateErrorForCov(pose_tmp,CamFrm,error2,k2,pointMatcher,NodeFinder);  
	
	error2 =  calculateErrorForCov(pose_tmp, CamFrm, NodeFinder, k2);  

	if(K<=3 || (error1 + error2 == 2* error)){ cov[idx]= 9999;}
	else{ cov[idx] =  error/(K-3) *( delta*delta/(error1 + error2 - 2* error));}
	 
	p[idx] += delta;
	
      }
      
//       cout<<"cov::  ";
//       for(int idx =0; idx<6; idx++){
// 	cout << cov[idx]<<",  ";
//       }
//       
//       cout<<endl;
      
 
  }


  
void HillClimbing::robotPoseUpdate(geometry_msgs::Pose &p,FrameGrabber & CamFrm,FindNodes &NodeFinder, float &_error){
  
    headingOffset = params.hillclimbing.HeadingOffset->get();
    MovingStepInit();
    
    int Max_iter_Num = params.hillclimbing.MaxIterationNum->get();;
    _error = 0;
    
        int iter_num=0;
	int noUpdate =0;
// 	cout<< "---------------------------------------"<<endl;
	while(noUpdate < 4&& iter_num<Max_iter_Num ){
	      iter_num++;
	      vector< geometry_msgs::Pose > poseArray;
// 	      cout<< "iter_num    "<< iter_num <<endl;
	      
	      int update_times=0;
	      for(int j =0; j<6; j++){
		    int idx = j;
			      
		    getNeighbors(p, idx, poseArray);
		    
		    float error[poseArray.size() ];
		    
		    int min_i, min_error = std::numeric_limits<float>::max();
		    for(unsigned int i=0; i<poseArray.size(); ++i){
			  error[i] = calculateError(poseArray[i],CamFrm,NodeFinder);
			  if(  error[i] < min_error ){
			      min_error = error[i];
			      min_i =i;
			  }
	  
		    }
		    _error = min_error;
		    
		    // stay 
		    if(min_i == 0) {
			  moving_step[idx] = std::max(  0.5 * moving_step[idx] , min_moving_step[idx]);
  // 			  continue;
		    }
		    else {//move to next  
			  p = poseArray[min_i];
			  update_times++;  
		    }
		    
		    double roll, pitch, yaw;
		    tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z,p.orientation.w);
  // 	    	    q.normalized();
		    tf::Matrix3x3 m(q);
		    m.getRPY(roll, pitch, yaw);
		    if(m_Math::RadianAngleDiff(yaw, getHeading()) > M_PI/2.0){

		       p.position.x = -p.position.x;
		       p.position.y = -p.position.y;
		       yaw= m_Math::CorrectAngleRadian360(yaw - M_PI);
		       p.orientation = tf::createQuaternionMsgFromRollPitchYaw( roll,  pitch,  yaw);
// 		       cout<<"Go to Symetric position "<<endl;
// 		       cout << idx<<"  "<< p.position.x<<", " << p.position.y<<", " << p.position.z<<", " << roll<<", " << pitch <<", " << yaw  <<endl;
		      
		    }

// 		    cout << idx<<"  "<< p.position.x<<", " << p.position.y<<", " << p.position.z<<", " << roll<<", " << pitch <<", " << yaw  <<  "  error: "<< min_error<<endl;

	      }
		  
	     if(update_times ==0){noUpdate++;}
	     else{noUpdate=0; }

	}
  
}

float HillClimbing::getConfidence(geometry_msgs::Pose &p,FrameGrabber & CamFrm,FindNodes &NodeFinder){
  
      float ErrorThreshold1 = std::max(double(params.hillclimbing.ErrorThreshold1->get()), 1.0);
      float ErrorThreshold2 = std::max(double(params.hillclimbing.ErrorThreshold2->get()), 1.0);
      
      float error = calculateError(p,CamFrm, NodeFinder);
      
      
      float errorDTM =  AssociateData.getDTMError();
      float errorMTD =  AssociateData.getMTDError();
      
      int DTMInlierNum = AssociateData.getDTMInlierNum();
      int MTDInlierNum = AssociateData.getMTDInlierNum();
      
      int DTMOutlierNum = AssociateData.getDTMOutlierNum();
      int MTDOutlierNum = AssociateData.getMTDOutlierNum();
      
      float DTMInlierPct = AssociateData.getDTMInlierPct();
      float MTDInlierPct = AssociateData.getMTDInlierPct();
      
//       error = sqrt (error) / float(DTMInlierNum + MTDInlierNum);
      
      error = sqrt(  error/ float(DTMInlierNum + MTDInlierNum));
      
      
//       cout<<"error  "<<error<<endl;
      if(DTMInlierPct == 0 || MTDInlierPct == 0){return 0.0; cout<<"bad1"<<endl;}
      if(DTMInlierNum == 0 || MTDInlierNum == 0){return 0.0; cout<<"bad2"<<endl;}
//       cout<<DTMInlierPct<<"   "<< error;
      error = error/DTMInlierPct;
//       cout<<"  "<< error<<endl;
      if(error< ErrorThreshold1){ return 1.0;}
      else if(error > ErrorThreshold2) {return 0.0;}
      else{
	return 1.0- (error - ErrorThreshold1)/(ErrorThreshold2 -ErrorThreshold1) ;
	
      }
  
}


geometry_msgs::Pose HillClimbing::SamplingPoseAround(geometry_msgs::Pose center, float conf){
      geometry_msgs::Pose p;
      float  range_x = (1-conf) * fieldA/2.0 * 0.5;
      float range_y = (1-conf) * fieldB/2.0 * 0.55;
      float range_yaw = (1-conf) * M_PI *0.5;
      
      float x_max= std::min(center.position.x + range_x,  fieldA/2.0 + 0.5);
      float x_min= std::max(center.position.x - range_x, -fieldA/2.0 - 0.5);
      float y_max= std::min(center.position.y + range_y,  fieldB/2.0 + 0.5);
      float y_min= std::max(center.position.y - range_y, -fieldB/2.0 - 0.5);
      p.position.x = random(x_min, x_max);
      p.position.y = random(y_min, y_max);
      p.position.z = random(0.5, 0.58);
      
       double roll_ref, pitch_ref, yaw_ref;
      tf::Quaternion q(center.orientation.x,center.orientation.y, center.orientation.z,center.orientation.w);
// 	    	    q.normalized();
      tf::Matrix3x3 m(q);
      m.getRPY(roll_ref, pitch_ref, yaw_ref);
      
      
      
      
      double roll = 0 ,pitch =0; double yaw;
//       double yaw_min = 0, yaw_max = 2*M_PI;
//       if(p.position.x > fieldA/2 - 0.5  ){
// 	yaw_min = 0.5*M_PI; 
// 	yaw_max = 1.5*M_PI;
//       }
//       else if(p.position.y > fieldB/2 - 0.5  ){
// 	yaw_min =  1.0*M_PI; 
// 	yaw_max =   2*M_PI;
//       }
// 
//       else if(p.position.x < -fieldA/2 + 0.5 ){
// 	yaw_min = -0.5*M_PI; 
// 	yaw_max =  0.5*M_PI;
//       }
//       else if( p.position.y < -fieldB/2 + 0.5){
// 	yaw_min = 0.0*M_PI; 
// 	yaw_max =  1.0*M_PI;
//       }
//       else{
// 	yaw_min = 0.0*M_PI; 
// 	yaw_max = 2.0*M_PI;
// 	}


      
      
     
      
      float heading =  getHeading();
//       float yaw_max= yaw_ref + range_yaw;
//       float yaw_min= yaw_ref - range_yaw;
      float yaw_max= heading + range_yaw;
      float yaw_min= heading - range_yaw;
      
      
      yaw =  random(yaw_min,yaw_max);
      yaw = m_Math::CorrectAngleRadian360(yaw);

      
      
//       if(m_Math::RadianAngleDiff(yaw, getHeading()) > M_PI/2.0){ 
// 	  p.position.x = -p.position.x;
//           p.position.y = -p.position.y;
// 	
// 	  yaw = m_Math::CorrectAngleRadian360(yaw-M_PI);
//       }

      p.orientation = tf::createQuaternionMsgFromRollPitchYaw( roll,  pitch,  yaw);
      

      return p;
}


void HillClimbing::hypothesisEvalueate(float conf, FrameGrabber & CamFrm,FindNodes &NodeFinder){
      hypothesisArray.header.seq = SeqNum++;
      hypothesisArray.header.stamp =ros::Time::now();
      hypothesisArray.poses.clear();
      hypothesisWeights.clear();
      
      if(conf ==1){return;}
      
      int MaxHypothesisNum = params.hillclimbing.MaxHypothesisNum->get();
      
     
      	
      int HypothesisNum = (1-conf) * MaxHypothesisNum;
      if(HypothesisNum==0){return;}
     
      
      Vector<pair<geometry_msgs::Pose, float> > hythoposes_tmp;
      hythoposes_tmp.push_back( make_pair(currentPose, 
				getConfidence(currentPose,  CamFrm, NodeFinder )));

      for(int i=1; i<HypothesisNum; ++i ){
	 geometry_msgs::Pose p = SamplingPoseAround(currentPose, conf);
	 hythoposes_tmp.push_back( make_pair(p, getConfidence(p,  CamFrm, NodeFinder )));
      }
      
        std::sort(hythoposes_tmp.begin(), hythoposes_tmp.end(),  
		  boost::bind(&pair<geometry_msgs::Pose, float>::second, _1) > 
		  boost::bind(&pair<geometry_msgs::Pose, float>::second, _2));
      
   
      for(int i=0; i<hythoposes_tmp.size(); ++i ){
	  hypothesisArray.poses.push_back(hythoposes_tmp[i].first);
	  hypothesisWeights.push_back(hythoposes_tmp[i].second);
      }
  
}





  
 void HillClimbing:: mainLoop(geometry_msgs::Pose &pose,FrameGrabber & CamFrm ,FindNodes &NodeFinder){
   
        headingOffset = params.hillclimbing.HeadingOffset->get();
//      geometry_msgs::Pose p[3];
//      p[0]= pose, p[1] =pose, p[2]=pose;
// 
//      double roll, pitch, yaw;
//      tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w);
// //      q.normalized();
//      tf::Matrix3x3 m(q);
//      m.getRPY(roll, pitch, yaw);
//      
//      double deltaYaw = M_PI/12.0 ;
//      p[1].orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll, pitch, yaw + deltaYaw);
//      p[2].orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll, pitch, yaw - deltaYaw);
//      
//      float error[3];
//      
//      int j =1;
//      if(lost_frame_num >=5){
//          j =3; cout<<"lost----------------------------------"<<endl;;
//      }
     
//      cout<< "Best Init    ";
//      for(int i =0; i<j; ++i){
	getFieldInfo(CamFrm.forw_Proj.fieldInfo);
	
	
       robotPoseUpdate(pose ,CamFrm, NodeFinder, currentError);
       currentPose = pose;
      
       float conf=  getConfidence(pose,  CamFrm, NodeFinder );
//        cout<<"confidence:  "<<conf<<endl;
       hypothesisEvalueate( conf,   CamFrm, NodeFinder);
       
       if(hypothesisArray.poses.size()>0 && hypothesisWeights[0]> conf){
	    pose = hypothesisArray.poses[0];
// 	    cout<<"conf2  "<<hypothesisWeights[0]<<endl;
      }
//        cout<<endl;
        calculateCov(pose, CamFrm,NodeFinder);
	
	

 }
 
 
 
 
 
 // ************************************** //
// ********* Private Methods *********** // 	
// ************************************** //

double HillClimbing::random(double start,double end)
{
    return start + (end - start)*gsl_rng_uniform(m_rng);
//     return start + (end - start) * rand() / (RAND_MAX + 1.0);
}
float HillClimbing::random(float start,float end)
{
    return start + (end - start)*gsl_rng_uniform(m_rng);
//     return start + (end - start) * rand() / (RAND_MAX + 1.0);
}
 
 
 


