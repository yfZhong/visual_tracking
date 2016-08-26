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
	count=0;
	
	H = ORG_IMAGE_HEIGHT;
        W = ORG_IMAGE_WIDTH;
	siX = UNDISTOTED_IMAGE_WIDTH; 
        siY = UNDISTOTED_IMAGE_HEIGHT;
	
	 for(int idx =0; idx<6; idx++){
	   cov[idx] = 1000;
	}
	confidence =0;
	
	robot_rot_height = 0.578;
	
    
  }
  
  
  
 void HillClimbing::MovingStepInit(){
   
      float MovingStepCoefficient = params.hillclimbing.MovingStepCoefficient->get();
       
       moving_step[0] = 1.0 * MovingStepCoefficient;
       moving_step[1] = 1.0 * MovingStepCoefficient;
       moving_step[2] = 0.1 * MovingStepCoefficient;
       moving_step[3] = 0.1 * MovingStepCoefficient;
       moving_step[4] = 0.1 * MovingStepCoefficient;
       moving_step[5] = 1.0 * MovingStepCoefficient;
    
  };
  

  
  
  HillClimbing::~HillClimbing(){ 
    
    gsl_rng_free(m_rng);
    
    
  }
    
    
    




 void HillClimbing:: hillClimbingLoop(geometry_msgs::Pose &pose,FrameGrabber & CamFrm ,FindNodes &NodeFinder){

   
        if(CamFrm.forw_Proj.getTFSuccessfully == 0){return;}
   
        headingOffset = params.hillclimbing.HeadingOffset->get();
	geometry_msgs::Pose Initpose=pose;
	robot_rot_height = pose.position.z;
	getFieldInfo(CamFrm.forw_Proj.fieldInfo);
	currentPose = pose;

	
       //pose updating to minize the error
       robotPoseUpdate(pose ,CamFrm, NodeFinder, currentError);
       currentPose = pose;
      
       
       if(sqrt(pow(Initpose.position.x - pose.position.x,2) + pow(Initpose.position.y - pose.position.y,2)
	 + pow(Initpose.position.z - pose.position.z,2)) >0.5){ currentPose = pose =Initpose;   }
      
       
       
       
       
       
       // multi hypothesis approach
       doModelMatching(pose, CamFrm,NodeFinder);
		    
       float conf = AssociateData.getConfidence();
   
       
       confidence = conf;
       if(hypothesisEvalueate( conf,   CamFrm, NodeFinder)){
		pose = hypothesisArray.poses[0];
		currentPose = pose; 
      }
      
      
      
        //covariance for KF
        calculateCov(pose, CamFrm,NodeFinder);
	

	
	//transform trunk pose to camera pose for vis
	   CamFrm.forw_Proj.setRobotPose( currentPose );
	   for(int i=0; i<hypothesisArray.poses.size(); ++i ){
		
		geometry_msgs::PoseStamped robotPoseS, camPoseS;
		robotPoseS.header = hypothesisArray.header;
		robotPoseS.pose = hypothesisArray.poses[i];
		
		
	        CamFrm.forw_Proj.TfRobotPose2CamPose( robotPoseS ,  camPoseS );
		
		geometry_msgs::Quaternion q2;
		q2.x =0; q2.y = -0.7071;q2.z =0;q2.w =0.7071;
		
		camPoseS.pose.orientation = tools.quaternionMultiplication(camPoseS.pose.orientation, q2);
		
		 hypothesisArray.poses[i]= camPoseS.pose;
		
	    }
	
	

 }
 
 
 
 void HillClimbing::robotPoseUpdate(geometry_msgs::Pose &p,FrameGrabber & CamFrm,FindNodes &NodeFinder, float &_error){
  
    headingOffset = params.hillclimbing.HeadingOffset->get();
    MovingStepInit();
    
    int Max_iter_Num = params.hillclimbing.MaxIterationNum->get();
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
		    
		    float error[poseArray.size()];
		    
		    int min_i;
		    
		    doModelMatching(p, CamFrm,NodeFinder);
		    float min_error = AssociateData.getSumOfError();
		    float min_errorAvg = AssociateData.getAvgError();
		    float max_conf = AssociateData.getConfidence();
		    min_i= 0;
// 		      cout<<"conf0      "<<max_conf  << "  error  "<<  min_error <<"  errorAvg "<<  min_errorAvg <<endl;
		    
		    for(unsigned int i=1; i<poseArray.size(); ++i){
			  doModelMatching(poseArray[i], CamFrm,NodeFinder);
			  error[i] = AssociateData.getSumOfError();
			  float errorAvg = AssociateData.getAvgError();
			  float conf = AssociateData.getConfidence();
			  float dist = getDistance( poseArray[i], p );
// 			  float yawDiff =  getYawDiff(poseArray[i], p);
// 	                  error[i] = calculateError(poseArray[i],CamFrm,NodeFinder);
			  if(  error[i] < min_error && errorAvg< min_errorAvg &&  conf>=max_conf && dist < 1.0 &&poseArray[i].position.z<0.7&& 
			    poseArray[i].position.z>0.3 ){
			      min_error = error[i];
			      min_i =i;
			      min_errorAvg= errorAvg;
			      max_conf = conf;  
			  }
		    }
		    _error = min_error;
		    
		    // stay 
		    if(min_i == 0) {
			  moving_step[idx] = std::max(  0.5 * moving_step[idx] , min_moving_step[idx]);
  // 			  continue;
		    }
		    else {//move to next  
		          geometry_msgs::Pose p0 = p;
			  p = poseArray[min_i];
			  update_times++;  

                     }
	      }
		  
	     if(update_times ==0){noUpdate++;}
	     else{noUpdate=0; }

	}
  
}


bool HillClimbing::hypothesisEvalueate(float conf, FrameGrabber & CamFrm,FindNodes &NodeFinder){
      hypothesisArray.header.seq = SeqNum++;
      hypothesisArray.header.stamp =ros::Time::now();
      hypothesisArray.poses.clear();
      hypothesisWeights.clear();
      
//       if(conf ==1){return;}
      
      int MaxHypothesisNum = params.hillclimbing.MaxHypothesisNum->get();
      
      	
      int HypothesisNum = (1-0.9*conf) * MaxHypothesisNum;
     
      if(HypothesisNum<=1){return false;}

      
      float min_error, min_errorAvg, max_conf;
       
       
      doModelMatching(currentPose, CamFrm,NodeFinder);
      min_error = AssociateData.getSumOfError();
      min_errorAvg = AssociateData.getAvgError();
      max_conf= AssociateData.getConfidence();
      
      hypothesisArray.poses.push_back(currentPose);
      hypothesisWeights.push_back(min_errorAvg);
      
       
      geometry_msgs::Pose p_optimal = currentPose;
	
      bool findbetter =0;int iter_num=0;
      while( findbetter == 0 && iter_num <4){
	    iter_num ++;
	    for(int i=1; i<HypothesisNum; ++i ){
		geometry_msgs::Pose p = SamplingPoseAround(currentPose, conf);
		
		doModelMatching(p, CamFrm,NodeFinder);
		float error1 = AssociateData.getSumOfError();
		float errorAvg1 = AssociateData.getAvgError();
		float confAvg1= AssociateData.getConfidence();
		
	        hypothesisArray.poses.push_back(p);
	        hypothesisWeights.push_back(errorAvg1);
		
		float dist = getDistance(p, currentPose );
		float yawDiff = getYawDiff(p, currentPose );
		
		double roll, pitch, yaw;
		tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
	    	q.normalized();
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		
		if( error1<min_error &&  errorAvg1<= min_errorAvg&& confAvg1>= max_conf
		   && dist<1.0  && fabs(roll) < 0.25 && fabs(pitch) < 0.25  && fabs(p.position.z-0.53) <0.5 /*&& yawDiff < 0.3* M_PI*/
		){
		    p_optimal =  p;
		    min_error = error1;
		    min_errorAvg = errorAvg1;
		    max_conf = confAvg1;
		    findbetter=1;
      
		}
	    }
	 
      }
      

          if(findbetter ==0){return false;}
 
          hypothesisArray.poses[0]= p_optimal;
	  hypothesisWeights[0]= min_errorAvg;
	  confidence = max_conf;
	  return true;
	  
}










 
void HillClimbing:: EPnPLoop(geometry_msgs::Pose &pose,FrameGrabber & CamFrm ,FindNodes &NodeFinder){
	  headingOffset = params.hillclimbing.HeadingOffset->get();
	  getFieldInfo(CamFrm.forw_Proj.fieldInfo);
	  
	  int Max_iter_Num = params.hillclimbing.MaxIterationNum->get();

	  robot_rot_height = pose.position.z;
	  
	  int iter_num=0;
	  int Update = 1;
	  float conf;
	  
	  float avgerror= 100000;
      // 	cout<< "---------------------------------------"<<endl;
	  while(avgerror >3.0  && iter_num<Max_iter_Num  &&Update==1){
                iter_num++;
		Update =0;
		doModelMatching(pose, CamFrm,NodeFinder);
		
		float errorInit = AssociateData.getSumOfError();
		float errorAvgInit = AssociateData.getAvgError();
		float confAvgInit = AssociateData.getConfidence();
		
		geometry_msgs::Pose pose_tmp = pose;
		
		if( EPnP(AssociateData,CamFrm, NodeFinder, pose_tmp )){
		      doModelMatching(pose_tmp, CamFrm,NodeFinder);
		      float error1 = AssociateData.getSumOfError();
		      float errorAvg1 = AssociateData.getAvgError();
		      float confAvg1= AssociateData.getConfidence();
		  
		      double roll, pitch, yaw;
		      tf::Quaternion q(pose_tmp.orientation.x, pose_tmp.orientation.y, pose_tmp.orientation.z, pose_tmp.orientation.w);
      	    	      q.normalized();
		      tf::Matrix3x3 m(q);
		      m.getRPY(roll, pitch, yaw);

				    
		      float dist = getDistance(pose_tmp, pose  );
		      float yawDiff =  getYawDiff(pose_tmp, pose);
		     
		     if(error1< errorInit  &&  errorAvg1<= errorAvgInit && confAvgInit <= confAvg1
		      && dist<1.0  && fabs(roll) < 0.15 && fabs(pitch) < 0.15  && fabs(pose_tmp.position.z-0.54) <0.6 && fabs(yawDiff) <0.3* M_PI){ 
		          pose = pose_tmp;
			  errorInit = error1;
			  errorAvgInit = errorAvg1;
			  confAvgInit = confAvg1;
			  Update =1;

		      }
		  
		}

                  avgerror= errorAvgInit;
		   
		  
		  
                  confidence = confAvgInit;

	          currentPose = pose;

		  if(hypothesisEvalueate2( confAvgInit,   CamFrm, NodeFinder)){
		    
		       pose = hypothesisArray.poses[0];
		       float dist2 = getDistance(currentPose, pose  );
		       float yawDiff =  getYawDiff(currentPose, pose);
		 
		       currentPose = pose;
		       avgerror =  hypothesisWeights[0];
		       Update =1;
		    
		  }
			    
		

		
	  
	  }
      
	  calculateCov(pose, CamFrm, NodeFinder);
     

}
 
 
 
bool HillClimbing:: EPnP( DataAssociation& AssociateData,FrameGrabber & CamFrm ,FindNodes &NodeFinder,geometry_msgs::Pose &pose){
   
	
	if(AssociateData.getDTMInlierNum() <4 ){return false;}

	
	std::vector<cv::Point2f> imgPts;
	std::vector<cv::Point3f> worldPts;
	
        int offsetx = (siX - W) / 2.;
	int offsety = (siY - H) / 2.;
	
	
	for ( unsigned int i = 0; i < AssociateData.DetectionToModelCorrespondences.size() ; i++ ) { 
	      
	      imgPts.push_back(cv::Point( AssociateData.DetectionToModelCorrespondences[i].first[0] - offsetx, 
					  AssociateData.DetectionToModelCorrespondences[i].first[1] - offsety ));
	      cv::Point3f pWorld;
	      cv::Point2f p = cv::Point(  AssociateData.DetectionToModelCorrespondences[i].second[0] ,  
					  AssociateData.DetectionToModelCorrespondences[i].second[1] );
	      CamFrm.forw_Proj.projectImgPoint2WorldCord(  p, pWorld);
	      worldPts.push_back(pWorld );
	}


       
	

	
      geometry_msgs::PoseStamped cameraPose;
      geometry_msgs::PoseStamped robotPose;
	
//       robotPose.pose= pose;
      EPnP_poseUpdate.calculatePose(imgPts,worldPts,CamFrm.forw_Proj,cameraPose, robotPose );

       pose= robotPose.pose;
       
      
       return true;
   
 }
 
 
 
bool HillClimbing::hypothesisEvalueate2(float conf, FrameGrabber & CamFrm,FindNodes &NodeFinder){
      hypothesisArray.header.seq = SeqNum++;
      hypothesisArray.header.stamp =ros::Time::now();
      hypothesisArray.poses.clear();
      hypothesisWeights.clear();
      
      
      int MaxHypothesisNum = params.hillclimbing.MaxHypothesisNum->get();
      
      	
      int HypothesisNum = (1-0.8*conf) * MaxHypothesisNum;
      if(HypothesisNum<=1){return false;}
     
      
      
       float min_error, min_errorAvg, max_conf;
       
       
      doModelMatching(currentPose, CamFrm,NodeFinder);
      min_error = AssociateData.getSumOfError();
      min_errorAvg = AssociateData.getAvgError();
      max_conf= AssociateData.getConfidence();
      
      hypothesisArray.poses.push_back(currentPose);
      hypothesisWeights.push_back(min_errorAvg);
      
       
      geometry_msgs::Pose p_optimal = currentPose;
	
      bool findbetter =0;int iter_num=0;
      while( findbetter == 0 && iter_num <4){
	    iter_num ++;
	    for(int i=1; i<HypothesisNum; ++i ){
		geometry_msgs::Pose p = SamplingPoseAround(currentPose, conf);
		
		doModelMatching(p, CamFrm,NodeFinder);
		EPnP(AssociateData,CamFrm, NodeFinder, p );
		
		doModelMatching(p, CamFrm,NodeFinder);
		float error1 = AssociateData.getSumOfError();
		float errorAvg1 = AssociateData.getAvgError();
		float confAvg1= AssociateData.getConfidence();
		
		
		double roll, pitch, yaw;
		tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
	    	q.normalized();
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		
		correctHeading( p);
                float dist = getDistance(p, currentPose);
		float yawDiff =  getYawDiff(p, currentPose);
		
		hypothesisArray.poses.push_back(p);
	        hypothesisWeights.push_back(errorAvg1);
		
		
		
		if( error1<min_error &&  errorAvg1<= min_errorAvg&& confAvg1>= max_conf
		   && dist<1.0  && fabs(roll) < 0.15 && fabs(pitch) < 0.15  && fabs(p.position.z-0.53) <0.4 /*&& fabs(yawDiff) <0.3* M_PI*/
		){
		    p_optimal =  p;
		    min_error = error1;
		    min_errorAvg = errorAvg1;
		    max_conf = confAvg1;
		    findbetter=1;
      
		}
	    }
	 
      }

          if(findbetter ==0){return false;}
          hypothesisArray.poses[0]= p_optimal;
	  hypothesisWeights[0]= min_errorAvg;
	  confidence = max_conf;
	  return true;
	  
	  
  
}









float HillClimbing::doModelMatching(geometry_msgs::Pose &p,FrameGrabber & CamFrm,FindNodes &NodeFinder){
  
        CamFrm.forw_Proj.setRobotPose( p);
	CamFrm.forw_Proj.GetModelLineComps();
	float errorDTM;float errorMTD;
        AssociateData.AssociateDetectionToModel( CamFrm, NodeFinder, errorDTM);
	AssociateData.AssociateModelToDetection( CamFrm, NodeFinder, errorMTD);
  
  
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
//       error =  calculateErrorForCov(pose, CamFrm, NodeFinder, K);  
      
      
      doModelMatching(pose, CamFrm, NodeFinder); 
      error = AssociateData.getSumOfError();
      K = AssociateData.getSumOfInlierNum();
      float conf = AssociateData.getConfidence();
      
      double delta = 0.01;
      
      
      for(int idx =0; idx<6; idx++){
	
	p[idx] += delta;
	pose_tmp.position.x =  p[0];
	pose_tmp.position.y =  p[1];
	pose_tmp.position.z =  p[2];
	pose_tmp.orientation = tf::createQuaternionMsgFromRollPitchYaw ( p[3], p[4], p[5]);  
// 	calculateErrorForCov(pose_tmp,CamFrm,error1,k1,pointMatcher,NodeFinder);  
//         error1 =  calculateErrorForCov(pose_tmp, CamFrm, NodeFinder, k1);  
	
	doModelMatching(pose_tmp, CamFrm, NodeFinder); 
	error1 = AssociateData.getSumOfError();
	k1 = AssociateData.getSumOfInlierNum();
	
	
	p[idx] -= 2*delta;
	pose_tmp.position.x =  p[0];
	pose_tmp.position.y =  p[1];
	pose_tmp.position.z =  p[2];
	pose_tmp.orientation = tf::createQuaternionMsgFromRollPitchYaw ( p[3], p[4], p[5]);


	doModelMatching(pose_tmp, CamFrm, NodeFinder); 
	error2 = AssociateData.getSumOfError();
	k2 = AssociateData.getSumOfInlierNum();
	
	
	
	if(K<=3){ cov[idx]= 999;}
	else if(error1 + error2 == 2* error){ cov[idx]= 5.0;}
	else{ cov[idx] =  error/(K-3) *( delta*delta/fabs(error1 + error2 - 2* error));
	  if(NodeFinder.m_LinearGraph_Buffer->size()>3 && conf>0.95 ){cov[idx] /=3.0;   }
	  
	  
	  
	}
	 
	p[idx] += delta;
// 	cout<< cov[idx]<<"  ";
	
      }
      
 
  }

 
 
 
 
 // ************************************** //
// ********* Private Methods *********** // 	
// ************************************** //


  //from Hafez
double HillClimbing::getHeading()
{       headingOffset = params.hillclimbing.HeadingOffset->get() ;
        return m_Math::CorrectAngleRadian360(headingData.heading + headingOffset );
}
void HillClimbing::getFieldInfo( FieldInfo &_fieldInfo ){
      fieldA= _fieldInfo.A;
      fieldB =_fieldInfo.B;
}





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

      
      p[idx] += dev;
      pose_tmp.position.x =  std::max(-fieldA/2.0 , p[0]) ;
      pose_tmp.position.y =  std::max(-fieldB/2.0 , p[1]) ; 
      pose_tmp.position.z =   std::max(0.4 , p[2]) ; 
      
      
      pose_tmp.position.x =  std::min(fieldA/2.0 , pose_tmp.position.x) ;
      pose_tmp.position.y =  std::min(fieldB/2.0 , pose_tmp.position.y) ;
      pose_tmp.position.z =  std::min(0.65 , pose_tmp.position.z) ;
      
      pose_tmp.orientation = tf::createQuaternionMsgFromRollPitchYaw ( p[3], p[4], p[5]);  
      poseArray.push_back( pose_tmp );
//       cout<<"yawN  "<< p[5];
      
      
      p[idx] -= dev;
      p[idx] -= dev;
      pose_tmp.position.x =  std::max(-fieldA/2.0 , p[0]) ;
      pose_tmp.position.y =  std::max(-fieldB/2.0 , p[1]) ; 
      pose_tmp.position.z =   std::max(0.4 , p[2]) ; 
      
      
      pose_tmp.position.x =  std::min(fieldA/2.0 , pose_tmp.position.x) ;
      pose_tmp.position.y =  std::min(fieldB/2.0 , pose_tmp.position.y) ;
      pose_tmp.position.z =  std::min(0.65 , pose_tmp.position.z) ;
      
  
      pose_tmp.orientation = tf::createQuaternionMsgFromRollPitchYaw ( p[3], p[4], p[5]);  
      poseArray.push_back( pose_tmp );
//       cout<<"  "<< p[5]<<endl;
}
 

geometry_msgs::Pose HillClimbing::SamplingPoseAround(geometry_msgs::Pose center, float conf){
      geometry_msgs::Pose p;

      float range_x = std::min((1 - 0.9* conf) * fieldA/2.0 * 0.5, 0.3) ;
      float range_y =  std::min( (1 - 0.9* conf) * fieldB/2.0 * 0.5, 0.3) ;
      float range_yaw = std::min((1 - 0.9* conf) * M_PI * 0.3, 0.4);
      
      float x_max= std::min(center.position.x + range_x,  fieldA/2.0 );
      float x_min= std::max(center.position.x - range_x, -fieldA/2.0 );
      float y_max= std::min(center.position.y + range_y,  fieldB/2.0 );
      float y_min= std::max(center.position.y - range_y, -fieldB/2.0 );
//       float z_max= std::min(center.position.z + 0.1,  0.59);
//       float z_min= std::max(center.position.z - 0.1,  0.55);
     
      float z_max= std::min(robot_rot_height + 0.05,  0.59);
      float z_min= std::max(robot_rot_height - 0.05,  0.55);
      
    
      
      p.position.x = random(x_min, x_max);
      p.position.y = random(y_min, y_max);
      p.position.z = random(z_min, z_max);
      

      
       double roll_ref, pitch_ref, yaw_ref;
      tf::Quaternion q(center.orientation.x,center.orientation.y, center.orientation.z,center.orientation.w);
      q.normalized();
      tf::Matrix3x3 m(q);
      m.getRPY(roll_ref, pitch_ref, yaw_ref);

      
      double roll /*= roll_ref */,pitch /*=pitch_ref*/ ,  yaw;

      
      roll  = 0;
      pitch = 0;
      
      yaw   = random(yaw_ref - range_yaw, yaw_ref + range_yaw);
      yaw = m_Math::CorrectAngleRadian360(yaw);
       

      p.orientation = tf::createQuaternionMsgFromRollPitchYaw( roll,  pitch,  yaw);
      return p;
}
 
 
 void HillClimbing:: correctHeading( geometry_msgs::Pose &pose ){
	    double roll, pitch, yaw;
	    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	    q.normalized();
	    tf::Matrix3x3 m(q);
	    m.getRPY(roll, pitch, yaw);
		
	    if(m_Math::RadianAngleDiff(yaw, getHeading()) > M_PI/2.0){
		    pose.position.x = -pose.position.x;
		    pose.position.y = -pose.position.y;
		    yaw= m_Math::CorrectAngleRadian360(yaw - M_PI);
		    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( roll,  pitch,  yaw);   
// 		    cout<<"corrected"<<endl;
	    }
}

 float HillClimbing:: getDistance( geometry_msgs::Pose &pose1, geometry_msgs::Pose &pose2 ){
	    return   sqrt(pow(pose1.position.x - pose2.position.x,2) +pow(pose1.position.y - pose2.position.y,2) 
		                 +pow(pose1.position.z - pose2.position.z,2));
}


 float HillClimbing::getYawDiff( geometry_msgs::Pose &pose1, geometry_msgs::Pose &pose2 ){
            double roll, pitch, yaw1, yaw2;
	    tf::Quaternion q(pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w);
	    q.normalized();
	    tf::Matrix3x3 m(q);
	    m.getRPY(roll, pitch, yaw1);
	    
	    tf::Quaternion q2(pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w);
	    q.normalized();
	    tf::Matrix3x3 m2(q2);
	    m2.getRPY(roll, pitch, yaw2);
	    
            return m_Math::RadianAngleDiff(yaw1, yaw2) ;
}	
	

