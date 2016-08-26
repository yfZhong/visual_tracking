
#include <visual_tracking/vision_main.h>

using namespace std;
using namespace vision;
using namespace boost::timer;

Vision::Vision() 
{
    onInit();
 
    if(params.debug.saveProcessingTime->get()){
         string pkg_path = ros::package::getPath("visual_tracking") +"/../bags/processingTime/";
    }
    
    if(params.debug.saveTrajectory->get()){
         string pkg_path = ros::package::getPath("visual_tracking") +"/../bags/trajectory/";
         const char* path1= (pkg_path + "traj.csv").c_str(); 
	 trajectory_data.open (path1, ofstream::out|ofstream::trunc);
    }
    previous_time_to_begin =0;
      
}

Vision::~Vision()
{
   image_sub_.shutdown();
   image_pub_.shutdown();
   image_pub_1.shutdown();
   image_pub_2.shutdown();
   image_pub_3.shutdown();
   image_pub_4.shutdown();
   image_pub_5.shutdown();
   image_pub_6.shutdown();
   robotPose_pub.shutdown();
   cameraPose_pub.shutdown();
   associateLines_pub.shutdown();
   particlePose_pub.shutdown();
   particleCamPose_pub.shutdown();
   particlePoseMarker_pub.shutdown();
   cameraPose_pub_ICP.shutdown();
   robotPose_pub_ICP.shutdown();
   trajectory_pub.shutdown();
   
   read_trajectory_pub1.shutdown();
   read_trajectory_pub2.shutdown();
   read_trajectory_pub3.shutdown();
   read_trajectory_pub4.shutdown();
   read_trajectory_pub5.shutdown();
   read_trajectory_pub6.shutdown();
   read_trajectory_pub7.shutdown();
   read_trajectory_pub8.shutdown();
   read_trajectory_pub9.shutdown();
   
   
   delete it;
   delete camera;
   if(params.debug.saveProcessingTime->get()){
	  processingTime.close();
   }
   if(params.debug.saveTrajectory->get()){
	  trajectory_data.close();
   }	
   
}


void Vision::onInit(){
	
    ROS_INFO("Starting Nimbro-OP vision node");

    useBagfile = params.camera.useBagfile->get();

    if (useBagfile == true)
    {
	    camera = new CameraDummy();

    }
    else
    { 
	    camera = new Camera();
    }

      
    if (false == camera->InitCameraDevice(true))
    {
	    ROS_ERROR("Failed to initialize Camera!");
    }
    ROS_INFO("Camera is Started");

    it = new image_transport::ImageTransport(nh);	

    image_pub_  = it->advertise("/vision/fieldHull", 1);
    image_pub_1 = it->advertise("/vision/skeletonPixels", 1);
    image_pub_2 = it->advertise("/vision/nodeGraph", 1);
    image_pub_3 = it->advertise("/vision/nodeGraph_undistorted", 1);
    image_pub_4 = it->advertise("/vision/ModelLines", 1);
    image_pub_5 = it->advertise("/vision/observations_to_model", 1);
    image_pub_6 = it->advertise("/vision/model_to_observations", 1);


    robotPose_pub = nh.advertise< geometry_msgs::PoseStamped >( "/robotPose", 2 );
    cameraPose_pub = nh.advertise< geometry_msgs::PoseStamped >( "/cameraPose", 2 );
    if(params.debug.useKalmanFilter->get()){
	robotPose_pub_KF = nh.advertise< geometry_msgs::PoseWithCovarianceStamped  >( "/KFRobotPoses", 2 );
    }
    associateLines_pub = nh.advertise< visualization_msgs::MarkerArray >( "associateLines", 1 );
    particlePose_pub = nh.advertise< geometry_msgs::PoseArray >( "/particlePoses", 2 );
    particleCamPose_pub  = nh.advertise< geometry_msgs::PoseArray >( "/particleCamPoses", 2 );
    particlePoseMarker_pub = nh.advertise< visualization_msgs::MarkerArray >( "/particlePosesMarker", 1 );
    fieldLineMarker_pub =  nh.advertise< visualization_msgs::MarkerArray >( "/fieldElementsMarker", 1 );
    cameraPose_pub_ICP =  nh.advertise< geometry_msgs::PoseStamped >( "/IcpCamPoses", 2 );
    robotPose_pub_ICP = nh.advertise< geometry_msgs::PoseStamped >( "/IcpRobotPoses", 2 );

    HypothesisArray_pub = nh.advertise< geometry_msgs::PoseArray >( "/hypothesisArray", 2 );

    trajectory_pub =  nh.advertise< visualization_msgs::MarkerArray >( "/trajectory", 1 );

    read_trajectory_pub1 =  nh.advertise< visualization_msgs::MarkerArray >( "/trajectory1", 1 );
    read_trajectory_pub2 =  nh.advertise< visualization_msgs::MarkerArray >( "/trajectory2", 1 );
    read_trajectory_pub3 =  nh.advertise< visualization_msgs::MarkerArray >( "/trajectory3", 1 );
    read_trajectory_pub4 =  nh.advertise< visualization_msgs::MarkerArray >( "/trajectory4", 1 );

    read_trajectory_pub5 =  nh.advertise< visualization_msgs::MarkerArray >( "/trajectory5", 1 );
    read_trajectory_pub6 =  nh.advertise< visualization_msgs::MarkerArray >( "/trajectory6", 1 );
    read_trajectory_pub7 =  nh.advertise< visualization_msgs::MarkerArray >( "/trajectory7", 1 );
    read_trajectory_pub8 =  nh.advertise< visualization_msgs::MarkerArray >( "/trajectory8", 1 );
    read_trajectory_pub9 =  nh.advertise< visualization_msgs::MarkerArray >( "/trajectory_ER_G", 1 );


    camera->TakeCapture();
    startingTime =  ros::Time::now();

    ROS_INFO("Init finished");

    drawField();
}


void Vision::update()
{       
        //only use for visualizing the trajectory after you having a .csv file with trajectory data.
        if(params.debug.visTrajectoryData->get()){
	   drawField();
	   vis_trajectory();
	   return;
	}
       
        ros::Time t1 = ros::Time::now();
	double timeToBegin = (t1 - startingTime).toSec();;
	
	//cam
	double confidence = camera->TakeCapture();
	if (confidence < 0.75 && !camera->IsReady())
	{
		return;
	}

	ros::Duration dura= (camera->captureTime - CameraFrame.header.stamp);
	
	//Init robot Pose
	if(CameraFrame.imagecounter ==0 //begining
	  ||  dura.toSec()<0 // bagfile jump back to the begining 
	  ||(x_init!=params.location.x->get()|| y_init!=params.location.y->get() 
	  ||  yaw_init !=params.orientation.z->get()) // //detects changes in parameter tunner
	){
	    drawField();
	    
	    robotPoseS.header.frame_id = "world";
	    x_init = robotPoseS.pose.position.x= params.location.x->get();
	    y_init = robotPoseS.pose.position.y= params.location.y->get();
	    robotPoseS.pose.position.z= params.location.z->get();
	
	    double roll  = params.orientation.x->get();
	    double pitch = params.orientation.y->get();
	    double yaw   = params.orientation.z->get();
	    yaw_init =yaw;
	    robotPoseS.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll, pitch, yaw);
	    
	}
	

	if(CameraFrame.imagecounter < 20 ){ drawField();} //vis the soccer field model

	double roll, pitch, yaw;
	tf::Quaternion q(robotPoseS.pose.orientation.x,robotPoseS. pose.orientation.y,robotPoseS. pose.orientation.z, robotPoseS.pose.orientation.w);
	q.normalized();
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw); 

	//initial the height z, roll, pitch using IMU or Kinematic model	
	tf::StampedTransform floor_rot;
	try{
	      CameraFrame.forw_Proj. listener.lookupTransform("/ego_floor", "/ego_rot",
		    camera->captureTime- ros::Duration(params.projection.timeToShift->get()),floor_rot);

	      robotPoseS.pose.position. z = floor_rot.getOrigin().getZ();
	      
	      tfScalar roll2, pitch2, yaw2;
	      floor_rot.getBasis().getEulerYPR(yaw2, pitch2, roll2);
	      roll = roll2;
	      pitch = pitch2;
		
	      }
	    catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform a pose : %s", ex.what());
	} 
      
	robotPoseS.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( roll,  pitch,  yaw);
	
	 
// 	// ***************************** //
// 	// * 1. Input BRG--HSV             * //
// 	// ***************************** //
        CameraFrame.imagecounter++;
	CameraFrame.header.stamp = camera->captureTime;
	robotPoseS.header.stamp = camera->captureTime;
	robotPoseS.header.seq =  CameraFrame.imagecounter;
	
	Mat input_img = camera->rawImage;
        cvtColor(input_img, CameraFrame.rawHSV, CV_BGR2HSV);
	
	Mat channels[3];
	split(CameraFrame.rawHSV, channels);//	channels[2]
	
 	
	CameraFrame.Brightness_Channel = channels[2].clone();
	
	
		
// 	// ***************************** //
// 	// * 2. Find Field             * //
// 	// ***************************** //
	ros::Time t2 = ros::Time::now();
	CameraFrame.GreenBinary = Mat::zeros(H,W, CV_8UC1);
	cv::inRange(CameraFrame.rawHSV, Scalar(params.fieldhsv.h0->get(), params.fieldhsv.s0->get(), params.fieldhsv.v0->get()),
		                    Scalar(params.fieldhsv.h1->get(), params.fieldhsv.s1->get(), params.fieldhsv.v1->get()), CameraFrame.GreenBinary);
        
	if( ! FieldFinder.FindFieldConvexHull(CameraFrame.GreenBinary, CameraFrame.fieldConvexHullMat, CameraFrame.fieldConvexHullPoints,  CameraFrame. m_Top)){
	    cout<<"No Field convexhull be found." << endl;
	    return;
	}
	double fieldTime = (ros::Time::now() - t2).toSec();

	
	
// 	// ***************************** //
// 	// * 3. Find Obstacles           * //
// 	// ***************************** //		

        ros::Time t22 = ros::Time::now();
     	hsvRangeC ranges[3];
	ranges[BALL_C] = params.ballhsv;
	ranges[GOAL_C] = params.goalhsv;
	ranges[BLACK_C] = params.obstaclehsv;
	bool inTemplate[3];
	inTemplate[BALL_C] = true;
	inTemplate[GOAL_C] = false;
	inTemplate[BLACK_C] = true;
	
	Mat binaryImgs[3];// ball, goal, obstacle

	
// 	FieldFinder.ColorClassification(CameraFrame.rawHSV, CameraFrame.fieldConvexHullMat, binaryImgs,ranges, inTemplate, 3);
// 	CameraFrame.ObstacleConvexHull.clear();
// 	ObstacleFinder.GetObstacleContours( binaryImgs[BLACK_C],CameraFrame.ObstacleConvexHull);
	
	double obstacleTime = (ros::Time::now() - t22).toSec();
	
// 	WhiteObjectFinder.GetwhiteSegments( CameraFrame.Brightness_Channel , CameraFrame.fieldConvexHullMat, CameraFrame.WhiteObjectConvexHull);
//      GoalPostsFinder.findGoalPosts(CameraFrame.rawHSV,CameraFrame.Brightness_Channel, CameraFrame.binaryImgs[GOAL_C],FieldFinder.fieldConvexHullPoints);

	
	
// 	// ********************************* //
// 	// * 4. Find Node Candidates * //
// 	// ********************************* //
        ros::Time t3 = ros::Time::now();
// 	LineFinder.findLines(CameraFrame);
// 	LineFinder.findSkeletons(CameraFrame);
	
	LineFinder.findBoundingRects(CameraFrame);
// 	ROS_ERROR("time findBoundingRects  : %f", (ros::Time::now() - t3).toSec());
	if(LineFinder.Rectangles.size()<2){return;}
	
	
// 	// ********************************* //
// 	// *   5. Find Nodes     * //
// 	// ********************************* //

	ros::Time t4 = ros::Time::now();
	NodeFinder.getRectangle(LineFinder.Rectangles);
	NodeFinder.findNodeGraph(CameraFrame);

	double lineTime = (ros::Time::now() - t3).toSec();
	
	//check the current observation is a bad observation or not.
	bool bad_obs = 1;
	int count =0;
	float sumOfLength=0;
	LinearGraph_Buffer::iterator it_;  
	for ( it_ = NodeFinder.m_LinearGraph_Buffer->begin( ); it_ != NodeFinder.m_LinearGraph_Buffer->end( ); it_++ ) { // 
	    if((*it_).sumOfLength < params.graphNode._MinCmpLineLength->get()){continue;}
	    count ++;
	    sumOfLength +=(*it_).sumOfLength;
	    if(count >=2 &&sumOfLength >200 ){ bad_obs =0; break; }

	}

        
	CameraFrame.forw_Proj.onInit(CameraFrame.header.stamp);

	
	float error;
	
// 	// ********************************* //
// 	// *   6.1. KF prediction step    * //
// 	// ********************************* //

	poseFilter.setCurTime(CameraFrame.header.stamp);
	
	  if(!params.debug.useMotionOdom){
	      poseFilter.prediction(); //use the motion speed from deead reconking data.
	  }
	  else{
	    poseFilter.prediction3();
	  }
	poseFilter.getRobotPose(robotPoseS);
	
	double trackTime=0;

// 	// ************************************************** //
// 	// *   7. Solving PnP problem/ pose optimization    * //
// 	// ************************************************** //
	
	ros::Time t5 = ros::Time::now();
	if(params.debug.useSolvePnP->get() && !bad_obs){  //use EPnP
	  poseUpdate.EPnPLoop(robotPoseS.pose, CameraFrame, NodeFinder);
	  trackTime = (ros::Time::now() - t5).toSec();  
	}
        else if(!bad_obs){ //naieve hill climbing method 
	        poseUpdate.hillClimbingLoop(robotPoseS.pose, CameraFrame, NodeFinder);
		 trackTime = (ros::Time::now() - t5).toSec();
// 	        ROS_ERROR("time HillClimbing  : %f", (ros::Time::now() - t5).toSec());
	}


// 	// ********************************* //
// 	// *   6.2. KF correction step    * //
// 	// ********************************* //

	double KfTime=0;
        ros::Time t6 = ros::Time::now();
	poseFilter.getMeasurementCov( poseUpdate.cov);
	poseFilter.correction( robotPoseS, poseUpdate.confidence );
	poseFilter.getRobotPose(robotPoseS);
	robotPose_pub_KF.publish(poseFilter.estimate_pose_cov);
	

	KfTime =(ros::Time::now() - t6).toSec();
	
	if(params.debug.saveProcessingTime->get() & dura.toSec()>0){
	  processingTime<<CameraFrame.imagecounter<<"   "<<fieldTime<<"   "<<obstacleTime
	  <<"   "<<lineTime <<"   "<<trackTime<<"   "<<KfTime<<endl;
	}
	
	
	//for vis
        CameraFrame.forw_Proj.setRobotPose( robotPoseS.pose );
	CameraFrame.forw_Proj.GetModelLineComps();
	AssociateData.AssociateDetectionToModel( CameraFrame, NodeFinder, error);
	AssociateData.AssociateModelToDetection( CameraFrame, NodeFinder, error);
	

	//transform robot trunk pose to camera pose.
	geometry_msgs::PoseStamped camPoseS;
	CameraFrame.forw_Proj.TfRobotPose2CamPose( robotPoseS ,  camPoseS );
	
	
// 	// ******************************************************************//
// 	// ************************ Particles Filter*************************//
// 	// ****************************************************************** //
	
// 	if(params.debug.useParticleFilter->get()){
// 	    particlefilter.setCurTime(CameraFrame.header.stamp);
// 	    particlefilter.setdetectedLines(LineFinder.LinesOnImg_After_Merged, LineFinder.MeasConf);
// 	    particlefilter.run(); 
// 	}
// 	//The output of particle filter pose
//         lineMatcher.backw_Proj.onInit(CameraFrame.header.stamp);
// 	lineMatcher.backw_Proj.setRobotPose( particlefilter.robotPose.pose);
//         lineMatcher.run(LineFinder.LinesOnImg_After_Merged);
	
	
	
	
	
	// ********************************** //
	// * 8. visualization of the output * //
	// ********************************** //
	ros::Time t7 = ros::Time::now();
	if(params.debug.showRobPose->get()){
	  
	   
	    robotPose_pub.publish(robotPoseS);
	    cameraPose_pub.publish(camPoseS);
	    HypothesisArray_pub.publish(poseUpdate.hypothesisArray);
	}

	
	if(params.debug.showFieldHull->get()){
	    //initial rgb image
             cv::Mat fullrgbframe = input_img.clone();
             //field hull
	     vector<vector<cv::Point> > tmphulls = vector<vector<cv::Point> >(1,CameraFrame.fieldConvexHullPoints);
             drawContours(fullrgbframe, tmphulls, -1,   cv::Scalar(0,200,255), 2, 8);
	     
	     //obstacles
	     
// 	     drawContours(fullrgbframe, CameraFrame.ObstacleConvexHull, -1,  cv::Scalar(20, 20, 200), 2, 8);
	     

	    sensor_msgs::Image img_out;
	    img_out.header = CameraFrame.header;
	    img_out.height = fullrgbframe.rows;
	    img_out.width = W;
	    img_out.step = 3*W;
// 	    img_out.is_bigendian = img->is_bigendian;
	    img_out.encoding = std::string("bgr8");
	    img_out.data.assign(fullrgbframe.datastart, fullrgbframe.dataend);
	    
	    image_pub_.publish(img_out);
	    
	    
	}


	 
	cv::Mat nodeGraph;
        if(params.debug.showSkeletonPixels->get()){
	  
	      cv::Mat skeletonPixels(cv::Size(W,H),CV_8UC3,cv::Scalar(150, 150, 150));
	      

		//draw detected points
		 for ( unsigned int i = 0; i < LineFinder.detectedPoins.size() ; i++ ) { 
		    if(LineFinder.detectedPoinsWithType[i].second ==0 ){
// 		    cv::circle(skeletonPixels,cv::Point( LineFinder.detectedPoins[i].x , (LineFinder.detectedPoins[i].y)), 0, cv::Scalar(0, 10, 200),1, 8, 2);
		    skeletonPixels.data[ (LineFinder.detectedPoins[i].y * W + LineFinder.detectedPoins[i].x)*3 +0 ] = 0;
		    skeletonPixels.data[ (LineFinder.detectedPoins[i].y * W + LineFinder.detectedPoins[i].x)*3 +1 ] = 10;
		    skeletonPixels.data[ (LineFinder.detectedPoins[i].y * W + LineFinder.detectedPoins[i].x)*3 +2 ] = 250;
		  }
                   
                   else if(LineFinder.detectedPoinsWithType[i].second ==1 ){
//                    cv::circle(skeletonPixels,cv::Point( LineFinder.detectedPoins[i].x , (LineFinder.detectedPoins[i].y)), 0, cv::Scalar(0, 200, 10),1, 8, 2);
		    skeletonPixels.data[ (LineFinder.detectedPoins[i].y * W + LineFinder.detectedPoins[i].x)*3 +0 ] = 0;
		    skeletonPixels.data[ (LineFinder.detectedPoins[i].y * W + LineFinder.detectedPoins[i].x)*3 +1 ] = 200;
		    skeletonPixels.data[ (LineFinder.detectedPoins[i].y * W + LineFinder.detectedPoins[i].x)*3 +2 ] = 10;
		  
		   
		  }
                   
                   else if(LineFinder.detectedPoinsWithType[i].second ==2){
//                    cv::circle(skeletonPixels,cv::Point( LineFinder.detectedPoins[i].x , (LineFinder.detectedPoins[i].y)), 0, cv::Scalar(200, 10, 0),1, 8, 2);
		    skeletonPixels.data[ (LineFinder.detectedPoins[i].y * W + LineFinder.detectedPoins[i].x)*3 +0 ] = 200;
		    skeletonPixels.data[ (LineFinder.detectedPoins[i].y * W + LineFinder.detectedPoins[i].x)*3 +1 ] = 10;
		    skeletonPixels.data[ (LineFinder.detectedPoins[i].y * W + LineFinder.detectedPoins[i].x)*3 +2 ] = 0;
	
		  }
		        
	 }
		  
    
         
		  for(unsigned int i=0; i<LineFinder.Rectangles.size(); i++){
		      cv::line( skeletonPixels, cv::Point((LineFinder.Rectangles[i].rect.x),                                    (LineFinder.Rectangles[i].rect.y )), 
					    cv::Point((LineFinder.Rectangles[i].rect.x+ LineFinder.Rectangles[i].rect.width), (LineFinder.Rectangles[i].rect.y  )),   cv::Scalar(0,0,0), 1, 8 );
		      cv::line( skeletonPixels, cv::Point((LineFinder.Rectangles[i].rect.x+ LineFinder.Rectangles[i].rect.width), (LineFinder.Rectangles[i].rect.y  )),   
					    cv::Point((LineFinder.Rectangles[i].rect.x+ LineFinder.Rectangles[i].rect.width), (LineFinder.Rectangles[i].rect.y + LineFinder.Rectangles[i].rect.height )),   cv::Scalar(0,0,0), 1, 8 );
		      cv::line( skeletonPixels, cv::Point((LineFinder.Rectangles[i].rect.x+ LineFinder.Rectangles[i].rect.width), (LineFinder.Rectangles[i].rect.y + LineFinder.Rectangles[i].rect.height )),  
					    cv::Point((LineFinder.Rectangles[i].rect.x),                                    (LineFinder.Rectangles[i].rect.y + LineFinder.Rectangles[i].rect.height )),    cv::Scalar(0,0,0), 1, 8 );
		      cv::line( skeletonPixels, cv::Point((LineFinder.Rectangles[i].rect.x),                                    (LineFinder.Rectangles[i].rect.y + LineFinder.Rectangles[i].rect.height )),
					    cv::Point((LineFinder.Rectangles[i].rect.x),                                    (LineFinder.Rectangles[i].rect.y )),  cv::Scalar(0,0,0), 1, 8 ); 
		    }
		    
		  
		  sensor_msgs::Image img_out1;
		  img_out1.header = CameraFrame.header;
		  img_out1.height = skeletonPixels.rows;
		  img_out1.width = W;
		  img_out1.step = 3*W;
    // 	          img_out1.is_bigendian = img->is_bigendian;
		  img_out1.encoding = std::string("bgr8");
		  img_out1.data.assign(skeletonPixels.datastart, skeletonPixels.dataend);
		  image_pub_1.publish(img_out1);
	  
		  nodeGraph = skeletonPixels.clone();
	}
	
	
	cv::Mat nodeGraph_undistorted;
        //showNodeGraph
        if(params.debug.showNodeGraph->get()){
	
	       nodeGraph_undistorted =cv::Mat(cv::Size(siX,siY),CV_8UC3,cv::Scalar(150, 150, 150));

	       vector<vector<cv::Point> > hulls = vector<vector<cv::Point> >(1,FieldFinder.fieldConvexHullPointsUndistort);
//              drawContours(nodeGraph_undistorted, hulls, -1,  cv::Scalar(100, 0, 10), 2, 8);
	     
	      
	      LinearGraph_Buffer::iterator it_;
      
	      int colorIdx=-1;
	      
	      for ( it_ = NodeFinder.m_LinearGraph_Buffer->begin( ); it_ != NodeFinder.m_LinearGraph_Buffer->end( ); it_++ ) { // 
		  
		  
		  if((*it_).sumOfLength < params.graphNode._MinCmpLineLength->get()){continue;}
		  colorIdx++;
		  vector<Line> &ls = it_->Lines;
		  vector<Line> &ls_udistorted = it_->UndistortedLines;
		  vector<Line> &TangentLines = it_->TangentLines;
		
		  cv::Scalar color = NodeFinder.colorPool[ colorIdx%(NodeFinder.colorPool.size())];
		  
                  cv::Scalar colorFix = cv::Scalar(0, 0, 0);
		  cv::Scalar colorNode = cv::Scalar(0, 0, 0);
		  
		  for(unsigned int lidx=0; lidx<ls.size();++lidx){
		    cv::line( nodeGraph, cv::Point(ls[lidx].s[0], (ls[lidx].s[1] )),  cv::Point(ls[lidx].e[0], (ls[lidx].e[1])),  colorFix, 2, 8 );
		    cv::circle(nodeGraph,cv::Point(ls[lidx].s[0], (ls[lidx].s[1])),3, color,2,8);
		    cv::circle(nodeGraph,cv::Point(ls[lidx].e[0], (ls[lidx].e[1])),3, color,2,8);
		  }
		  
		  
		  
		  if(params.debug.showTangentLine->get()){

	 
		       for(unsigned int lidx=0; lidx<TangentLines.size();++lidx){
			  cv::line( nodeGraph_undistorted, cv::Point(TangentLines[lidx].s[0], (TangentLines[lidx].s[1] )), 
							    cv::Point(TangentLines[lidx].e[0], (TangentLines[lidx].e[1])),  color, 3, 8 );
			 
		      } 
		      
			 std::ostringstream angleAvg, anglechange;
			 angleAvg << m_Math::Radian2Degree(- it_->undistortedAngleAvg);
			 anglechange<<  it_->undistortedAngleChangeAvg;
			 int midP = TangentLines.size()/2;

			vector<Vec2i> &Points = it_->Points;
			
			for(int i= 0; i < Points.size(); ++i){
			    cv::circle(nodeGraph_undistorted,cv::Point(Points[i][0], (Points[i][1])),3,color,-1);
			}
			 
			 

		  }
		  else{
		    
		    
		      for(unsigned int lidx=0; lidx<ls_udistorted.size();++lidx){
			  cv::line(nodeGraph_undistorted, cv::Point(ls_udistorted[lidx].s[0], (ls_udistorted[lidx].s[1] )), 
				                          cv::Point(ls_udistorted[lidx].e[0], (ls_udistorted[lidx].e[1])),  color, 2, 8 );
			  
			}
			
			vector<Vec2i> &Points = it_->Points;
			
			for(int i= 0; i < Points.size(); ++i){
			    cv::circle(nodeGraph_undistorted,cv::Point(Points[i][0], (Points[i][1])),3,color,3,8 );
			}

		    
		  }

	      }
	      

	      
		      
	      sensor_msgs::Image img_out2;
	      img_out2.header = CameraFrame.header;
	      img_out2.height = nodeGraph.rows;
	      img_out2.width = W;
	      img_out2.step = 3*W;
	      img_out2.encoding = std::string("bgr8");
	      img_out2.data.assign(nodeGraph.datastart, nodeGraph.dataend);
	      image_pub_2.publish(img_out2);

		
		
		
	      sensor_msgs::Image img_out3;
	      img_out3.header = CameraFrame.header;
	      img_out3.height = nodeGraph_undistorted.rows;
	      img_out3.width = nodeGraph_undistorted.cols;
	      img_out3.step = 3*nodeGraph_undistorted.cols;
	      img_out3.encoding = std::string("bgr8");
	      img_out3.data.assign(nodeGraph_undistorted.datastart, nodeGraph_undistorted.dataend);
	      image_pub_3.publish(img_out3);
		            
		      

	  }
	    
        if(params.debug.showModelLines->get()){
	    
	       cv::Mat ModelLines(cv::Size(siX,siY ),CV_8UC3,cv::Scalar(150, 150, 150));
	     
	       ModelLine_Buffer::iterator it_;
	       int j =0;
	       for ( it_ =  CameraFrame.forw_Proj.m_ModelLine_Buffer->begin( ); it_ !=  CameraFrame.forw_Proj.m_ModelLine_Buffer->end( ); it_++ ) { // 
		 j++;
		      vector<Vec2i> &points = it_->UndistortedPoints;
		      vector<Line> &ls_udistorted = it_->UndistortedLines;
		      Line &Longline = it_->LongLine;
		      if(it_->id >=0){cv::line( ModelLines, cv::Point(Longline.s[0], Longline.s[1] ), 
					      cv::Point(Longline.e[0], Longline.e[1]),  cv::Scalar(0,250, 255), 3, 8 ); 
			
		        cv::circle(ModelLines,cv::Point(Longline.s[0], Longline.s[1]),5,cv::Scalar(0,250, 255),-1);
		        cv::circle(ModelLines,cv::Point(Longline.e[0], Longline.e[1]),5,cv::Scalar(0,250, 255),-1);
		         
		      }
		      else{
		    	for(unsigned int lidx=0; lidx<ls_udistorted.size();++lidx){
			    cv::line( ModelLines, cv::Point(ls_udistorted[lidx].s[0], (ls_udistorted[lidx].s[1] )), 
					      cv::Point(ls_udistorted[lidx].e[0], (ls_udistorted[lidx].e[1])),  cv::Scalar(0,250, 255), 3, 8 );
			
			}
		       
		       
		          for(unsigned int lidx=0; lidx<points.size();++lidx){
			    cv::circle(ModelLines,cv::Point(points[lidx][0], (points[lidx][1])),5,cv::Scalar(0,250, 255),-1);
			
		           }
		       
		      }
		     
		   
		 
	       }
		  
	    
	      sensor_msgs::Image img_out4;
	      img_out4.header = CameraFrame.header;
	      img_out4.height = ModelLines.rows;
	      img_out4.width = siX;
	      img_out4.step = 3*siX;
	      img_out4.encoding = std::string("bgr8");
	      img_out4.data.assign(ModelLines.datastart, ModelLines.dataend);
	      image_pub_4.publish(img_out4);

	    
	  }
	     
	    
        //correspondence
	if(params.debug.showCorrespondence->get()){
// 	    cv::Mat correspondence(cv::Size(siX,siY),CV_8UC3,cv::Scalar(150, 150, 150));
	    cv::Mat correspondence =nodeGraph_undistorted.clone();
	    
	    ModelLine_Buffer::iterator it_;
	    for ( it_ =  CameraFrame.forw_Proj.m_ModelLine_Buffer->begin( ); it_ !=  CameraFrame.forw_Proj.m_ModelLine_Buffer->end( ); it_++ ) { // 
		  vector<Vec2i> &points = it_->UndistortedPoints;
		  vector<Line> &ls_udistorted = it_->UndistortedLines;
		  Line &Longline = it_->LongLine;
		  if(it_->id >=0){cv::line( correspondence, cv::Point(Longline.s[0], (Longline.s[1] )), 
					  cv::Point(Longline.e[0], (Longline.e[1])),  cv::Scalar(0,250, 255), 2, 8 ); 
		  
		        cv::circle(correspondence,cv::Point(Longline.s[0], Longline.s[1]),3,cv::Scalar(0,250, 255),-1);
		        cv::circle(correspondence,cv::Point(Longline.e[0], Longline.e[1]),3,cv::Scalar(0,250, 255),-1);
		  }
		  else{
		    for(unsigned int lidx=0; lidx<ls_udistorted.size();++lidx){
			cv::line( correspondence, cv::Point(ls_udistorted[lidx].s[0], (ls_udistorted[lidx].s[1] )), 
					  cv::Point(ls_udistorted[lidx].e[0], (ls_udistorted[lidx].e[1])),  cv::Scalar(0,250, 255), 2, 8 );
			for(unsigned int lidx=0; lidx<points.size();++lidx){
			    cv::circle(correspondence,cv::Point(points[lidx][0], (points[lidx][1])),3,cv::Scalar(0,250, 255),-1);
			}
		    }
		    
		  }
		  
		 
	      
	    }

	    cv::Mat correspondence_rev =  correspondence.clone();
	    
	   for ( unsigned int i = 0; i < AssociateData.DetectionToModelCorrespondences.size() ; i++ ) { 
		cv::Point p1(AssociateData.DetectionToModelCorrespondences[i].first[0], AssociateData.DetectionToModelCorrespondences[i].first[1]);
		cv::Point p2(AssociateData.DetectionToModelCorrespondences[i].second[0], AssociateData.DetectionToModelCorrespondences[i].second[1]);
		cv::line( correspondence,p1,p2,  cv::Scalar(24,85,200), 2, 8 );
	    }

	    for ( unsigned int i = 0; i < AssociateData.DetectionToModelOutliers.size() ; i++ ) { 
		cv::Point p1(AssociateData.DetectionToModelOutliers[i][0], AssociateData.DetectionToModelOutliers[i][1]);
		cv::circle(correspondence,p1 ,8 ,cv::Scalar(0,0,0),2);

	    }

	   
	    for ( unsigned int i = 0; i < AssociateData.ModelToDetectionCorrespondences.size() ; i++ ) { 
	        cv::Point p1(AssociateData.ModelToDetectionCorrespondences[i].first[0], AssociateData.ModelToDetectionCorrespondences[i].first[1]);
	        cv::Point p2(AssociateData.ModelToDetectionCorrespondences[i].second[0], AssociateData.ModelToDetectionCorrespondences[i].second[1]);
		cv::line( correspondence_rev, p1,p2,  cv::Scalar(24,85,200), 2, 8 );
	    }
	    
	   for ( unsigned int i = 0; i < AssociateData.ModelToDetectionOutliers.size() ; i++ ) { 
	        cv::Point p1(AssociateData.ModelToDetectionOutliers[i][0], AssociateData.ModelToDetectionOutliers[i][1]);
		cv::circle(correspondence_rev, p1 ,8 ,cv::Scalar(0,0,0),2);
    
	   }
	   
	   
          sensor_msgs::Image img_out5;
	  img_out5.header = CameraFrame.header;;
	  img_out5.height = correspondence.rows;
	  img_out5.width = siX;
	  img_out5.step = 3*siX;
	  img_out5.encoding = std::string("bgr8");
	  img_out5.data.assign(correspondence.datastart, correspondence.dataend);
	  image_pub_5.publish(img_out5);
           
     
	  sensor_msgs::Image img_out6;
	  img_out6.header = CameraFrame.header;;
	  img_out6.height = correspondence_rev.rows;
	  img_out6.width = siX;
	  img_out6.step = 3*siX;
	  img_out6.encoding = std::string("bgr8");
	  img_out6.data.assign(correspondence_rev.datastart, correspondence_rev.dataend);
	  image_pub_6.publish(img_out6);
	  
    }
    
    

    int update_trajectory=0;
    if( previous_time_to_begin + 2 < ceil(timeToBegin)  ){
	update_trajectory=1;
	previous_time_to_begin = ceil(timeToBegin);
      
    }
   
     
    if( ceil(timeToBegin) ==1){  cam_position =camPoseS.pose.position;  }

    if(params.debug.showTrajectory->get() &&update_trajectory==1 && timeToBegin>3){
	//mark model points and inlier model points on rviz
	    visualization_msgs::Marker m;
	    m.header.frame_id = "world";
	    m.header.stamp = CameraFrame.header.stamp; 
	    m.pose.orientation.x = 0;m.pose.orientation.y = 0;m.pose.orientation.z = 0; m.pose.orientation.w= 1.0;
	    m.color.a = 1.0;
	    m.scale.x = 0.07;
	    m.scale.y = 0.07;
	    m.scale.z = 0.07;

	    m.ns = "trajectory";
	    m.action = visualization_msgs::Marker::ADD;
	    m.color.r = 0.0;m.color.g = 0.0;m.color.b = 0.0;


	    m.pose.orientation.x = 0;m.pose.orientation.y = 0;m.pose.orientation.z = 0; m.pose.orientation.w= 1.0;

	    m.id = id++;
	    m.pose.position.x=0;m.pose.position.y=0;m.pose.position.z=0;
	    m.type = visualization_msgs::Marker::LINE_LIST;
	    m.scale.x = 0.01;
	
	    m.points.clear();
	    m.lifetime = ros::Duration();
	    
	    
	    geometry_msgs::Point p1 = cam_position;
	    m.points.push_back(p1);
	    
	    cam_position =camPoseS.pose.position;
	    geometry_msgs::Point p2 = cam_position;
	    m.points.push_back(p2);
	    trajectory.markers.push_back(m);

	
	  trajectory_pub.publish( trajectory );
    }
  
   
           

}


void Vision::drawField(){
      visualization_msgs::MarkerArray fieldElements;
  
      visualization_msgs::Marker fieldLines, GeeenGround, goalPosts;
      fieldLines.header.frame_id = "world";
      fieldLines.header.stamp = CameraFrame.header.stamp; 
      fieldLines.action = visualization_msgs::Marker::ADD;
      fieldLines.pose.orientation.x = 0;fieldLines.pose.orientation.y = 0;fieldLines.pose.orientation.z = 0; fieldLines.pose.orientation.w= 1.0;
      fieldLines.ns = "fieldLines";
      fieldLines.id = 1;
      fieldLines.pose.position.x=0;fieldLines.pose.position.y=0;fieldLines.pose.position.z=0;
      fieldLines.type = visualization_msgs::Marker::LINE_LIST;
      fieldLines.scale.x = 0.05;
      fieldLines.scale.y = 0.05;
      fieldLines.scale.z = 0.05;
      fieldLines.color.a = 1.0; fieldLines.color.r = 1.0;fieldLines.color.g = 1.0;fieldLines.color.b = 1.0;
      fieldLines.points.clear();
      fieldLines.lifetime = ros::Duration(60*60);
      
      goalPosts = GeeenGround = fieldLines;
      geometry_msgs::Point p;
      for( size_t i = 0; i <  lineMatcher.Field_Lines.size(); ++i ) {
	  p.x = lineMatcher.Field_Lines[i].s_w[0];
	  p.y = lineMatcher.Field_Lines[i].s_w[1];
	  p.z = lineMatcher.Field_Lines[i].s_w[2]-0.01;
	  fieldLines.points.push_back(p);

	  p.x = lineMatcher.Field_Lines[i].e_w[0];
	  p.y = lineMatcher.Field_Lines[i].e_w[1];
	  p.z = lineMatcher.Field_Lines[i].e_w[2]-0.01;
	  fieldLines.points.push_back(p);  
      }
      
      for( size_t i = 0; i <  lineMatcher.Field_Circle_Lines_Vis.size(); ++i ) {
	  p.x = lineMatcher.Field_Circle_Lines_Vis[i].s_w[0];
	  p.y = lineMatcher.Field_Circle_Lines_Vis[i].s_w[1];
	  p.z = lineMatcher.Field_Circle_Lines_Vis[i].s_w[2]-0.01;
	  fieldLines.points.push_back(p);

	  p.x = lineMatcher.Field_Circle_Lines_Vis[i].e_w[0];
	  p.y = lineMatcher.Field_Circle_Lines_Vis[i].e_w[1];
	  p.z = lineMatcher.Field_Circle_Lines_Vis[i].e_w[2]-0.01;
	  fieldLines.points.push_back(p);  

      }
      

      GeeenGround.ns = "GeeenGround";
      GeeenGround.id = 2;
      GeeenGround.scale.x = 0.05;
      GeeenGround.color.a = 1.0; GeeenGround.color.r = 0.0;GeeenGround.color.g = 0.6;GeeenGround.color.b = 0.0;
      GeeenGround.points.clear();
 
      for(int i =0; i<50; ++i){
	  float y = float(i)/50 * lineMatcher.fieldInfo.B/2;
	  p.x = -lineMatcher.fieldInfo.A /2.0;  
	  p.y = y;
	  p.z = -0.05;
	  GeeenGround.points.push_back(p);

	  p.x = lineMatcher.fieldInfo.A /2.0;
	  p.y = y;
	  p.z = -0.05;
	  GeeenGround.points.push_back(p);

	  p.x = -lineMatcher.fieldInfo.A /2.0;  
	  p.y = -y;
	  p.z = -0.05;
	  GeeenGround.points.push_back(p);

	  p.x = lineMatcher.fieldInfo.A /2.0;
	  p.y = -y;
	  p.z = -0.05;
	  GeeenGround.points.push_back(p);

      }

      
      
      goalPosts.ns = "goalPosts";
      goalPosts.type = visualization_msgs::Marker::CYLINDER;
      goalPosts.id = 3;
      goalPosts.scale.x = 0.1;goalPosts.scale.y = 0.1;goalPosts.scale.z = lineMatcher.fieldInfo.H;
      goalPosts.color.a = 1.0; goalPosts.color.r = 1.0;goalPosts.color.g = 1.0;goalPosts.color.b = 1.0;

      goalPosts.pose.position.x = -lineMatcher.fieldInfo.A /2.0 ;  
      goalPosts.pose.position.y = -lineMatcher.fieldInfo.F /2.0 ;
      goalPosts.pose.position.z =  lineMatcher.fieldInfo.H /2.0 ;
      fieldElements.markers.push_back( goalPosts);
      
      goalPosts.id = 4;
      goalPosts.pose.position.x = lineMatcher.fieldInfo.A /2.0;  
      fieldElements.markers.push_back( goalPosts);
      
      goalPosts.id = 5;
      goalPosts.pose.position.y = lineMatcher.fieldInfo.F /2.0;
      fieldElements.markers.push_back( goalPosts);
      
      goalPosts.id = 6;
      goalPosts.pose.position.x = -lineMatcher.fieldInfo.A /2.0;  
      fieldElements.markers.push_back( goalPosts);
      
      fieldElements.markers.push_back( fieldLines);
      fieldElements.markers.push_back( GeeenGround);
      fieldLineMarker_pub.publish(fieldElements);
         
}






int main( int argc, char** argv ) { 
  
    // initialize
    ros::init(argc, argv, "soccer_vision");
 
    params.Init();
    distortionModel.Init();
    
   
    //process the images
    Vision soccer_vision;
    
    //soccer_vision.onInit();
    ros::Publisher fps_pub = soccer_vision.nh.advertise<std_msgs::Int64>("/vision/fps", 10);
    
    
    const int RATE = 20;
    double fpsData = RATE;
    // determines the number of loops per second 
    VisionRate loop_rate(RATE, true);
    int loopCounter=0;
     
    float avg_update_frame_rate=30;;
     
    // loop stops if the node stops, e.g. by getting a kill signal 
    while (soccer_vision.nh.ok())
    {
          loopCounter++;
	  cpu_timer timer;
	  std_msgs::Int64 time_msg;

          ros::Time t1 = ros::Time::now();   

	      
	  soccer_vision.update();

	  if(loopCounter > 4){
	      fpsData = (0.9 * fpsData) + (0.1 * (1000000000l / timer.elapsed().wall)); // pow(10,9)nanosecond = 1 second 
	      time_msg.data = fpsData;
	  }

	  fps_pub.publish(time_msg);
	   
	  //SpinOnce, just make the loop operate once 
	  ros::spinOnce();
	    
	  // sleep, to keep 50ms delay
	  loop_rate.sleep();
 	  //r.sleep();
    }

    return 0;
  
}