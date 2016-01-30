
#include <visual_tracking/vision_main.h>

using namespace std;
using namespace vision;
using namespace boost::timer;

Vision::Vision() 
{
    onInit();
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
   delete it;
   delete camera;
   
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
	image_pub_5 = it->advertise("/vision/correspondence", 1);
	image_pub_6 = it->advertise("/vision/correspondence_reverse", 1);
     
	
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
	
	
	 camera->TakeCapture();
	
	ROS_INFO("Init finished");
	
	drawField();
}


void Vision::update()
{       
  
        ros::Time t1 = ros::Time::now();
	
	//cam
	double confidence = camera->TakeCapture();
	
// 	cout<<"conf: "<<confidence<<endl;
	if (confidence < 0.75 && !camera->IsReady() )
	{
		return;
	}

	ros::Duration dura= (camera->captureTime - CameraFrame.header.stamp);
	//Init robot Pose
	if(CameraFrame.imagecounter ==0 ||  dura.toSec()<0){
	    drawField();
	    
	    robotPoseS.header.frame_id = "world";
	    robotPoseS.pose.position.x= params.location.x->get();
	    robotPoseS.pose.position.y= params.location.y->get();
	    robotPoseS.pose.position.z= params.location.z->get();
	
	    double roll  = params.orientation.x->get();//st: 0.5*0.05PI;
	    double pitch = params.orientation.y->get();//st: 0.5*0.05PI; 
	    double yaw   = params.orientation.z->get();//st: 1.0*0.05PI;
	    robotPoseS.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll, pitch, yaw);
	    CameraFrame.imagecounter =0;
  
	}
	if(CameraFrame.imagecounter% 20==0 ){
	  drawField();
	}
	
	
        CameraFrame.imagecounter++;
	CameraFrame.header.stamp = camera->captureTime;
	robotPoseS.header.stamp = camera->captureTime;
	robotPoseS.header.seq =  CameraFrame.imagecounter;
	
        cvtColor(camera->rawImage, CameraFrame.rawHSV, CV_BGR2HSV);
	
	Mat channels[3];
	split(CameraFrame.rawHSV, channels);//	channels[2]
	

        ROS_ERROR("time RGB-HSV  : %f", (ros::Time::now() - t1).toSec());
	
	
//undistorting the whole image
// 	  Mat res;
// 	  distortionModel.CreateUndistortFull(camera->rawImage,res);
// 	  imwrite( "/home/yvonne/Desktop/pic/field/undistorted.jpg", res );
// 		
// 	  cv::imshow("Undistorted",res);   
// 	  cv::waitKey(1);
 	
	
	

	
// 	Scalar m = mean(channels[2](cv::Rect(0,200,640,480-200)));
// // 	cout<<"v= "<<m.val[0]<<endl;
// 	
// 	cout<<  m.val[0]/2<<endl;
// 	if(m.val[0]>100) m.val[0]=100;  
// 	if(m.val[0]<41) m.val[0]=41;	
// 	params.ballhsv.v0->set(50+ ((m.val[0]-41)/59.)*70.);
// 	params.ballhsv.s1->set(90+ ((m.val[0]-41)/59.)*30.);
// 	params.goalhsv.v0->set(50+ ((m.val[0]-41)/59.)*70.);
// 	
// 	params.goal.lineVoteDouble->set(57- ((m.val[0]-41)/59.)*20.);
	
// 	CameraFrame.Brightness_Channel = Mat::zeros(CameraFrame.rawHSV.size(), CV_8UC1);

	
	ros::Time t2 = ros::Time::now();
	CameraFrame.Brightness_Channel = channels[2].clone();
	CameraFrame.GreenBinary = Mat::zeros(H,W, CV_8UC1);
	
	cv::inRange(CameraFrame.rawHSV, Scalar(params.fieldhsv.h0->get(), params.fieldhsv.s0->get(), params.fieldhsv.v0->get()),
		                    Scalar(params.fieldhsv.h1->get(), params.fieldhsv.s1->get(), params.fieldhsv.v1->get()), CameraFrame.GreenBinary);
	
// 	cv::imshow("binaryImgs0 ball",CameraFrame.GreenBinary);
// 	imwrite( "/home/yvonne/Desktop/pic/field/GreenBinary.jpg", CameraFrame.GreenBinary );
// 	cv::waitKey(1);

	
// 	// ***************************** //
// 	// * 2. Find Field             * //
// 	// ***************************** //
	
	if( ! FieldFinder.FindFieldConvexHull(CameraFrame.GreenBinary, CameraFrame.fieldConvexHullMat, CameraFrame.fieldConvexHullPoints,  CameraFrame. m_Top)){
	  cout<<"No Field convexhull be found." << endl;
	  return;
	}
	
// 	if( ! FieldFinder.FindFieldConvexHull(GreenBinary, CameraFrame.fieldConvexHullPoints,  CameraFrame. m_Top)){
// 	  cout<<"No Field convexhull be found." << endl;
// 	  return;
// 	}
	
	

     
     	hsvRangeC ranges[3];
	ranges[BALL_C] = params.ballhsv;
	ranges[GOAL_C] = params.goalhsv;
	ranges[BLACK_C] = params.obstaclehsv;
	bool inTemplate[3];
	inTemplate[BALL_C] = true;
	inTemplate[GOAL_C] = false;
	inTemplate[BLACK_C] = true;
	
	Mat binaryImgs[3];// ball, goal, obstacle
	binaryImgs[BALL_C]= Mat::zeros(H,W, CV_8UC1);
	binaryImgs[GOAL_C]= Mat::zeros(H,W, CV_8UC1);
	binaryImgs[BLACK_C]= Mat::zeros(H,W, CV_8UC1);
	
	FieldFinder.ColorClassification(CameraFrame.rawHSV, CameraFrame.fieldConvexHullMat, binaryImgs,ranges, inTemplate, 3);
       
	ROS_ERROR("time FindField : %f", (ros::Time::now() - t2).toSec());
	
	
// 	cv::imshow("binaryImgs[BLACK_C]",binaryImgs[BLACK_C]);
// 	imwrite( "/home/yvonne/Desktop/pic/obstacle/BLACK_C.jpg", binaryImgs[BLACK_C] );
// 	cv::waitKey(1);
	

// 	// ***************************** //
// 	// * 3. Find Obstacles           * //
// 	// ***************************** //
	CameraFrame.ObstacleConvexHull.clear();
	ObstacleFinder.GetObstacleContours( binaryImgs[BLACK_C],CameraFrame.ObstacleConvexHull);
	

// 	WhiteObjectFinder.GetwhiteSegments( CameraFrame.Brightness_Channel , CameraFrame.fieldConvexHullMat, CameraFrame.WhiteObjectConvexHull);
//      GoalPostsFinder.findGoalPosts(CameraFrame.rawHSV,CameraFrame.Brightness_Channel, CameraFrame.binaryImgs[GOAL_C],FieldFinder.fieldConvexHullPoints);

	
	
// 	// ********************************* //
// 	// * 4. Find Node Candidates * //
// 	// ********************************* //

// 	LineFinder.findLines(CameraFrame);
// 	LineFinder.findSkeletons(CameraFrame);
	ros::Time t3 = ros::Time::now();
	LineFinder.findBoundingRects(CameraFrame);
	ROS_ERROR("time findBoundingRects  : %f", (ros::Time::now() - t3).toSec());
// 	if(LineFinder.Rectangles.size()<2){return;}
	
	
	
// 	// ********************************* //
// 	// *   5. Find Nodes     * //
// 	// ********************************* //
	
	//§§§§§§§§§§§§§§§§§§§§§§§§//
// 	NodeFinder.mainLoop(CameraFrame);
	ros::Time t4 = ros::Time::now();
	NodeFinder.getRectangle(LineFinder.Rectangles );
	NodeFinder.findNodeGraph(CameraFrame);
	ROS_ERROR("time findNodeGraph  : %f", (ros::Time::now() - t4).toSec());
// 	vector<Point2f> goalPositionOnReal;
// 	vector<LineSegment> resLines, alllL;

// 	bool goalRes = goalPostFinder.GetPosts(cannyImg,rawHSV,gray,
// 			goalBinary.clone(), _cameraPronjections, hullField,
// 			resLines, alllL, goalPositioOnReal,
// 			guiRawImg_pub.thereAreListeners(), guiRawImg);
	

	
	
	//§§§§§§§§§§§§§§§§§§§§§§§§//

        
	CameraFrame.forw_Proj.onInit(CameraFrame.header.stamp);


	
	
	float error;

	
	//hillclimb method
// 	poseUpdate.mainLoop(robotPoseS.pose, CameraFrame, pointMatcher, NodeFinder);
	
	ros::Time t5 = ros::Time::now();
	if(params.debug.useSolvePnP->get()){
	  poseUpdate.IterativeLeastSquare(robotPoseS.pose, CameraFrame, NodeFinder);
	  if(params.debug.useRansac->get()){
	    ROS_ERROR("time Ransac + EPnP  : %f", (ros::Time::now() - t5).toSec());
	    
	  }
	  else{ ROS_ERROR("time EPnP  : %f", (ros::Time::now() - t5).toSec());}
	  
	  
	}
        else{
	  //naieve hill climbing method 
	        poseUpdate.mainLoop(robotPoseS.pose, CameraFrame, NodeFinder);
	        ROS_ERROR("time HillClimbing  : %f", (ros::Time::now() - t5).toSec());
	}
	
// 	ROS_ERROR("time findNodeGraph  : %f", (ros::Time::now() - t4).toSec());
// 	
	
	
	
		//for vis
        CameraFrame.forw_Proj.setRobotPose( robotPoseS.pose );
	CameraFrame.forw_Proj.GetModelLineComps();
	AssociateData.AssociateDetectionToModel( CameraFrame, NodeFinder, error);
	AssociateData.AssociateModelToDetection( CameraFrame, NodeFinder, error);
// 	 cout<<"0    "<<robotPoseS.pose.position.x << "  "<<robotPoseS.pose.position.y<< "  "<<robotPoseS.pose.position.z<<endl;
	
	
	
// 	 error=poseUpdate.AssociateData.getDTMError();
	 
// 	 cout<<"error "<<error<<endl;
	 
// 	float conf;
// 	if(error > 100000){ conf = 0; }
// 	else{ conf = (100000.0- error)/100000.0; }
// 	 cout<<"conf "<<conf<<endl;
	
	if(params.debug.useKalmanFilter->get()){
	      ros::Time t6 = ros::Time::now();
	      poseFilter.setCurTime(CameraFrame.header.stamp);
	      poseFilter.getMeasurementCov( poseUpdate.cov);
	      poseFilter.mainLoop( robotPoseS, 1.0 );
	      poseFilter.getRobotPose(robotPoseS);
	      robotPose_pub_KF.publish(poseFilter.estimate_pose_cov);
	      ROS_ERROR("time KalmanFilter  : %f", (ros::Time::now() - t6).toSec());
	}


	

	
	   //§§§§§§§§§§§§§§§§§§§§§§§§//
//         geometry_msgs::PoseStamped rawcamPose, rawRobotPose;
//         poseCalculator.calculatePose(imgPts, worldPts, CameraFrame.forw_Proj, rawcamPose, rawRobotPose);
	
	
	
	
	
	
// 	float conf =1, dist_cnf=1, num_cnf=1;
// 	
// 	  
// 	if(params.debug.useKalmanFilter->get()==false ){
// 	  
// 		      //save the new pose	
// // 	    params.location.x->set(robotPose.position.x);
// // 	    params.location.y->set(robotPose.position.y);
// // 	    params.location.z->set(robotPose.position.z);
// // 	    
// // 	    //calculate roll pitch yaw from quaternion
// // 	    tf::Quaternion q(robotPose.orientation.x, robotPose.orientation.y, robotPose.orientation.z,robotPose.orientation.w);
// // 	    q.normalized();
// // 	    tf::Matrix3x3 m(q);
// // 	    m.getRPY(roll, pitch, yaw);
// // 	    
// // 	    params.orientation.x->set(roll);
// // 	    params.orientation.y->set(pitch);
// // 	    params.orientation.z->set(yaw);
// 	  
// 	}
// 	else{
// 	    //calculate the confidence
// 	    float AvgDistThresholdForConf = params.icp.AvgDistThresholdForConf->get();
// 	    float InlierNumThresholdForConf = params.icp.InlierNumThresholdForConf->get();
// 	  
// 	    if(pointMatcher.inlierNum < 4  ){ conf = 0 ;}
// 	    else{
// 		    //useful values
// 		  float average_projected_error = 0;
// 		  for ( int i = 0; i<pointMatcher.inlierNum; ++i ){
// 		    average_projected_error += sqrt(pow(pointMatcher.correspondences[i*8 + 4] - pointMatcher.correspondences[i*8 + 0], 2) + 
// 						    pow(pointMatcher.correspondences[i*8 + 5] - pointMatcher.correspondences[i*8 + 1], 2));
// 		  }
// 		  average_projected_error/= float(pointMatcher.inlierNum );
// 		  if(average_projected_error <AvgDistThresholdForConf){ dist_cnf = 1; }
// 		  else{ dist_cnf = std::max(float(0),  1- (average_projected_error-AvgDistThresholdForConf)/(AvgDistThresholdForConf*4));}
// 		  
// 		  
// 		  if(pointMatcher.inlierNum >InlierNumThresholdForConf){ num_cnf = 1; }
// 		  else{ num_cnf = float(pointMatcher.inlierNum )/ InlierNumThresholdForConf;}  
// 		  
// 		  conf= 0.5*(dist_cnf + num_cnf);
// 	    }
//   
// 	    
// 	}
// 	
	geometry_msgs::PoseStamped camPoseS;
	CameraFrame.forw_Proj.TfRobotPose2CamPose( robotPoseS ,  camPoseS );
        
	
	
// 	// ******************************************************************//
// 	// ************************ Particles Filter*************************//
// 	// ****************************************************************** //
	
	if(params.debug.useParticleFilter->get()){
	    particlefilter.setCurTime(CameraFrame.header.stamp);
// 	    particlefilter.setdetectedLines(LineFinder.LinesOnImg_After_Merged, LineFinder.MeasConf);
// 	    particlefilter.run(); 
	}
// 	//The output of particle filter pose
//         lineMatcher.backw_Proj.onInit(CameraFrame.header.stamp);
// 	lineMatcher.backw_Proj.setRobotPose( particlefilter.robotPose.pose);
//         lineMatcher.run(LineFinder.LinesOnImg_After_Merged);
	
	
	
	
	
	// ********************************** //
	// * 7. visualization of the output * //
	// ********************************** //
	ros::Time t7 = ros::Time::now();
// 	camera pose
	if(params.debug.showRobPose->get()){
	  
	  
// 	    cameraPose_pub_ICP.publish(rawcamPose);
// 	    robotPose_pub_ICP.publish(rawRobotPose);
	   
	    robotPose_pub.publish(robotPoseS);
	    cameraPose_pub.publish(camPoseS);
	    HypothesisArray_pub.publish(poseUpdate.hypothesisArray);
// 	    cameraPose_pub.publish(particlefilter.cameraPose);
// 	    particlePose_pub.publish(particlefilter.particles_ego_rot);
//          particleCamPose_pub.publish(particlefilter.particles_camera);
	}

	
	if(params.debug.showFieldHull->get()){
	    //initial rgb image
             cv::Mat fullrgbframe = (camera->rawImage).clone();
             //field hull
	     vector<vector<cv::Point> > tmphulls = vector<vector<cv::Point> >(1,CameraFrame.fieldConvexHullPoints);
             drawContours(fullrgbframe, tmphulls, -1,   cv::Scalar(0,200,255), 2, 8);
	     
	     //obstacles
	     drawContours(fullrgbframe, CameraFrame.ObstacleConvexHull, -1,  cv::Scalar(20, 150, 50), 2, 8);
// 	     drawContours(fullrgbframe, CameraFrame.WhiteObjectConvexHull, -1,  cv::Scalar(0, 200, 0), 2, 8);
	    
// 	     imwrite( "/home/yvonne/Desktop/pic/obstacle/input.jpg", fullrgbframe );
	     imwrite( "/home/yvonne/Desktop/video/pictures/input.jpg", fullrgbframe );
	     
	    sensor_msgs::Image img_out;
	    img_out.header = CameraFrame.header;
	    img_out.height = H;
	    img_out.width = W;
	    img_out.step = 3*W;
// 	    img_out.is_bigendian = img->is_bigendian;
	    img_out.encoding = std::string("bgr8");
	    img_out.data.assign(fullrgbframe.datastart, fullrgbframe.dataend);
	    image_pub_.publish(img_out);
	}


	
	
        if(params.debug.showSkeletonPixels->get()){
	  
	      cv::Mat skeletonPixels(cv::Size(W,H),CV_8UC3,cv::Scalar(150, 150, 150));
	      

		//draw detected points
		 for ( unsigned int i = 0; i < LineFinder.detectedPoins.size() ; i++ ) { 
                   cv::circle(skeletonPixels,cv::Point( LineFinder.detectedPoins[i].x , (LineFinder.detectedPoins[i].y)), 1, cv::Scalar(0, 10, 200),-1);
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
		  img_out1.height = H;
		  img_out1.width = W;
		  img_out1.step = 3*W;
    // 	          img_out1.is_bigendian = img->is_bigendian;
		  img_out1.encoding = std::string("bgr8");
		  img_out1.data.assign(skeletonPixels.datastart, skeletonPixels.dataend);
		  image_pub_1.publish(img_out1);
	  
	          imwrite( "/home/yvonne/Desktop/video/pictures/skeletonPixels.jpg", skeletonPixels );
	}
	
	
	cv::Mat nodeGraph_undistorted;
	//         //showNodeGraph
        if(params.debug.showNodeGraph->get()){
	
	      cv::Mat nodeGraph(cv::Size(W,H),CV_8UC3,cv::Scalar(150, 150, 150));
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
		  

		  
		  
		  for(unsigned int lidx=0; lidx<ls.size();++lidx){
		    cv::line( nodeGraph, cv::Point(ls[lidx].s[0], (ls[lidx].s[1] )),  cv::Point(ls[lidx].e[0], (ls[lidx].e[1])),  color, 1, 8 );
		    cv::circle(nodeGraph,cv::Point(ls[lidx].s[0], (ls[lidx].s[1])),2, color,-1);
		    cv::circle(nodeGraph,cv::Point(ls[lidx].e[0], (ls[lidx].e[1])),2, color,-1);
		  }
		  
// 		  std::ostringstream ss0, ss1,ss2;
// 	  ss0<<"Dist confidence:    "<< dist_cnf;
// 	  ss1<<"Num confidence:     "<< num_cnf;
// 	  ss2<<"Sum of confidence:  "<< conf;
// 	
// 	  cv::putText(correspondence,ss0.str(), cv::Point(50,30),cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,0,0),1);
// 	  cv::putText(correspondence,ss1.str(), cv::Point(50,50),cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,0,0),1);
// 	  cv::putText(correspondence,ss2.str(), cv::Point(50,70),cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,0,0),1);
		  
		  
		  if(params.debug.showTangentLine->get()){

	 
		       for(unsigned int lidx=0; lidx<TangentLines.size();++lidx){
			  cv::line( nodeGraph_undistorted, cv::Point(TangentLines[lidx].s[0], (TangentLines[lidx].s[1] )), 
							    cv::Point(TangentLines[lidx].e[0], (TangentLines[lidx].e[1])),  color, 2, 8 );
// 			  cout<<TangentLines[lidx].ang<<", ";
			 
		      } 
		      
// 		        cout<<"     avg  "<< it_->undistortedAngleAvg <<"   anglechange "<<  it_->undistortedAngleChangeAvg<<endl;
// 			cout<<endl;
			 std::ostringstream angleAvg, anglechange;
			 angleAvg << m_Math::Radian2Degree(- it_->undistortedAngleAvg);
			 anglechange<<  it_->undistortedAngleChangeAvg;
			 int midP = TangentLines.size()/2;
			 cv::putText(nodeGraph_undistorted, angleAvg.str(), cv::Point(TangentLines[midP].s[0]-30, (TangentLines[midP].s[1] -50)),cv::FONT_HERSHEY_TRIPLEX,1,cv::Scalar(0,0,0),1);
			 cv::putText(nodeGraph_undistorted, anglechange.str(), cv::Point(TangentLines[midP].s[0]-30, (TangentLines[midP].s[1]-20 )),cv::FONT_HERSHEY_TRIPLEX,1,cv::Scalar(0,0,0),1);
			 

			 

		  }
		  else{
		    
		    
		      for(unsigned int lidx=0; lidx<ls_udistorted.size();++lidx){
			  cv::line(nodeGraph_undistorted, cv::Point(ls_udistorted[lidx].s[0], (ls_udistorted[lidx].s[1] )), 
				                          cv::Point(ls_udistorted[lidx].e[0], (ls_udistorted[lidx].e[1])),  color, 3, 8 );
			  
			}
			
			vector<Vec2i> &Points = it_->Points;
			
			for(int i= 0; i < Points.size(); ++i){
			    cv::circle(nodeGraph_undistorted,cv::Point(Points[i][0], (Points[i][1])),5,color,-1);
			}
			
			
		    
		  }

	      }
	  
		    
		      sensor_msgs::Image img_out2;
		      img_out2.header = CameraFrame.header;
		      img_out2.height = H;
		      img_out2.width = W;
		      img_out2.step = 3*W;
		      img_out2.encoding = std::string("bgr8");
		      img_out2.data.assign(nodeGraph.datastart, nodeGraph.dataend);
		      image_pub_2.publish(img_out2);
		      
		      
		      
		      sensor_msgs::Image img_out3;
		      img_out3.header = CameraFrame.header;
		      img_out3.height = siY;
		      img_out3.width = siX;
		      img_out3.step = 3*siX;
		      img_out3.encoding = std::string("bgr8");
		      img_out3.data.assign(nodeGraph_undistorted.datastart, nodeGraph_undistorted.dataend);
		      image_pub_3.publish(img_out3);
		      
		      imwrite( "/home/yvonne/Desktop/video/pictures/nodeGraph.jpg", nodeGraph );
		      imwrite( "/home/yvonne/Desktop/video/pictures/nodeGraph_undistorted.jpg", nodeGraph_undistorted );
		      

	  }
	    
        if(params.debug.showModelLines->get()){
	    
	       cv::Mat ModelLines(cv::Size(siX,siY),CV_8UC3,cv::Scalar(150, 150, 150));
	     
	       ModelLine_Buffer::iterator it_;
	       int j =0;
	       for ( it_ =  CameraFrame.forw_Proj.m_ModelLine_Buffer->begin( ); it_ !=  CameraFrame.forw_Proj.m_ModelLine_Buffer->end( ); it_++ ) { // 
		 j++;
		      vector<Vec2i> &points = it_->UndistortedPoints;
		      vector<Line> &ls_udistorted = it_->UndistortedLines;
		      Line &Longline = it_->LongLine;
		      if(it_->id >=0){cv::line( ModelLines, cv::Point(Longline.s[0], (Longline.s[1] )), 
					      cv::Point(Longline.e[0], (Longline.e[1])),  cv::Scalar(0,250, 255), 3, 8 ); 
			
		         
		      }
		      else{
		    	for(unsigned int lidx=0; lidx<ls_udistorted.size();++lidx){
			    cv::line( ModelLines, cv::Point(ls_udistorted[lidx].s[0], (ls_udistorted[lidx].s[1] )), 
					      cv::Point(ls_udistorted[lidx].e[0], (ls_udistorted[lidx].e[1])),  cv::Scalar(0,250, 255), 3, 8 );
			
// 			    cout<<"  "<< ls_udistorted[lidx].s[0]<<"  "<<ls_udistorted[lidx].s[1]<<"  end  "
// 			         <<ls_udistorted[lidx].e[0]<<"  "<<ls_udistorted[lidx].e[1]<<endl;
			}
		       
		      }
		     
		      for(unsigned int lidx=0; lidx<points.size();++lidx){
			cv::circle(ModelLines,cv::Point(points[lidx][0], (points[lidx][1])),3,cv::Scalar(0,250, 255),-1);
			
		      }
		 
	       }
		  
	    
	    
	      sensor_msgs::Image img_out4;
	      img_out4.header = CameraFrame.header;
	      img_out4.height = siY;
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
					  cv::Point(Longline.e[0], (Longline.e[1])),  cv::Scalar(0,250, 255), 1, 8 ); 
		  }
		  else{
		    for(unsigned int lidx=0; lidx<ls_udistorted.size();++lidx){
			cv::line( correspondence, cv::Point(ls_udistorted[lidx].s[0], (ls_udistorted[lidx].s[1] )), 
					  cv::Point(ls_udistorted[lidx].e[0], (ls_udistorted[lidx].e[1])),  cv::Scalar(0,250, 255), 1, 8 );
		
		    }
		    
		  }
		  
		  for(unsigned int lidx=0; lidx<points.size();++lidx){
		    cv::circle(correspondence,cv::Point(points[lidx][0], (points[lidx][1])),3,cv::Scalar(0,250, 255),-1);
		  }
	      
	    }

	    cv::Mat correspondence_rev =  correspondence.clone();
	    
	    
	    
// 	    for ( unsigned int i = 0; i < poseUpdate.AssociateData.DetectionToModelCorrespondences.size() ; i++ ) { 
// 		cv::Point p1(poseUpdate.AssociateData.DetectionToModelCorrespondences[i].first[0], poseUpdate.AssociateData.DetectionToModelCorrespondences[i].first[1]);
// 		cv::Point p2(poseUpdate.AssociateData.DetectionToModelCorrespondences[i].second[0], poseUpdate.AssociateData.DetectionToModelCorrespondences[i].second[1]);
// 		cv::line( correspondence,p1,p2,  cv::Scalar(24,85,200), 2, 8 );
// 	    }
// 
// 	    for ( unsigned int i = 0; i < poseUpdate.AssociateData.DetectionToModelOutliers.size() ; i++ ) { 
// 		cv::Point p1(poseUpdate.AssociateData.DetectionToModelOutliers[i][0], poseUpdate.AssociateData.DetectionToModelOutliers[i][1]);
// 		cv::circle(correspondence,p1 ,8 ,cv::Scalar(0,0,0),2);
// 
// 	    }

	    
	   for ( unsigned int i = 0; i < AssociateData.DetectionToModelCorrespondences.size() ; i++ ) { 
		cv::Point p1(AssociateData.DetectionToModelCorrespondences[i].first[0], AssociateData.DetectionToModelCorrespondences[i].first[1]);
		cv::Point p2(AssociateData.DetectionToModelCorrespondences[i].second[0], AssociateData.DetectionToModelCorrespondences[i].second[1]);
		cv::line( correspondence,p1,p2,  cv::Scalar(24,85,200), 2, 8 );
	    }

	    for ( unsigned int i = 0; i < AssociateData.DetectionToModelOutliers.size() ; i++ ) { 
		cv::Point p1(AssociateData.DetectionToModelOutliers[i][0], AssociateData.DetectionToModelOutliers[i][1]);
		cv::circle(correspondence,p1 ,8 ,cv::Scalar(0,0,0),2);

	    }

	    
	    

      
// 	  std::ostringstream ss0, ss1,ss2;
// 	  ss0<<"Dist confidence:    "<< dist_cnf;
// 	  ss1<<"Num confidence:     "<< num_cnf;
// 	  ss2<<"Sum of confidence:  "<< conf;
// 	
// 	  cv::putText(correspondence,ss0.str(), cv::Point(50,30),cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,0,0),1);
// 	  cv::putText(correspondence,ss1.str(), cv::Point(50,50),cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,0,0),1);
// 	  cv::putText(correspondence,ss2.str(), cv::Point(50,70),cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,0,0),1);
	  
	  
// 	    for ( unsigned int i = 0; i < poseUpdate.AssociateData.ModelToDetectionCorrespondences.size() ; i++ ) { 
// 	        cv::Point p1(poseUpdate.AssociateData.ModelToDetectionCorrespondences[i].first[0], poseUpdate.AssociateData.ModelToDetectionCorrespondences[i].first[1]);
// 	        cv::Point p2(poseUpdate.AssociateData.ModelToDetectionCorrespondences[i].second[0], poseUpdate.AssociateData.ModelToDetectionCorrespondences[i].second[1]);
// 		cv::line( correspondence_rev, p1,p2,  cv::Scalar(24,85,200), 2, 8 );
// 	    }
// 	    
// 	   for ( unsigned int i = 0; i < poseUpdate.AssociateData.ModelToDetectionOutliers.size() ; i++ ) { 
// 	        cv::Point p1(poseUpdate.AssociateData.ModelToDetectionOutliers[i][0], poseUpdate.AssociateData.ModelToDetectionOutliers[i][1]);
// 		cv::circle(correspondence_rev, p1 ,8 ,cv::Scalar(0,0,0),2);
//     
// 	   }
	   
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
	  img_out5.height = siY;
	  img_out5.width = siX;
	  img_out5.step = 3*siX;
  // 	    img_out5.is_bigendian = img->is_bigendian;
	  img_out5.encoding = std::string("bgr8");
	  img_out5.data.assign(correspondence.datastart, correspondence.dataend);
	  image_pub_5.publish(img_out5);
         
         
     
	  sensor_msgs::Image img_out6;
	  img_out6.header = CameraFrame.header;;
	  img_out6.height = siY;
	  img_out6.width = siX;
	  img_out6.step = 3*siX;
  // 	    img_out6.is_bigendian = img->is_bigendian;
	  img_out6.encoding = std::string("bgr8");
	  img_out6.data.assign(correspondence_rev.datastart, correspondence_rev.dataend);
	  image_pub_6.publish(img_out6);
	  imwrite( "/home/yvonne/Desktop/video/pictures/correspondence.jpg", correspondence );
	  imwrite( "/home/yvonne/Desktop/video/pictures/correspondence_rev.jpg", correspondence_rev );
	  
	  
    }
    
    
    
     ROS_ERROR("time visualization  : %f", (ros::Time::now() - t7).toSec());
    

	
// 	//HoughLines
// 	if(params.debug.showHoughLines->get()){
// 	    cv::Mat houghLines_(siY,siX,CV_8UC3,cv::Scalar(150, 150, 150));
// 	    std::ostringstream ss;
//             ss <<"Line Num: "<< LineFinder.LinesOnImg.size();
// 		cv::putText(houghLines_,ss.str(), cv::Point(100, 200),
// 						   cv::FONT_HERSHEY_TRIPLEX,3,cv::Scalar(0,0,0),2);
// 	    for(unsigned int i=0; i <LineFinder.LinesOnImg.size(); ++i ){
//     
// 		float r =  rand()*255; 
// 		float g =  rand()*255;
// 		float b =  rand()*255;
// 		cv::Point p1=  cv::Point( LineFinder.LinesOnImg[i].s[0],LineFinder.LinesOnImg[i].s[1] );
// 		cv::Point p2=  cv::Point( LineFinder.LinesOnImg[i].e[0],LineFinder.LinesOnImg[i].e[1] );
// 		cv::line( houghLines_, p1, p2, cv::Scalar( r, g, b), 5, 0 );
// 	      
// 	    }
// 	    
// 	    sensor_msgs::Image img_out3;
// 	    img_out3.header =CameraFrame.header;
// 	    img_out3.height = siY;
// 	    img_out3.width = siX;
// 	    img_out3.step = 3*siX;
// // 	    img_out3.is_bigendian = img->is_bigendian;
// 	    img_out3.encoding = std::string("bgr8");
// 	    img_out3.data.assign(houghLines_.datastart, houghLines_.dataend);
// 	    image_pub_3.publish(img_out3);
// 	}
// 	
// 	//showMergedLines
// 	if(params.debug.showMergedLines->get()){
// 	    cv::Mat MergedLines(siY,siX,CV_8UC3,cv::Scalar(150, 150, 150));
// 	    
// 	    std::ostringstream ss1,ss2;
//             ss1 <<"Line Num: "<< LineFinder.LinesOnImg_After_Merged.size();
// 	    ss2 <<"Meas Conf: "<< LineFinder.MeasConf;
// 	    cv::putText(MergedLines,ss1.str(), cv::Point(100, 200),
// 						   cv::FONT_HERSHEY_TRIPLEX,3,cv::Scalar(0,0,0),2);
// 	    cv::putText(MergedLines,ss2.str(), cv::Point(100, 300),
// 					    cv::FONT_HERSHEY_TRIPLEX,3,cv::Scalar(0,0,0),2);
// 	  
// 	    for(unsigned int i=0; i <LineFinder.LinesOnImg_After_Merged.size(); ++i ){
//     
// 		float r =  rand()*255; 
// 		float g =  rand()*255;
// 		float b =  rand()*255;
// 		cv::Point p1=  cv::Point( LineFinder.LinesOnImg_After_Merged[i].s[0],LineFinder.LinesOnImg_After_Merged[i].s[1] );
// 		cv::Point p2=  cv::Point( LineFinder.LinesOnImg_After_Merged[i].e[0],LineFinder.LinesOnImg_After_Merged[i].e[1] );
// 		cv::line( MergedLines, p1, p2, cv::Scalar( r, g, b), 5, 0 );
// 	    }
// 	    
// 	    sensor_msgs::Image img_out4;
// 	    img_out4.header =CameraFrame.header;
// 	    img_out4.height = siY;
// 	    img_out4.width = siX;
// 	    img_out4.step = 3*siX;
// // 	    img_out4.is_bigendian = img->is_bigendian;
// 	    img_out4.encoding = std::string("bgr8");
// 	    img_out4.data.assign(MergedLines.datastart, MergedLines.dataend);
// 	    image_pub_4.publish(img_out4);      
// 	}
// 	
// 
//         
//         
// 	//matching result
// 	if(params.debug.showAssociateLines->get()){
// 	    //mark model points and inlier model points on rviz
// 	    visualization_msgs::MarkerArray associateLines;  
// 	    
// 		visualization_msgs::Marker m4, m5;
// 		m4.header.frame_id = "world";
// 		m4.header.stamp = CameraFrame.header.stamp; 
// 		m4.action = visualization_msgs::Marker::ADD;
// 	        m4.pose.orientation.x = 0;m4.pose.orientation.y = 0;m4.pose.orientation.z = 0; m4.pose.orientation.w= 1.0;
// 		m4.ns = "lines";
// 		m4.id = 4;
// 		m4.pose.position.x=0;m4.pose.position.y=0;m4.pose.position.z=0;
// 		m4.type = visualization_msgs::Marker::LINE_LIST;
// 		m4.scale.x = 0.05;;
// 		m4.color.a = 1.0;  m4.color.r = 0.2; m4.color.g = 0.2; m4.color.b = 0.2;
// 		m4.points.clear();
// 
// 
// 		for( size_t i = 0; i <  lineMatcher.AssociationLines.size(); ++i ) {
// 		  geometry_msgs::Point p;
// 		  p.x = lineMatcher.AssociationLines[i].s_w[0];
// 		  p.y = lineMatcher.AssociationLines[i].s_w[1];
// 		  p.z = lineMatcher.AssociationLines[i].s_w[2];
// 		  m4.points.push_back(p);
// 
// 		  p.x = lineMatcher.AssociationLines[i].e_w[0];
// 		  p.y = lineMatcher.AssociationLines[i].e_w[1];
// 		  p.z = lineMatcher.AssociationLines[i].e_w[2];
// 		  m4.points.push_back(p);  
// 
// 	          }
// 		
// 		
// 		m5 = m4;
// 		m5.ns = "MergedLines";
// 		m5.id = 5;
// 		m5.scale.x = 0.05; 
// 		m5.color.a = 1.0;  m5.color.r = 1.0;m5.color.g = 0.2;m5.color.b = 1.0;
// 		m5.points.clear();
// 
// 		for( size_t i = 0; i <  lineMatcher.m_LineBuffer_After_Merge.size(); ++i ) {
//       
// 		    geometry_msgs::Point p;
// 		    p.x = lineMatcher.m_LineBuffer_After_Merge[i].s_w[0];
// 		    p.y = lineMatcher.m_LineBuffer_After_Merge[i].s_w[1];
// 		    p.z = lineMatcher.m_LineBuffer_After_Merge[i].s_w[2];
// 		    m5.points.push_back(p);
// 
// 		    p.x = lineMatcher.m_LineBuffer_After_Merge[i].e_w[0];
// 		    p.y = lineMatcher.m_LineBuffer_After_Merge[i].e_w[1];
// 		    p.z = lineMatcher.m_LineBuffer_After_Merge[i].e_w[2];
// 		    m5.points.push_back(p);  
// 
// 		}
// 		
// 		associateLines.markers.push_back( m4 );
// 		associateLines.markers.push_back( m5 );
// 	  
// 	  
// 	     associateLines_pub.publish( associateLines );
// 	}
        
        
        
        

	

// 	  // **************************************** //
// 	  // * publishing the output processed image* //
// 	  // *****************************************//
//          

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
      fieldLines.scale.x = 0.025;
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
// 	    GeeenGround.pose.orientation.x =0.0;GeeenGround.pose.orientation.y = 0;GeeenGround.pose.orientation.z = 0; GeeenGround.pose.orientation.w= 1.0;
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
// 	    goalPosts.points.clear();
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
    
    
    //process the yuyv format images
    Vision soccer_vision;
    
//     soccer_vision.onInit();
    ros::Publisher fps_pub = soccer_vision.nh.advertise<std_msgs::Int64>("/vision/fps", 10);
    
    
    const int RATE = 30;
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
// 	  if(soccer_vision.CameraFrame.imagecounter >0 ){
	      
	     soccer_vision.update();
	    
// 	  }
// 	  else{
// 	    ROS_INFO("No Input Image Message.");
// 	  }
	  if(loopCounter > 4){
	      fpsData = (0.9 * fpsData) + (0.1 * (1000000000l / timer.elapsed().wall)); // pow(10,9)nanosecond = 1 second 
	      time_msg.data = fpsData;
	      
	      ROS_ERROR("time Update Loop  : %f, %f,  %f", (ros::Time::now() - t1).toSec() , timer.elapsed().wall/ 1000000000l ,  0.9*avg_update_frame_rate + (0.1 *1.0/(ros::Time::now() - t1).toSec()));
	      
	      cout<< fpsData<<endl;
	      ROS_ERROR("-----------------------------------------------------" );
	  }
// 	  if(params.debug.publishTime->get()){
	      fps_pub.publish(time_msg);
	    
// 	  }
	  
	  //SpinOnce, just make the loop operate once 
	  ros::spinOnce();
	    
	  // sleep, to keep 50ms delay
	  loop_rate.sleep();
    }


//     ros::spin();
//     delete cam;
    return 0;
  
}