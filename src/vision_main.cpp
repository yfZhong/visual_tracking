
#include <visual_tracking/vision_main.h>

using namespace std;
using namespace vision;
using namespace boost::timer;

Vision::Vision() 
{
    W = params.camera.width->get();
    H = params.camera.height->get();
    siX = params.camera.widthUnDistortion->get();
    siY = params.camera.heightUnDistortion->get();
    
    onInit();
    drawField();
}

Vision::~Vision()
{
   image_sub_.shutdown();
   image_pub_.shutdown();
   image_pub_1.shutdown();
   image_pub_2.shutdown();
   image_pub_3.shutdown();
   image_pub_4.shutdown();
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
	image_pub_3 = it->advertise("/vision/houghLines", 1);
	image_pub_4 = it->advertise("/vision/MergedLines", 1);
	image_pub_6 = it->advertise("/vision/correspondence", 1);
     
	
	robotPose_pub = nh.advertise< geometry_msgs::PoseStamped >( "/robotPose", 2 );
	cameraPose_pub = nh.advertise< geometry_msgs::PoseStamped >( "/cameraPose", 2 );
	associateLines_pub = nh.advertise< visualization_msgs::MarkerArray >( "associateLines", 1 );
        particlePose_pub = nh.advertise< geometry_msgs::PoseArray >( "/particlePoses", 2 );
	particleCamPose_pub  = nh.advertise< geometry_msgs::PoseArray >( "/particleCamPoses", 2 );
	particlePoseMarker_pub = nh.advertise< visualization_msgs::MarkerArray >( "/particlePosesMarker", 1 );
	fieldLineMarker_pub =  nh.advertise< visualization_msgs::MarkerArray >( "/fieldElementsMarker", 1 );
	cameraPose_pub_ICP =  nh.advertise< geometry_msgs::PoseStamped >( "/IcpCamPoses", 2 );
	robotPose_pub_ICP = nh.advertise< geometry_msgs::PoseStamped >( "/IcpRobotPoses", 2 );
	robotPose_pub_KF = nh.advertise< geometry_msgs::PoseWithCovarianceStamped  >( "/KFRobotPoses", 2 );
	
	
	
	ROS_INFO("Init finished");
}


void Vision::update()
{
	
	//cam
	double confidence = camera->TakeCapture();
	
	if (confidence < 0.75 && !camera->IsReady() )
	{
		return;
	}
        CameraFrame.imagecounter++;
	CameraFrame.header.stamp = camera->captureTime;
	
        cvtColor(camera->rawImage, CameraFrame.rawHSV, CV_BGR2HSV);
	
	Mat channels[3];
	split(CameraFrame.rawHSV, channels);//	channels[2]

	
// 	Scalar m = mean(channels[2](Rect(0,200,640,480-200)));
// 	cout<<"b= "<<m<<endl;
// 	if(m.val[0]>100) m.val[0]=100;  
// 	if(m.val[0]<41) m.val[0]=41;	
// 	params.ballhsv.v0->set(50+ ((m.val[0]-41)/59.)*70.);
// 	params.ballhsv.s1->set(90+ ((m.val[0]-41)/59.)*30.);
// 	params.goalhsv.v0->set(50+ ((m.val[0]-41)/59.)*70.);
// 	
// 	params.goal.lineVoteDouble->set(57- ((m.val[0]-41)/59.)*20.);
	
// 	CameraFrame.Brightness_Channel = Mat::zeros(CameraFrame.rawHSV.size(), CV_8UC1);

	
	CameraFrame.Brightness_Channel = channels[2].clone();
	
// 	CameraFrame.GreenBinary =  Mat::zeros(CameraFrame.rawHSV.size(), CV_8UC1);
// 	CameraFrame.binaryImgs[BALL_C] = Mat::zeros(H,W, CV_8UC1);
//      CameraFrame.binaryImgs[GOAL_C] = Mat::zeros(H,W, CV_8UC1);
// 	CameraFrame.binaryImgs[BLACK_C] = Mat::zeros(H,W, CV_8UC1);
// 	Mat fieldBinaryRaw = Mat::zeros(CameraFrame.rawHSV.size(), CV_8UC1);
	
	
	cv::inRange(CameraFrame.rawHSV, Scalar(params.fieldhsv.h0->get(), params.fieldhsv.s0->get(), params.fieldhsv.v0->get()),
		                    Scalar(params.fieldhsv.h1->get(), params.fieldhsv.s1->get(), params.fieldhsv.v1->get()), CameraFrame.GreenBinary);
	
// 	// ***************************** //
// 	// * 2. Find Field             * //
// 	// ***************************** //
	
	if( ! FieldFinder.FindFieldConvexHull(CameraFrame.GreenBinary, CameraFrame.fieldConvexHullMat, CameraFrame.fieldConvexHullPoints,  CameraFrame. m_Top)){
	  cout<<"No Field convexhull be found." << endl;
	  return;
	}
     
     	hsvRangeC ranges[3];
	ranges[BALL_C] = params.ballhsv;
	ranges[GOAL_C] = params.goalhsv;
	ranges[BLACK_C] = params.obstaclehsv;
	bool inTemplate[3];
	inTemplate[BALL_C] = true;
	inTemplate[GOAL_C] = false;
	inTemplate[BLACK_C] = true;
	
	
	FieldFinder.ColorClassification(CameraFrame.rawHSV, CameraFrame.fieldConvexHullMat, CameraFrame.binaryImgs,ranges, inTemplate, 3);
       

// 	// ***************************** //
// 	// * 3. Find Obstacles           * //
// 	// ***************************** //
	CameraFrame.ObstacleConvexHull.clear();
	ObstacleFinder.GetObstacleContours( CameraFrame.binaryImgs[BLACK_C],CameraFrame.ObstacleConvexHull);

	
	
//        GoalPostsFinder.findGoalPosts(CameraFrame.rawHSV,CameraFrame.Brightness_Channel, CameraFrame.binaryImgs[GOAL_C],FieldFinder.fieldConvexHullPoints);


//  	
//      cv::imshow("binaryImgs0 ball",CameraFrame.binaryImgs[0]);
// 	cv::waitKey(1);
	
	
// 	// ********************************* //
// 	// * 4. Find Node Candidates * //
// 	// ********************************* //

// 	LineFinder.findLines(CameraFrame);
	LineFinder.findSkeletons(CameraFrame);
	
	
// 	// ********************************* //
// 	// *   5. Find Nodes     * //
// 	// ********************************* //
	
	//§§§§§§§§§§§§§§§§§§§§§§§§//
	NodeFinder.mainLoop(CameraFrame);
	
// 	vector<Point2f> goalPositionOnReal;
// 	vector<LineSegment> resLines, alllL;

// 	bool goalRes = goalPostFinder.GetPosts(cannyImg,rawHSV,gray,
// 			goalBinary.clone(), _cameraProjections, hullField,
// 			resLines, alllL, goalPositionOnReal,
// 			guiRawImg_pub.thereAreListeners(), guiRawImg);
	
// 	
// 	
// 
// // 	 cout<<LineFinder.LinesOnImg.size()<<endl;
// 	 
// // 	double yaw = 30./180. * M_PI;double roll = 30./180. * M_PI;double pitch = 30./180. * M_PI;
// // 	geometry_msgs::Quaternion a = tf::createQuaternionMsgFromRollPitchYaw ( 0,  pitch,  yaw);
// // 	cout<<"a  "<< a.x<<", "<< a.y<<", "<< a.z<<", "<< a.w<<std::endl;
// // 	
// // // 	yaw = 60./180. * M_PI;
// // 	double delta_yaw = 30./180. * M_PI;
// // 	geometry_msgs::Quaternion b = tf::createQuaternionMsgFromRollPitchYaw ( 0, 0,  delta_yaw);
// // 	cout<<"b  "<< b.x<<", "<< b.y<<", "<< b.z<<", "<< b.w<<std::endl;
// // 	
// // // 	geometry_msgs::Quaternion c = tf::createQuaternionMsgFromRollPitchYaw ( 0, 0,  yaw);
// // // 	std::cout<<"c "<< c.x<<", "<< c.y<<", "<< c.z<<", "<< c.w<<std::endl;
// // 	
// // 	
// // 	
// // 	geometry_msgs::Quaternion d = tools.quaternionMultiplication(b,a);
// // // 	d = tools.quaternionMultiplication(c,d);
// // 	std::cout<<"a*b "<< d.x<<", "<< d.y<<", "<< d.z<<", "<< d.w<<std::endl;
// // 	
// // 	geometry_msgs::Quaternion e = tf::createQuaternionMsgFromRollPitchYaw ( 0,  pitch,  yaw+delta_yaw);
// // 	
// // //         c = tools.quaternionMultiplication(b,a);
// // 	std::cout<<"e  "<< e.x<<", "<< e.y<<", "<< e.z<<", "<< e.w<<std::endl;
// // 	
// // 	
// // 	cout<<endl;
// 	
// 	
// 	
// 	
// 	/*
// 	 yaw = -90./180. * M_PI;
// 	 a = tf::createQuaternionMsgFromRollPitchYaw ( 0,  0,  yaw);
// 	cout<<"-90  "<< a.x<<", "<< a.y<<", "<< a.z<<", "<< a.w<<std::endl;
// 	
// 	
// 	
// 	
// 	 yaw = 270./180. * M_PI;
// 	 a = tf::createQuaternionMsgFromRollPitchYaw ( 0,  0,  yaw);
// 	 cout<<"270  "<< a.x<<", "<< a.y<<", "<< a.z<<", "<< a.w<<std::endl;
// 	
// 	cout<<endl;*/
// // 	cout<<endl;
// 	
// 	
// 	
// 	// ********************************* //
// 	// particle
// 	// ********************************* //
// // 	geometry_msgs::Quaternion a,b;
// // 	a.x= 0; a.y=0; a.z=1; a.w=1;
// // 	tools.quaternionNormalize(a);
// // 	 std::cout<< a.x<<", "<< a.y<<", "<< a.z<<", "<< a.w<<std::endl;
// // 	b.x= 0; b.y=1; b.z=0; b.w=0;
// 
// // 	geometry_msgs::Quaternion c = tools.quaternionMultiplication(a,b);
// // 		      std::cout<< c.x<<", "<< c.y<<", "<< c.z<<", "<< c.w<<std::endl;
// // 	robotPose.orientation = c ;
// 	
        
	
	
	//§§§§§§§§§§§§§§§§§§§§§§§§//
	geometry_msgs::Pose robotPose;
	robotPose.position.x= params.location.x->get();robotPose.position.y= params.location.y->get();robotPose.position.z= params.location.z->get();
	
        float roll  = params.orientation.x->get();//st: 0.5*0.05PI;
        float pitch = params.orientation.y->get();//st: 0.5*0.05PI; 
        float yaw   = params.orientation.z->get();//st: 1.0*0.05PI;
        robotPose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll, pitch, yaw);
	       
	CameraFrame.forw_Proj.onInit(CameraFrame.header.stamp);;
	CameraFrame.forw_Proj.setRobotPose(robotPose);
	CameraFrame.forw_Proj.mainLoop();	
	
        pointMatcher.matcher(NodeFinder.undistortedNodeSamplePoins, CameraFrame.forw_Proj.ModelPointsInImg);
	
	
	float AvgDistThresholdForConf = params.icp.AvgDistThresholdForConf->get();
	float InlierNumThresholdForConf = params.icp.InlierNumThresholdForConf->get();
	float conf;float dist_cnf, num_cnf;
	if(pointMatcher.inlierNum <4  ){ conf = 0 ;}
	else{
	  
	      
		//useful values
	      float average_projected_error = 0;
	      for ( int i = 0; i<pointMatcher.inlierNum; ++i ){
		average_projected_error += sqrt(pow(pointMatcher.correspondences[i*8 + 4] - pointMatcher.correspondences[i*8 + 0], 2) + 
						pow(pointMatcher.correspondences[i*8 + 5] - pointMatcher.correspondences[i*8 + 1], 2));
	      }
	      average_projected_error/= float(pointMatcher.inlierNum );
	      if(average_projected_error <AvgDistThresholdForConf){ dist_cnf = 1; }
	      else{ dist_cnf = std::max(float(0),  1- (average_projected_error-AvgDistThresholdForConf)/(AvgDistThresholdForConf*4));}
	      
	      
	       if(pointMatcher.inlierNum >InlierNumThresholdForConf){ num_cnf = 1; }
	       else{ num_cnf = float(pointMatcher.inlierNum )/ InlierNumThresholdForConf;}  
	       
	       conf= 0.5*(dist_cnf + num_cnf);
	 
	}
	
	int offsetx = (siX - W) / 2.;
	int offsety = (siY - H) / 2.;
	std::vector<cv::Point2f> imgPts;// 2d image points
	std::vector<cv::Point3f> worldPts;//3D world points
	
	
	//point to Point
	for (unsigned int i = 0; i<pointMatcher.inlierIdx.size(); ++i ){
	    int idx = pointMatcher.inlierIdx[i];
	    cv::Point  inlier =  CameraFrame.forw_Proj.ModelPointsInImgWithId[idx].first;
	    imgPts.push_back(cv::Point(inlier.x- offsetx ,inlier.y-offsety ));
	    
	    int id = CameraFrame.forw_Proj.ModelPointsInImgWithId[idx].second;
	    bool findProjectedP = false;
	    if( id>=0){
	       for(unsigned int j=0; j <CameraFrame.forw_Proj.Field_Lines_Img.size(); ++j ){
		    if(id == CameraFrame.forw_Proj.Field_Lines_Img[j].id){
			Vec2i  pProject;
			m_Math::GetProjectivePoint( Vec2i(pointMatcher.correspondences[i*8 + 2], pointMatcher.correspondences[i*8 + 3] ), CameraFrame.forw_Proj.Field_Lines_Img[j] ,  pProject);
			cv::Point  pOnImg (pProject[0],pProject[1]);
			pointMatcher.correspondences[i*8 + 6] = pProject[0];
			pointMatcher.correspondences[i*8 + 7] = pProject[1];
			
			cv::Point3f  pWorld;
			CameraFrame.forw_Proj.projectImgPoint2WorldCord(pOnImg, pWorld);
			findProjectedP = true;
			worldPts.push_back(pWorld);
			break;
		    }
	        }
	    }
	    if(findProjectedP == false) {
		  worldPts.push_back(cv::Point3f(CameraFrame.forw_Proj.ModelPointsInWorldCord[idx].x,
						 CameraFrame.forw_Proj.ModelPointsInWorldCord[idx].y,
						 CameraFrame.forw_Proj.ModelPointsInWorldCord[idx].z));
	    }
  
        }
//         
	
	
	
	

	
        //Point to plane
//         for (unsigned int i = 0; i<model.pointMatcher.inlierIdx.size(); ++i ){
// 	    int idx = model.pointMatcher.inlierIdx[i];
// 	    cv::Point  inlier =  model.forw_Proj.ModelPointsInImgWithId[idx].first;
// 	    imgPts.push_back(cv::Point(inlier.x- offsetx ,inlier.y-offsety ));
// 
// 	    int id = model.forw_Proj.ModelPointsInImgWithId[idx].second;
// 	  
// 	    cv::Point  pOnImg;
// 	    if(id<0){ pOnImg =  cv::Point(model.pointMatcher.correspondences[i*8 + 4],model.pointMatcher.correspondences[i*8 + 5]); }
// 	    
// 	    else{
// 	       bool findProjectedP = false;
// 	       for(unsigned int j=0; j <model.forw_Proj.Field_Lines_Img.size(); ++j ){
// 		 
// 		    if(id == model.forw_Proj.Field_Lines_Img[j].id){
// 			Vec2i  pProject;
// 			m_Math::GetProjectivePoint( Vec2i(model.pointMatcher.correspondences[i*8 + 2], model.pointMatcher.correspondences[i*8 + 3] ), 
// 						    model.forw_Proj.Field_Lines_Img[j] ,  pProject);
// 			pOnImg =  cv::Point(pProject[0],pProject[1]);
//                         findProjectedP = true;
// 			break;
// 		    }
// 	        }
// 	        if(findProjectedP ==false){pOnImg =  cv::Point(model.pointMatcher.correspondences[i*8 + 4],model.pointMatcher.correspondences[i*8 + 5]);
// 		}
// // 			model.pointMatcher.correspondences[i*8 + 6] = pOnImg.x;
// // 			model.pointMatcher.correspondences[i*8 + 7] = pOnImg.y;
// 			cout<< model.forw_Proj.ModelPointsInImg[idx].x<<" =? "<<pOnImg.x<<"--------------"<<
// 		               model.forw_Proj.ModelPointsInImg[idx].y<<" =? "<< pOnImg.y<<endl;
// 	    }
// 	    
// 	    cv::Point3f  pWorld;
// 	    model.forw_Proj.projectImgPoint2WorldCord(pOnImg, pWorld);
// 	    worldPts.push_back(pWorld);
//         }
	 
	 
// 	for ( int i = 0; i<model.pointMatcher.inlierNum; ++i ){
// 	    imgPts.push_back(cv::Point(model.pointMatcher.correspondences[i*8 + 0]- offsetx , model.pointMatcher.correspondences[i*8 + 1]-offsety ));
// 	}
// 	for (unsigned int i = 0; i<model.pointMatcher.inlierIdx.size(); ++i ){
//              worldPts.push_back(cv::Point3f(model.forw_Proj.ModelPointsInWorldCord[model.pointMatcher.inlierIdx[i]].x,
// 					    model.forw_Proj.ModelPointsInWorldCord[model.pointMatcher.inlierIdx[i]].y,
// 					    model.forw_Proj.ModelPointsInWorldCord[model.pointMatcher.inlierIdx[i]].z));
//   
//         }

	
	
	
	
	
	
	
	
	   //§§§§§§§§§§§§§§§§§§§§§§§§//
        geometry_msgs::PoseStamped rawcamPose, rawRobotPose;
        poseCalculator.calculatePose(imgPts, worldPts, CameraFrame.forw_Proj, rawcamPose, rawRobotPose);
	
	
	
       poseFilter.setCurTime(CameraFrame.header.stamp );
       poseFilter.measurementSub( rawRobotPose, conf );
	
	
	
	
// 	// ******************************************************************//
// 	// ************************ Particles Filter*************************//
// 	// ****************************************************************** //
// 	particlefilter.setCurTime(CameraFrame.header.stamp);
//         particlefilter.setdetectedLines(LineFinder.LinesOnImg_After_Merged, LineFinder.MeasConf);
// 	particlefilter.run();
// 	//The output of particle filter pose
//         lineMatcher.backw_Proj.onInit(CameraFrame.header.stamp);
// 	lineMatcher.backw_Proj.setRobotPose( particlefilter.robotPose.pose);
//         lineMatcher.run(LineFinder.LinesOnImg_After_Merged);
	
	
	
	
	
	// ********************************** //
	// * 7. visualization of the output * //
	// ********************************** //
	
// 	camera pose
	if(params.debug.showRobPose->get()){
	  
	  
	    cameraPose_pub_ICP.publish(rawcamPose);
	    robotPose_pub_ICP.publish(rawRobotPose);
	    robotPose_pub_KF.publish(poseFilter.estimate_pose_cov);
	    
// 	    robotPose_pub.publish(particlefilter.robotPose);
// 	    cameraPose_pub.publish(particlefilter.cameraPose);
// 	    particlePose_pub.publish(particlefilter.particles_ego_rot);
//          particleCamPose_pub.publish(particlefilter.particles_camera);
	}
	if(params.debug.showFieldModel->get()){
            drawField();
	}
	
	if(params.debug.showFieldHull->get()){
	    //initial rgb image
             cv::Mat fullrgbframe = (camera->rawImage).clone();
             //field hull
	     vector<vector<cv::Point> > tmphulls = vector<vector<cv::Point> >(1,CameraFrame.fieldConvexHullPoints);
             drawContours(fullrgbframe, tmphulls, -1,  cv::Scalar(200, 0, 50), 2, 8);
	     
	     //obstacles
	     drawContours(fullrgbframe, CameraFrame.ObstacleConvexHull, -1,  cv::Scalar(0, 0, 0), 2, 8);
	     
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
	  
// 	    cv::Mat skeletonPixels(cv::Size(siX,siY),CV_8UC3,cv::Scalar(150, 150, 150));
// 
// 	    //draw detected points
// 	    for ( unsigned int i = 0; i < LineFinder.undistortedDetectedPoints.size() ; i++ ) { 
// 		cv::circle(skeletonPixels, LineFinder.undistortedDetectedPoints[i], 2, cv::Scalar(0, 10, 200),-1);
// 	    }
// 	    
// // 	     vector<vector<cv::Point> > tmphulls = vector<vector<cv::Point> >(1,FieldFinder.fieldConvexHullPoints);
// //           drawContours(skeletonPixels, tmphulls, -1,  cv::Scalar(200, 0, 50), 2, 8);
// 	    
// 	      sensor_msgs::Image img_out1;
// 	      img_out1.header = CameraFrame.header;
// 	      img_out1.height = siY;
// 	      img_out1.width = siX;
// 	      img_out1.step = 3*siX;
// // 	      img_out1.is_bigendian = img->is_bigendian;
// 	      img_out1.encoding = std::string("bgr8");
// 	      img_out1.data.assign(skeletonPixels.datastart, skeletonPixels.dataend);
// 	      image_pub_1.publish(img_out1);
	  
	  	cv::Mat skeletonPixels(cv::Size(W,H),CV_8UC3,cv::Scalar(150, 150, 150));

		//draw detected points
		for ( unsigned int i = 0; i < LineFinder.detectedPoins.size() ; i++ ) { 
		    cv::circle(skeletonPixels, LineFinder.detectedPoins[i], 2, cv::Scalar(0, 10, 200),-1);
		}
		
    // 	     vector<vector<cv::Point> > tmphulls = vector<vector<cv::Point> >(1,FieldFinder.fieldConvexHullPoints);
    //           drawContours(skeletonPixels, tmphulls, -1,  cv::Scalar(200, 0, 50), 2, 8);
		
		  sensor_msgs::Image img_out1;
		  img_out1.header = CameraFrame.header;
		  img_out1.height = H;
		  img_out1.width = W;
		  img_out1.step = 3*W;
    // 	      img_out1.is_bigendian = img->is_bigendian;
		  img_out1.encoding = std::string("bgr8");
		  img_out1.data.assign(skeletonPixels.datastart, skeletonPixels.dataend);
		  image_pub_1.publish(img_out1);
	  
	  
	}
	
	
	//         //showNodeGraph
        if(params.debug.showNodeGraph->get()){
	  
	  	cv::Mat nodeGraph(cv::Size(W,H),CV_8UC3,cv::Scalar(150, 150, 150));
		LinearGraph_Buffer::iterator it;
		int j=-1;
                for ( it = NodeFinder.m_LinearGraph_Buffer->begin( ); it != NodeFinder.m_LinearGraph_Buffer->end( ); it++ ) { // [1]
		  ++j;
			       if(it->sumOfLength !=0 ){
			  
			      vector<int> reachableNodes = it->reachableNodeIds;
			      vector<Line> Lines = it->Lines;

			      cv::Scalar color = NodeFinder.colorPool[ j%(NodeFinder.colorPool.size())];
			      
// 			      for(unsigned int k =0; k<reachableNodes.size(); ++k){
// 				  cv::Point p(( * m_NodeBuffer_tmp )[ reachableNodes[k] ].f_x_pos, ( * m_NodeBuffer_tmp )[ reachableNodes[k] ].f_y_pos); 
// 				  cv::circle(nodeGraph, p, 3, color,-1);
// 				  std::ostringstream ss;
// 				  ss<<reachableNodes[k];
// 				  cv::putText(nodeGraph,ss.str(), cv::Point(p.x-20,p.y-20),cv::FONT_HERSHEY_TRIPLEX,0.3,cv::Scalar(0,0,0),1);
// 			      } 
			      
			      
			      for(unsigned int k = 0; k<Lines.size();k++){
				  cv::Point p1(Lines[k].s[0], Lines[k].s[1] ); 
				  cv::Point p2(Lines[k].e[0], Lines[k].e[1] ); 
				  cv::line( nodeGraph, p1, p2, color, 2, 0 );
				  cv::circle(nodeGraph, p1, 4, color,-1);
				  cv::circle(nodeGraph, p2, 4, color,-1);
			      } 
			  }
			  
			
		      } 

		  sensor_msgs::Image img_out2;
		  img_out2.header = CameraFrame.header;
		  img_out2.height = H;
		  img_out2.width = W;
		  img_out2.step = 3*W;
    // 	      img_out2.is_bigendian = img->is_bigendian;
		  img_out2.encoding = std::string("bgr8");
		  img_out2.data.assign(nodeGraph.datastart, nodeGraph.dataend);
		  image_pub_2.publish(img_out2);
	  
	  
	}
	
	

	
// 	//HoughLines
	if(params.debug.showHoughLines->get()){
	    cv::Mat houghLines_(siY,siX,CV_8UC3,cv::Scalar(150, 150, 150));
	    std::ostringstream ss;
            ss <<"Line Num: "<< LineFinder.LinesOnImg.size();
		cv::putText(houghLines_,ss.str(), cv::Point(100, 200),
						   cv::FONT_HERSHEY_TRIPLEX,3,cv::Scalar(0,0,0),2);
	    for(unsigned int i=0; i <LineFinder.LinesOnImg.size(); ++i ){
    
		float r =  rand()*255; 
		float g =  rand()*255;
		float b =  rand()*255;
		cv::Point p1=  cv::Point( LineFinder.LinesOnImg[i].s[0],LineFinder.LinesOnImg[i].s[1] );
		cv::Point p2=  cv::Point( LineFinder.LinesOnImg[i].e[0],LineFinder.LinesOnImg[i].e[1] );
		cv::line( houghLines_, p1, p2, cv::Scalar( r, g, b), 5, 0 );
	      
	    }
	    
	    sensor_msgs::Image img_out3;
	    img_out3.header =CameraFrame.header;
	    img_out3.height = siY;
	    img_out3.width = siX;
	    img_out3.step = 3*siX;
// 	    img_out3.is_bigendian = img->is_bigendian;
	    img_out3.encoding = std::string("bgr8");
	    img_out3.data.assign(houghLines_.datastart, houghLines_.dataend);
	    image_pub_3.publish(img_out3);
	}
	
	//showMergedLines
	if(params.debug.showMergedLines->get()){
	    cv::Mat MergedLines(siY,siX,CV_8UC3,cv::Scalar(150, 150, 150));
	    
	    std::ostringstream ss1,ss2;
            ss1 <<"Line Num: "<< LineFinder.LinesOnImg_After_Merged.size();
	    ss2 <<"Meas Conf: "<< LineFinder.MeasConf;
	    cv::putText(MergedLines,ss1.str(), cv::Point(100, 200),
						   cv::FONT_HERSHEY_TRIPLEX,3,cv::Scalar(0,0,0),2);
	    cv::putText(MergedLines,ss2.str(), cv::Point(100, 300),
					    cv::FONT_HERSHEY_TRIPLEX,3,cv::Scalar(0,0,0),2);
	  
	    for(unsigned int i=0; i <LineFinder.LinesOnImg_After_Merged.size(); ++i ){
    
		float r =  rand()*255; 
		float g =  rand()*255;
		float b =  rand()*255;
		cv::Point p1=  cv::Point( LineFinder.LinesOnImg_After_Merged[i].s[0],LineFinder.LinesOnImg_After_Merged[i].s[1] );
		cv::Point p2=  cv::Point( LineFinder.LinesOnImg_After_Merged[i].e[0],LineFinder.LinesOnImg_After_Merged[i].e[1] );
		cv::line( MergedLines, p1, p2, cv::Scalar( r, g, b), 5, 0 );
	    }
	    
	    sensor_msgs::Image img_out4;
	    img_out4.header =CameraFrame.header;
	    img_out4.height = siY;
	    img_out4.width = siX;
	    img_out4.step = 3*siX;
// 	    img_out4.is_bigendian = img->is_bigendian;
	    img_out4.encoding = std::string("bgr8");
	    img_out4.data.assign(MergedLines.datastart, MergedLines.dataend);
	    image_pub_4.publish(img_out4);      
	}
	

        
        
	//matching result
	if(params.debug.showAssociateLines->get()){
	    //mark model points and inlier model points on rviz
	    visualization_msgs::MarkerArray associateLines;  
	    
		visualization_msgs::Marker m4, m5;
		m4.header.frame_id = "world";
		m4.header.stamp = CameraFrame.header.stamp; 
		m4.action = visualization_msgs::Marker::ADD;
	        m4.pose.orientation.x = 0;m4.pose.orientation.y = 0;m4.pose.orientation.z = 0; m4.pose.orientation.w= 1.0;
		m4.ns = "lines";
		m4.id = 4;
		m4.pose.position.x=0;m4.pose.position.y=0;m4.pose.position.z=0;
		m4.type = visualization_msgs::Marker::LINE_LIST;
		m4.scale.x = 0.05;;
		m4.color.a = 1.0;  m4.color.r = 0.2; m4.color.g = 0.2; m4.color.b = 0.2;
		m4.points.clear();


		for( size_t i = 0; i <  lineMatcher.AssociationLines.size(); ++i ) {
		  geometry_msgs::Point p;
		  p.x = lineMatcher.AssociationLines[i].s_w[0];
		  p.y = lineMatcher.AssociationLines[i].s_w[1];
		  p.z = lineMatcher.AssociationLines[i].s_w[2];
		  m4.points.push_back(p);

		  p.x = lineMatcher.AssociationLines[i].e_w[0];
		  p.y = lineMatcher.AssociationLines[i].e_w[1];
		  p.z = lineMatcher.AssociationLines[i].e_w[2];
		  m4.points.push_back(p);  

	          }
		
		
		m5 = m4;
		m5.ns = "MergedLines";
		m5.id = 5;
		m5.scale.x = 0.05; 
		m5.color.a = 1.0;  m5.color.r = 1.0;m5.color.g = 0.2;m5.color.b = 1.0;
		m5.points.clear();

		for( size_t i = 0; i <  lineMatcher.m_LineBuffer_After_Merge.size(); ++i ) {
      
		    geometry_msgs::Point p;
		    p.x = lineMatcher.m_LineBuffer_After_Merge[i].s_w[0];
		    p.y = lineMatcher.m_LineBuffer_After_Merge[i].s_w[1];
		    p.z = lineMatcher.m_LineBuffer_After_Merge[i].s_w[2];
		    m5.points.push_back(p);

		    p.x = lineMatcher.m_LineBuffer_After_Merge[i].e_w[0];
		    p.y = lineMatcher.m_LineBuffer_After_Merge[i].e_w[1];
		    p.z = lineMatcher.m_LineBuffer_After_Merge[i].e_w[2];
		    m5.points.push_back(p);  

		}
		
		associateLines.markers.push_back( m4 );
		associateLines.markers.push_back( m5 );
	  
	  
	     associateLines_pub.publish( associateLines );
	}
        
        
        
        
        //correspondence
        if(params.debug.showCorrespondence->get()){
	    cv::Mat correspondence(cv::Size(siX,siY),CV_8UC3,cv::Scalar(150, 150, 150));
	    //draw detected points
	    for ( unsigned int i = 0; i < NodeFinder.undistortedNodeSamplePoins.size() ; i++ ) { 
		cv::circle(correspondence,NodeFinder.undistortedNodeSamplePoins[i], 4, cv::Scalar(0, 10, 200),-1);
	    }
	    //draw model projected points (distored)
	    for ( unsigned int i = 0; i <CameraFrame.forw_Proj.ModelPointsInImg.size() ; i++ ) { 
		cv::circle(correspondence, CameraFrame.forw_Proj.ModelPointsInImg[i], 4, cv::Scalar(0,250, 255),-1);
	    }

    //         //draw inliers and associate lines
	    for ( int i = 0; i <pointMatcher.inlierNum; i++ ) { 
		//images detected points
		cv::circle(correspondence, cv::Point(pointMatcher.correspondences[i*8 + 2], pointMatcher.correspondences[i*8 + 3]), 
					  4, cv::Scalar(0, 250, 0),-1);
		//projected model points
		cv::circle(correspondence, cv::Point(pointMatcher.correspondences[i*8 + 6], pointMatcher.correspondences[i*8 + 7]), 
					      5, cv::Scalar(100, 100, 200),2);
		// draw lines between input points to transformed points
		cv::line( correspondence, cv::Point(pointMatcher.correspondences[i*8 + 0], pointMatcher.correspondences[i*8 + 1] ),
					      cv::Point(pointMatcher.correspondences[i*8 + 2], pointMatcher.correspondences[i*8 + 3]), 
					      cv::Scalar(200,85,20), 1, 8 );
		// draw lines between transformed points to closest model points
		cv::line( correspondence, cv::Point(pointMatcher.correspondences[i*8 + 2], pointMatcher.correspondences[i*8 + 3] ),
					      cv::Point(pointMatcher.correspondences[i*8 + 6], pointMatcher.correspondences[i*8 + 7]), 
					      cv::Scalar(24,85,200), 1, 8 );
	    }
    
	 
	      std::ostringstream ss0, ss1,ss2;
// 	      ss0<<"Dist confidence:    "<< dist_cnf;
// 	      ss1<<"Num confidence:     "<< num_cnf;
// 	      ss2<<"Sum of confidence:  "<< num_cnf;
	    
// 	      cv::putText(correspondence,ss0.str(), cv::Point(50,100),cv::FONT_HERSHEY_TRIPLEX,2,cv::Scalar(0,0,0),2);
//               cv::putText(correspondence,ss1.str(), cv::Point(50,150),cv::FONT_HERSHEY_TRIPLEX,2,cv::Scalar(0,0,0),2);
// 	      cv::putText(correspondence,ss2.str(), cv::Point(50,200),cv::FONT_HERSHEY_TRIPLEX,2,cv::Scalar(0,0,0),2);
	      

	    for(unsigned int i=0; i <CameraFrame.forw_Proj.Field_Lines_Img.size(); ++i ){
	        
		cv::Point p1=  cv::Point( CameraFrame.forw_Proj.Field_Lines_Img[i].s[0],CameraFrame.forw_Proj.Field_Lines_Img[i].s[1] );
		cv::Point p2=  cv::Point( CameraFrame.forw_Proj.Field_Lines_Img[i].e[0],CameraFrame.forw_Proj.Field_Lines_Img[i].e[1] );
		cv::line( correspondence, p1, p2, cv::Scalar(0,250, 255), 2, 0 );
	    }

	  
	    sensor_msgs::Image img_out6;
	    img_out6.header = CameraFrame.header;;
	    img_out6.height = siY;
	    img_out6.width = siX;
	    img_out6.step = 3*siX;
// 	    img_out6.is_bigendian = img->is_bigendian;
	    img_out6.encoding = std::string("bgr8");
	    img_out6.data.assign(correspondence.datastart, correspondence.dataend);
	    image_pub_6.publish(img_out6);
	}
	

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
    // loop stops if the node stops, e.g. by getting a kill signal 
    while (soccer_vision.nh.ok())
    {
      
	  cpu_timer timer;
	  std_msgs::Int64 time_msg;


// 	  if(soccer_vision.CameraFrame.imagecounter >0 ){
	      soccer_vision.update();
	   
// 	  }
// 	  else{
// 	    ROS_INFO("No Input Image Message.");
// 	  }
	      
	  fpsData = (0.9 * fpsData) + (0.1 * (1000000000l / timer.elapsed().wall));
	  time_msg.data = fpsData;
	  cout<< fpsData<<endl;
	 
	  
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