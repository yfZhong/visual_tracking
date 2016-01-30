// #include "ForwardProjection.h"
#include <visual_tracking/Projection/ForwardProjection.h>

#include <stdlib.h>     /* srand, rand */


using namespace vision;

ForwardProjection::ForwardProjection():listener(ros::Duration(10)){
  robotPoseInit();
  
  seq = 1;
  cur_time_stamp = ros::Time::now();
  
  fx = distortionModel.cameraMatrix.at<double>(0, 0);
  fy = distortionModel.cameraMatrix.at<double>(1, 1);
  cx = distortionModel.cameraMatrix.at<double>(0, 2); 
  cy = distortionModel.cameraMatrix.at<double>(1, 2); 
  
  W = params.camera.width->get();
  H = params.camera.height->get();
  siX = params.camera.widthUnDistortion->get();
  siY = params.camera.heightUnDistortion->get();
  offsetx = (siX - W) / 2.;
  offsety = (siY - H) / 2.;
  
  
  
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
  generateFieldPoints(); //line points in world cord
  m_ModelLine_Buffer  = new ModelLine_Buffer;
  
//   points_pub_cam = node.advertise< geometry_msgs::PointStamped >( "line_points_cam_cord", 1 );
  getTFSuccessfully = 0;

}


 void ForwardProjection::generateFieldLines(){
      Field_Lines.clear();
      int id = 1;
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
      
      
//       s_w = Vec3f(fieldInfo.A/2 - fieldInfo.G + 0.01, 0 , 0), e_w = Vec3f(fieldInfo.A/2 - fieldInfo.G- 0.01, 0 , 0);
//       Field_Lines.push_back(Line_w(id ++, s_w, e_w));
//       
//       s_w = Vec3f(fieldInfo.A/2 - fieldInfo.G , + 0.01 , 0), e_w = Vec3f(fieldInfo.A/2 - fieldInfo.G , - 0.01 , 0);
//       Field_Lines.push_back(Line_w(id ++, s_w, e_w));
      
}


void ForwardProjection::generateFieldPoints(){
      //Points on Field Lines
      SamplePointDist_WorldCord = params.icp.SamplePointDist_WorldCord->get();   
      m_Math::SamplePointsOnLines(Field_Lines, SamplePointDist_WorldCord, PointsInWorldCord);
      
      //Points on Center Circle
      int id =-1;
      for(int i=0 ; i <4*10; i++){// center circle
	  double theta = i * 2* M_PI /(4*10.0);
// 	  cout<<" id "<<id<<endl;
	  PointsInWorldCord.push_back( make_pair( tf::Vector3( fieldInfo.H/2.0  *cos(theta) , fieldInfo.H/2.0  *sin(theta), 0 ), id));
	  id--;
      } 
      
//         PointsInWorldCord.push_back( make_pair( tf::Vector3( fieldInfo.A/2.0 - fieldInfo.G , 0, 0 ), 0));
//         PointsInWorldCord.push_back( make_pair( tf::Vector3(-fieldInfo.A/2.0 + fieldInfo.G , 0, 0 ), 0));
      
  
}

void ForwardProjection::robotPoseInit(){
  
  rbPose.position.x=0;rbPose.position.y=0;rbPose.position.z=0.6;
  rbPose.orientation.x= 0;rbPose.orientation.y= 0;rbPose.orientation.z= 0;rbPose.orientation.w= 1.0;
  
}
 void ForwardProjection::setCurTime(ros::Time t){
    if(t<cur_time_stamp){timeToshift = 0;}
    else{timeToshift = params.projection.timeToShift->get();}
    cur_time_stamp = t;
   
}

void ForwardProjection::setTfEgorot2Cam(){
   tf::StampedTransform tfmsg;
         try{
// 		  if(listener.waitForTransform("camera_optical","ego_rot",  cur_time_stamp- ros::Duration(timeToshift), ros::Duration(0.5))){
// 		    listener.lookupTransform("camera_optical","ego_rot",    cur_time_stamp- ros::Duration(timeToshift), tfmsg);
// 		  }
// 		  else{
// 		      geometry_msgs::TransformStamped msg;
// 		      msg.transform.translation.x = -0.0320715;
// 		      msg.transform.translation.y =  0.2620675;
// 		      msg.transform.translation.z = -0.0653293;
// 		      msg.transform.rotation.x = 0.481647;
// 		      msg.transform.rotation.y = -0.471644;
// 		      msg.transform.rotation.z = 0.526234;
// 		      msg.transform.rotation.w = 0.518311;
// 		      tf::transformStampedMsgToTF(msg,tfmsg);
// 		      cout<<"No tf from ego_rot to camera_optical."<<endl;
// 		}
		listener.waitForTransform("camera_optical","ego_rot",  cur_time_stamp- ros::Duration(timeToshift), ros::Duration(0.5));
		listener.lookupTransform("camera_optical","ego_rot",    cur_time_stamp- ros::Duration(timeToshift), tfmsg);
		
		tf_Egorot2Cam =  static_cast<const tf::Transform&>(tfmsg);
		getTFSuccessfully = 1;
	   
	}
      catch(tf::TransformException& ex){
	  ROS_ERROR("Received an exception trying to transform a pose : %s", ex.what());
	  getTFSuccessfully = 0;
      }   
}
void ForwardProjection::onInit(ros::Time t ){
    setCurTime(t);
    setTfEgorot2Cam();
}

void ForwardProjection::setRobotPose(geometry_msgs::Pose p){
    rbPose = p;
    tf::Transform tf_world2Egorot;
    tf::poseMsgToTF (rbPose, tf_world2Egorot);
  
    if(getTFSuccessfully == 1){
	tf_World2Cam =  tf_Egorot2Cam*tf_world2Egorot.inverse();
	
	
	rotation_World2Cam = tf_World2Cam.getBasis();
	translation_World2Cam =  tf_World2Cam.getOrigin();
	
	rotation_Cam2World =  tf_World2Cam.inverse().getBasis();
	translation_Cam2World =  tf_World2Cam.inverse().getOrigin();
    //     cout<<"translation_Cam2World " <<endl;
    //     cout<<translation_Cam2World.x()<< "  " <<translation_Cam2World.y()<< "  "<<translation_Cam2World.z()<< "  " <<endl;
	
	tf::poseTFToMsg (tf_World2Cam.inverse(), cameraPose);
    }
    
}

void ForwardProjection::tfFieldPoints2CamCord(){ 

      PointsInCamCord.clear();
      for( size_t i = 0; i < PointsInWorldCord.size(); ++i ) {
           tf::Vector3 v = PointsInWorldCord[i].first;
	   tf::Vector3 v2( rotation_World2Cam[0].dot(v) + translation_World2Cam.x(), 
			   rotation_World2Cam[1].dot(v) + translation_World2Cam.y(), 
			   rotation_World2Cam[2].dot(v) + translation_World2Cam.z());
	 
	   PointsInCamCord.push_back( make_pair(v2,PointsInWorldCord[i].second));

      }
}

void ForwardProjection::projectFieldPoints2Image(){
  
  ModelPointsInImg.clear();
  ModelPointsInImgWithId.clear();
//   ModelPointsInWorldCord.clear();

  for( size_t i = 0; i < PointsInCamCord.size(); ++i ) {
       
        Eigen:: Matrix<float, 3, 1> pointC;  //point in camera cordinate
       pointC << PointsInCamCord[i].first.x(), PointsInCamCord[i].first.y(), PointsInCamCord[i].first.z();

      if(pointC(2,0) >0){ //only check those points in front of the camera

	  pointC << pointC(0,0)/pointC(2,0),pointC(1,0)/pointC(2,0), 1;
// 	  
	  Eigen:: Matrix<float, 3, 1> pixelC;  // point in pixel cordinate
// 	  pixelC = C * pointC;
          pixelC(0,0) = pointC(0,0) * fx + cx + offsetx ;
	  pixelC(1,0) = pointC(1,0) * fy + cy + offsety ;
	  pixelC(2,0) =1;
	  
	  if(pixelC(0,0)>= 0 && pixelC(0,0) < siX && pixelC(1,0)>= 0  && pixelC(1,0) < siY 
	    && distortionModel.unDistortionModel.at<uchar>(pixelC(1,0) ,pixelC(0,0)) == 1 ){// those points would be projected inside the image plane

	      ModelPointsInImg.push_back(cv::Point(pixelC(0,0), pixelC(1,0)));
	      ModelPointsInImgWithId.push_back(make_pair(cv::Point(pixelC(0,0), pixelC(1,0)),PointsInWorldCord[i].second));
	      
	      
	      
	      
// 	      cout<<ModelPointsInImg.back().x <<"   "<<ModelPointsInImg.back().y<<endl;
	      
// 	     
// 	      
// 	      cout<< W.x<<"  "<< PointsInWorldCord[i].first.x()<<"     "<<
// 	             W.y<<"  " <<PointsInWorldCord[i].first.y()<<"     "<<
// 	             W.z<<"  "<< PointsInWorldCord[i].first.z()<<"     "<<endl;
	      
// 	      ModelPointsInWorldCord.push_back(cv::Point3f(PointsInWorldCord[i].first.x(), 
// 							   PointsInWorldCord[i].first.y(), 
// 						           PointsInWorldCord[i].first.z()));
	      
	  }
      }
      
  }

}

void ForwardProjection::generateFieldLineInImage(){
     Field_Lines_Img.clear();
     
     int id;
     Vec2i s,e;
    
     for(unsigned int i =0; i<ModelPointsInImgWithId.size(); ++i ){
	  s = Vec2i( ModelPointsInImgWithId[i].first.x, ModelPointsInImgWithId[i].first.y);
	  id = ModelPointsInImgWithId[i].second;
	  
	  if(id == -1 && i<ModelPointsInImgWithId.size()-1 && (ModelPointsInImgWithId[i+1].second == -1) ){ 
	       e = Vec2i( ModelPointsInImgWithId[i+1].first.x, ModelPointsInImgWithId[i+1].first.y);
	       Field_Lines_Img.push_back( Line(id, s, e));
	       continue;
	  }
/*	  
	  if(id == -1 ){break;}*/
	  
	  for(unsigned int j = i + 1; j<ModelPointsInImgWithId.size(); ++j ){
	      if (id == ModelPointsInImgWithId[j].second) {continue;}
	      else if(j>i+1){
		e = Vec2i( ModelPointsInImgWithId[j-1].first.x, ModelPointsInImgWithId[j-1].first.y);
		Field_Lines_Img.push_back( Line(id, s, e));
		i= j; 
		break;}
	      else { i= j; 
		    break; }
	  }
     }


}



void ForwardProjection::GetModelLineComps() {
    if(params.icp.SamplePointDist_WorldCord->get() != SamplePointDist_WorldCord)
      { generateFieldPoints();}
	        
    tfFieldPoints2CamCord();
    projectFieldPoints2Image();
    
    m_ModelLine_Buffer->clear();
   
     
     int id;
     Vec2i s, e;
//    cout<<endl;
//    for(unsigned int i =0; i<ModelPointsInImgWithId.size(); ++i ){
//       id = ModelPointsInImgWithId[i].second;
//       cout<<id<<"   ";
//      
//      
//      
//   }
//   cout<<endl;
//   
     
     for(unsigned int i =0; i<ModelPointsInImgWithId.size(); ++i ){
	 
	  id = ModelPointsInImgWithId[i].second;
          s = Vec2i( ModelPointsInImgWithId[i].first.x, ModelPointsInImgWithId[i].first.y);
	  
	  if(id < 0){
	     
	      ModelLineElement circle;
	      circle.sumOfLengthUndistorted =0;
	      circle.id = -1;
	
	      circle.UndistortedPoints.push_back(s);
	      int id_previous=id;

	      for(unsigned int j = i + 1; j<ModelPointsInImgWithId.size(); ++j ){
		 
		  int id2 = ModelPointsInImgWithId[j].second;

		  Vec2i s0( ModelPointsInImgWithId[j-1].first.x, ModelPointsInImgWithId[j-1].first.y);
		  Vec2i e0( ModelPointsInImgWithId[j].first.x,   ModelPointsInImgWithId[j].first.y);

		  circle.UndistortedPoints.push_back(e0);
		  
		  if(  abs(id2 - id_previous)==1 ){
		      circle.UndistortedLines.push_back(Line(id,s0, e0));
		      circle.UndistortedPoints.push_back(e0);
		      circle.sumOfLengthUndistorted +=circle.UndistortedLines.back().len;
		  }
		  
		  id_previous = id2;
		  
		  if( j==ModelPointsInImgWithId.size()-1 && id2==-40 && id ==-1 ){ 
		      circle.UndistortedLines.push_back(Line(id,e0, s));
		      circle.sumOfLengthUndistorted +=circle.UndistortedLines.back().len;
    
		  }
		  
	      }
	      
	      if(circle.UndistortedLines.size()>0 ){
		    m_ModelLine_Buffer->push_back(circle);

	      }
	     
	     break;

	      
	  } 
	  else{
	      ModelLineElement fieldLIne;
	      fieldLIne.sumOfLengthUndistorted =0;
	      Vec2i s0 = Vec2i( ModelPointsInImgWithId[i].first.x, ModelPointsInImgWithId[i].first.y);
	      Vec2i e0;
	      fieldLIne.UndistortedPoints.push_back(s0);
	      for(unsigned int j = i + 1; j<ModelPointsInImgWithId.size(); ++j ){
		  int id2 = ModelPointsInImgWithId[j].second;
		  
		  
		  if(id2 != id ){
		    
		    e0 = Vec2i( ModelPointsInImgWithId[j-1].first.x,   ModelPointsInImgWithId[j-1].first.y);
		    i =j-1;  
		    break;}
		    
		   if(j== ModelPointsInImgWithId.size()-1  ){ 
		         e0 = Vec2i( ModelPointsInImgWithId[j].first.x,   ModelPointsInImgWithId[j].first.y);}
// 		    
// 		    s = Vec2i( ModelPointsInImgWithId[j-1].first.x, ModelPointsInImgWithId[j-1].first.y);
// 		    e = Vec2i( ModelPointsInImgWithId[j].first.x,   ModelPointsInImgWithId[j].first.y);
// 
// 		    fieldLIne.UndistortedLines.push_back(Line(id, s, e));
// 		    fieldLIne.UndistortedPoints.push_back(e);
// 		    fieldLIne.sumOfLengthUndistorted +=fieldLIne.UndistortedLines.back().len;
		    

	      }
// 	      
	      
// 	      if(fieldLIne.sumOfLengthUndistorted >0 ){
		
			fieldLIne.id = id;
			if( e0[0]==0&& e0[1]==0 ){continue;  }
			if(!(s0[0]==e0[0]&& s0[1]==e0[1])){
			    fieldLIne.LongLine = Line(id, s0, e0);
			    
			    vector<cv::Point>  PointsOnLine;
			    m_Math::SamplePointsOnLine(fieldLIne.LongLine, 40.0, PointsOnLine);
			    
			    for(int p =0; p<PointsOnLine.size()-1; p++ ){
			      Vec2i  ss(PointsOnLine[p].x, PointsOnLine[p].y);
			      Vec2i  ee(PointsOnLine[p+1].x, PointsOnLine[p+1].y);
			      fieldLIne.UndistortedLines.push_back(Line(id, ss, ee));
			      fieldLIne.UndistortedPoints.push_back(ee);
			      fieldLIne.sumOfLengthUndistorted +=fieldLIne.UndistortedLines.back().len;
			      
			    }
	
			    fieldLIne.undistortedAngle = fieldLIne.LongLine.ang;
			    m_ModelLine_Buffer->push_back(fieldLIne);

			}
// 	      }
	  }
	  
	  
	  
     }

  
}




// Main loop 
void ForwardProjection::mainLoop() {
  // SamplePointDistance is changed. generate field points again
  if(params.icp.SamplePointDist_WorldCord->get() != SamplePointDist_WorldCord)
      { generateFieldPoints();}
	        
    tfFieldPoints2CamCord();
    projectFieldPoints2Image();
    generateFieldLineInImage();
}


bool ForwardProjection::projectWordPoint2Img(cv::Point3f pIn, cv::Point & pOut){

        tf::Vector3 vIn(pIn.x, pIn.y,pIn.z);
        tf::Vector3 v2( rotation_World2Cam[0].dot(vIn) + translation_World2Cam.x(), 
		        rotation_World2Cam[1].dot(vIn) + translation_World2Cam.y(), 
		        rotation_World2Cam[2].dot(vIn) + translation_World2Cam.z());
        if( v2.z() != 0 ){

	    pOut.x = v2.x()/v2.z() * fx + cx + offsetx ;
	    pOut.y = v2.y()/v2.z() * fy + cy + offsety ;
	    
	    return true;
	}
	else{return false;}
}


void ForwardProjection::projectImgPoint2WorldCord(cv::Point p, cv::Point3f& pOut){
      
      tf::Vector3 pointInCamCordNorm((p.x - (cx + offsetx) ) / fx,
                                     (p.y - (cy + offsety) ) / fy,
				                      1    );
      
      float z = -translation_Cam2World.z()/(rotation_Cam2World[2].dot(pointInCamCordNorm));
      tf::Vector3 pointInCamCord = tf::Vector3( z * pointInCamCordNorm.x(),
						z * pointInCamCordNorm.y(),
						            z);
      
       pOut = cv::Point3f( rotation_Cam2World[0].dot(pointInCamCord) + translation_Cam2World.x(), 
			   rotation_Cam2World[1].dot(pointInCamCord) + translation_Cam2World.y(), 
			                          0);

}




void ForwardProjection::TfRobotPose2CamPose(geometry_msgs::PoseStamped &robotPose, geometry_msgs::PoseStamped &cameraPose){
       cameraPose.header = robotPose.header;
       
       setRobotPose(robotPose.pose);
//        geometry_msgs::Transform msg;
//        tf::transformTFToMsg (tf_World2Cam.inverse(), msg);
        tf::poseTFToMsg (tf_World2Cam.inverse(), cameraPose.pose);
//        cameraPose.pose.position.x = msg.translation.x;
//        cameraPose.pose.position.y = msg.translation.y;
//        cameraPose.pose.position.z = msg.translation.z;
//        cameraPose.pose.orientation = msg.rotation;
   
}

void ForwardProjection::TfCamPose2RobotPose(geometry_msgs::PoseStamped &cameraPose, geometry_msgs::PoseStamped &robotPose){
       robotPose.header=cameraPose.header;
       
       tf::Transform tfmsg;
       tf::poseMsgToTF( cameraPose.pose, tfmsg );
       
       tf::Transform tf_world2Robot = tfmsg*tf_Egorot2Cam;
       tf::poseTFToMsg (tf_world2Robot, robotPose.pose);
}
