#include<visual_tracking/CalculatePose/lostTrackRecover.h>









LostTrackerRecover::LostTrackerRecover(){



}
LostTrackerRecover::~LostTrackerRecover(){




}

void LostTrackerRecover::getFieldInfo( FieldInfo &_fieldInfo ){
      fieldA= _fieldInfo.A;
      fieldB =_fieldInfo.B;
  
  
}

geometry_msgs::Pose LostTrackerRecover::SamplingRandomPose(){
      geometry_msgs::Pose p;
      p.position.x = random(-fieldA/2-0.5, fieldA/2 + 0.5);
      p.position.y = random(-fieldB/2-0.5, fieldB/2 + 0.5);
      p.position.z = random(0.5, 0.7);
      
      
      double roll = 0 ,pitch =0; double yaw;
      double yaw_min = 0, yaw_max = 2*M_PI;
      if(p.position.x > fieldA/2 - 0.5  ){
	yaw_min = 0.5*M_PI; 
	yaw_max = 1.5*M_PI;
      }
      else if(p.position.y > fieldB/2 - 0.5  ){
	yaw_min =  1.0*M_PI; 
	yaw_max =   2*M_PI;
      }

      else if(p.position.x < -fieldA/2 + 0.5 ){
	yaw_min = -0.5*M_PI; 
	yaw_max =  0.5*M_PI;
      }
      else if( p.position.y < -fieldB/2 + 0.5){
	yaw_min = 0.0*M_PI; 
	yaw_max =  1.0*M_PI;
      }
      else{
	yaw_min = 0.0*M_PI; 
	yaw_max = 2.0*M_PI;
	}
      yaw =  random(yaw_min,yaw_max);
//       yaw =  random(0, 2*M_PI );
    
      roll = random(-0.1, 0.1 );
      pitch = random(-0.1, 0.1 );

      
//       roll = m_Math::CorrectAngleRadian180(roll);
//       pitch = m_Math::CorrectAngleRadian180(pitch);
      yaw = m_Math::CorrectAngleRadian360(yaw);

      
      p.orientation = tf::createQuaternionMsgFromRollPitchYaw( roll,  pitch,  yaw);
      
     
//       if(m_Math::RadianAngleDiff(yaw, getHeading()) > M_PI/2.0){ 
// 	p.position.x = -p.position.x;
//         p.position.y = -p.position.y;
// 	
// 	double newYaw = m_Math::CorrectAngleRadian360(yaw-M_PI);
// 	p.orientation  = tf::createQuaternionMsgFromRollPitchYaw (roll, pitch, newYaw);
//       }
     
      return p;
}

geometry_msgs::Pose LostTrackerRecover::SamplingPoseAround(geometry_msgs::Pose center, float range){
      geometry_msgs::Pose p;
      float x_max= std::min(center.position.x + range,  fieldA/2.0 + 0.5);
      float x_min= std::max(center.position.x - range, -fieldA/2.0 - 0.5);
      float y_max= std::min(center.position.y + range,  fieldB/2.0 + 0.5);
      float y_min= std::max(center.position.y- range,  -fieldB/2.0 - 0.5);
      p.position.x = random(x_min, x_max);
      p.position.y = random(y_min, y_max);
      p.position.z = random(0.5, 0.58);
      
      
      double roll = 0 ,pitch =0; double yaw;
      double yaw_min = 0, yaw_max = 2*M_PI;
      if(p.position.x > fieldA/2 - 0.5  ){
	yaw_min = 0.5*M_PI; 
	yaw_max = 1.5*M_PI;
      }
      else if(p.position.y > fieldB/2 - 0.5  ){
	yaw_min =  1.0*M_PI; 
	yaw_max =   2*M_PI;
      }

      else if(p.position.x < -fieldA/2 + 0.5 ){
	yaw_min = -0.5*M_PI; 
	yaw_max =  0.5*M_PI;
      }
      else if( p.position.y < -fieldB/2 + 0.5){
	yaw_min = 0.0*M_PI; 
	yaw_max =  1.0*M_PI;
      }
      else{
	yaw_min = 0.0*M_PI; 
	yaw_max = 2.0*M_PI;
	}

      
      yaw =  random(yaw_min,yaw_max);
      
/*    
      roll = random(-0.1, 0.1 );
      pitch = random(-0.1, 0.1 );*/

      
//    roll = m_Math::CorrectAngleRadian180(roll);
//    pitch = m_Math::CorrectAngleRadian180(pitch);
      yaw = m_Math::CorrectAngleRadian360(yaw);

      
      p.orientation = tf::createQuaternionMsgFromRollPitchYaw( roll,  pitch,  yaw);
      
     
//      if(m_Math::RadianAngleDiff(yaw, getHeading()) > M_PI/2.0){ 
// 	p.position.x = -p.position.x;
//      p.position.y = -p.position.y;
// 	
// 	double newYaw = m_Math::CorrectAngleRadian360(yaw-M_PI);
// 	p.orientation  = tf::createQuaternionMsgFromRollPitchYaw (roll, pitch, newYaw);
//      }
     
      return p;
}

