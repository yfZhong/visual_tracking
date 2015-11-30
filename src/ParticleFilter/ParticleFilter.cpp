
// #include "ParticleFilter.h"

#include <visual_tracking/ParticleFilter/ParticleFilter.h>

using namespace vision;


ParticleFilter::ParticleFilter(){
     cur_time_stamp = ros::Time::now();
     particleNum = 0;
     SeqNum=0;
     robotPoseInit();
     
     m_rng = allocate_rng();
     reSamplingAccordingWeights = true;
     InitSampling();
     randomSampleNum =0;
     
     heading_sub_robotstate = nodeHandle.subscribe("/robotmodel/robot_heading", 1, &ParticleFilter::handleHeadingData, this);
     headingOffset = params.particleFilter.HeadingOffset->get() ;
     MeasConf = 1.0;
}

void ParticleFilter::robotPoseInit(){
      robotPose.header.frame_id = "world";
      robotPose.header.seq = SeqNum;
      robotPose.header.stamp = cur_time_stamp;
      robotPose.pose.position.x=0;robotPose.pose.position.y=0;robotPose.pose.position.z=0.6;
      robotPose.pose.orientation.x= 0;robotPose.pose.orientation.y= 0;robotPose.pose.orientation.z= 0;robotPose.pose.orientation.w= 1.0;
      cameraPose = robotPose;
      cameraPose.pose.position.z = 0.9;
}

ParticleFilter::~ParticleFilter(){
  gsl_rng_free(m_rng);
}


gsl_rng* ParticleFilter::allocate_rng()
{
	gsl_rng_env_setup();
	const gsl_rng_type* T = gsl_rng_default;
	gsl_rng* rng = gsl_rng_alloc(T);
	gsl_rng_set(rng, (unsigned long)time(NULL));

	return rng;
}

void ParticleFilter::setdetectedLines(std::vector< Line > LinesOnImg_ , float Conf){
  LinesOnImg = LinesOnImg_;
  MeasConf = Conf;
}

void ParticleFilter::setCurTime(ros::Time t){
    if(t<cur_time_stamp){
      //set particleNum to be 0 so that the the filter will resample.
      particleNum = 0;}
    cur_time_stamp = t;
  
  
}




void ParticleFilter::InitSampling(){
  
      params.particleFilter.RandomSamplePct->set(0.2);
  
      particleNum = params.particleFilter.ParticleNum->get();

      particles_ego_rot.header.frame_id = "world";
      particles_ego_rot.header.seq = SeqNum++;
      particles_ego_rot.header.stamp = cur_time_stamp;
      particles_ego_rot.poses.clear();
      weights.clear();
      
      geometry_msgs::Pose p;
//       srand((unsigned)time(NULL));
      
//       double roll = 0 ,pitch =0; double yaw;
      
// 	p.position.x = 0;
// 	p.position.y = 0;
// 	p.position.z = 0.6;
// 	yaw = 0;
// 	p.orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll,  pitch,  yaw);
// 	particles_ego_rot.poses.push_back(p);
// 	weights.push_back(1.0/particleNum);
     while(int(particles_ego_rot.poses.size()) < particleNum){
	 p = SamplingRandomPose();
	 particles_ego_rot.poses.push_back(p);
	 weights.push_back(1.0/particleNum);
// 	 p.orientation.z = -p.orientation.z;
// 	 particles_ego_rot.poses.push_back(p);
// 	 weights.push_back(1.0/particleNum);

      }
          
}
geometry_msgs::Pose ParticleFilter::SamplingRandomPose(){
      geometry_msgs::Pose p;
      p.position.x = random(-lineMatcher.fieldInfo.A/2-0.5, lineMatcher.fieldInfo.A/2 + 0.5);
      p.position.y = random(-lineMatcher.fieldInfo.B/2-0.5, lineMatcher.fieldInfo.B/2 + 0.5);
      p.position.z = random(0.5, 0.7);
      
      
      double roll = 0 ,pitch =0; double yaw;
      double yaw_min = 0, yaw_max = 2*M_PI;
      if(p.position.x > lineMatcher.fieldInfo.A/2 - 0.5  ){
	yaw_min = 0.5*M_PI; 
	yaw_max = 1.5*M_PI;
      }
      else if(p.position.y > lineMatcher.fieldInfo.B/2 - 0.5  ){
	yaw_min =  1.0*M_PI; 
	yaw_max =   2*M_PI;
      }

      else if(p.position.x < -lineMatcher.fieldInfo.A/2 + 0.5 ){
	yaw_min = -0.5*M_PI; 
	yaw_max =  0.5*M_PI;
      }
      else if( p.position.y < -lineMatcher.fieldInfo.B/2 + 0.5){
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
      
     
      if(m_Math::RadianAngleDiff(yaw, getHeading()) > M_PI/2.0){ 
	p.position.x = -p.position.x;
        p.position.y = -p.position.y;
	
	double newYaw = m_Math::CorrectAngleRadian360(yaw-M_PI);
	p.orientation  = tf::createQuaternionMsgFromRollPitchYaw (roll, pitch, newYaw);
      }
     
      return p;
}


void ParticleFilter::updateWeights(){
   
     double sum_of_weight = 0;
     std::vector<double> weights_tmp;
     weights.clear();
     
//      float SumOfLineLengthOnImage = 0;
//      for(unsigned int j = 0; j< LinesOnImg.size(); ++j){
//        SumOfLineLengthOnImage += LinesOnImg[j].len; 
//      }
     
//      if( SumOfLineLengthOnImage>params.particleFilter.MinSumOfLineLengthOnImage->get() ){
     
     lineMatcher.backw_Proj.onInit(cur_time_stamp);
     
	for ( int i = 0; i< particleNumForUpdate;++i  ){
		// ******************************************* //
		// *********  projection model init    ********//
		// ******************************************* //
	         geometry_msgs::Pose pose = particlesForUpdate[i];
	  
	        //reject the most unlikely particles
		double roll, pitch, yaw;
		//calculate roll pitch yaw from quaternion
		tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w);
		q.normalized();
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		
		if(m_Math::RadianAngleDiff(yaw, getHeading())  > M_PI/2.0  ||
		  fabs(roll) > 0.4 * 0.5*M_PI  || fabs(pitch) > 0.3* 0.5*M_PI || 
		  pose.position.z >1.0 ||  pose.position.z < 0.1 ){
		  weights_tmp.push_back( 0 );
		  sum_of_weight +=  0;  
		  continue;
		}
			
		
		lineMatcher.backw_Proj.setRobotPose( pose );

		// ************************************** //
		// *  lineMatcher for data association * // 	
		// ************************************** //
		  
		lineMatcher.run(LinesOnImg);	      
		
		//calculating the weight
		double weighted_dist = 0; //double sum_of_len=0;
	      
		for(unsigned int j=0; j < lineMatcher.m_LineBuffer_After_Merge.size(); ++j){
// 		  weighted_dist += lineMatcher.m_LineBuffer_After_Merge[j].len_w * lineMatcher.m_LineBuffer_After_Merge[j].conf;
// 		  sum_of_len+=lineMatcher.m_LineBuffer_After_Merge[j].len_w;
		  
		  weighted_dist +=  lineMatcher.m_LineBuffer_After_Merge[j].conf;
// 		  cout<< i<<" - "<<j <<": "<<lineMatcher.m_LineBuffer_After_Merge[j].conf<<endl;
		  
// 		  weighted_dist +=  sqrt(pow(pose.position.x, 2) +  pow(pose.position.y,2)) +yaw/6;
		 
		}  
		
// 		weighted_dist *= fabs(particles.poses[i].position.z-0.6);
	    
		weighted_dist = std::max( weighted_dist, 0.0000001);
		
		
// 		weights_tmp.push_back( lineMatcher.NumProjectedToDifferentLines/weighted_dist );
// 		sum_of_weight +=  lineMatcher.NumProjectedToDifferentLines/weighted_dist;
		
		
// 	        weights_tmp.push_back( lineMatcher.sumofLineLength/weighted_dist );
// 		sum_of_weight +=  lineMatcher.sumofLineLength/weighted_dist;
		
	        weights_tmp.push_back( lineMatcher.sumofLineLength * lineMatcher.NumProjectedToDifferentLines/weighted_dist );
		sum_of_weight +=  lineMatcher.sumofLineLength* lineMatcher.NumProjectedToDifferentLines/weighted_dist;
		
// 		weights_tmp.push_back( 1.0/weighted_dist );
// 		sum_of_weight +=  1.0/weighted_dist;
		
		
	}

	
// 	for ( int j = 0; j< particleNumForUpdate; ++j){
// 	      weights.push_back(1.0 /double(particleNumForUpdate));
// 	}
	//almost impossible
        if(sum_of_weight==0){ 	
	    for ( int j = 0; j< particleNumForUpdate; ++j){
	      weights.push_back(1.0 /double(particleNumForUpdate));
	    }
	}
	else{
	   
	    //normalized to [0,1] so that the sumOfWeights=1
	    for ( int j = 0; j< particleNumForUpdate; ++j){
		weights.push_back(weights_tmp[j] /sum_of_weight);
	    }
// 	    reSamplingAccordingWeights = true;
	}
	
//      }
//      else{//The detected lines are so short(Bad measurement)
//         reSamplingAccordingWeights = false;
//     
//     }
     
     
}




void ParticleFilter::ReSampling(){

	randomSampleNum = particleNumForUpdate * params.particleFilter.RandomSamplePct->get();
	int resamplingNum = particleNumForUpdate- randomSampleNum;
	
	particles_tmp.poses.clear();
	particles_tmp.header.frame_id = "world";
	particles_tmp.header.seq = SeqNum++;
	particles_tmp.header.stamp = cur_time_stamp;
	particles_tmp.poses.clear();
	geometry_msgs::Pose p;
	
	
        if(resamplingNum >0) {    
	    systematic_resampling(resamplingNum,resampleIdx);
// 	     if(resamplingNum >0) {  
		weights.clear();
		//sampling base on weights
		for(int j =0; j<resamplingNum; ++j){
		    int idx = resampleIdx[j];
		    particles_tmp.poses.push_back(particlesForUpdate[idx]);
		    weights.push_back( 1.0/double(particleNum));
		}
// 	     }
	}
	else{
	   weights.clear();
	}
	
	//random sampling
	for(int k =0; k<randomSampleNum; ++k){
	    p = SamplingRandomPose();
	    particles_tmp.poses.push_back(p);
	    weights.push_back( 1.0/double(particleNum));
	}
	
	
	//add those particle that keep particleNumKeepUnchanged
	for(int j =0; j< particleNumKeepUnchanged; ++j){
	    particles_tmp.poses.push_back(particlesKeepUnchanged[j]);
	    weights.push_back( 1.0/double(particleNum));
        }
}


void ParticleFilter::systematic_resampling(int resamplingNum, std::vector<int>& Idxlist){

  Idxlist.clear();
  
  std::vector<double> cum_weights;
  cum_weights.push_back(weights[0]);
  for(unsigned int i= 1; i< weights.size(); ++i){
    cum_weights.push_back(cum_weights[i-1] +  weights[i] );
  }
  
  if(cum_weights[weights.size() -1] <= 0 ){ 
//     cout<<" whoops ,sum of weights less than 0 "<<endl;
    resamplingNum =0;
    randomSampleNum = particleNumForUpdate; 
    return; 
    
  }
  
  double u0 = random(0.0, 1.0/double(resamplingNum) );
  int k =0;
  
  for(int j =0; j<resamplingNum; ++j){
    
      while(u0 >cum_weights[k] ){k++;}
      Idxlist.push_back(k);
      u0 += 1.0/double(resamplingNum);
  }
}

void ParticleFilter::predict(){
  
    for(int j =0; j<particleNum; ++j){
      
      geometry_msgs::Pose p =  particles_tmp.poses[j];
      // predict
      
      
      
      
      // add random values
      p.position.x += gsl_ran_gaussian(m_rng, 0.05) ;
      p.position.y += gsl_ran_gaussian(m_rng, 0.05) ;
      p.position.z += gsl_ran_gaussian(m_rng, 0.005) ;
      p.position.z =std::max(p.position.z, 0.2);
      
      double delta_roll  = gsl_ran_gaussian(m_rng, 0.0005* 0.1 * M_PI);//st: 0.5*0.05PI;
      double delta_pitch = gsl_ran_gaussian(m_rng, 0.0005* 0.1 * M_PI);//st: 0.5*0.05PI; 
      double delta_yaw   = gsl_ran_gaussian(m_rng, 0.5* 0.1 * M_PI);//st: 1.0*0.05PI;
      geometry_msgs::Quaternion delta_orientation = tf::createQuaternionMsgFromRollPitchYaw ( delta_roll, delta_pitch,  delta_yaw);
      p.orientation  = tools.quaternionMultiplication(delta_orientation, p.orientation);
      
//       //checking the heading
//     	double roll, pitch, yaw;
//     	//calculate roll pitch yaw from quaternion
//     	tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
//     	q.normalized();
//     	tf::Matrix3x3 m(q);
//     	m.getRPY(roll, pitch, yaw);
// 
// 	if(m_Math::RadianAngleDiff(yaw, getHeading())  > M_PI/2.0){ 
// 	  p.position.x = -p.position.x;
// 	  p.position.y = -p.position.y;
// 	  
// // 	  p.orientation.x = -p.orientation.x;
// // 	  p.orientation.y = -p.orientation.y; 
// // 	  p.orientation.z = -p.orientation.z; 
// 
//           double newYaw = m_Math::CorrectAngleRadian360(yaw-M_PI);
// 	  
//           p.orientation  = tf::createQuaternionMsgFromRollPitchYaw (roll, pitch, newYaw);
//      
// 	}
      
      particles_tmp.poses[j] = p;
      
    }
    
    particles_ego_rot = particles_tmp; 
}


void ParticleFilter::robotPoseUpdate(geometry_msgs::PoseArray & _particles){
  
      robotPose.header.seq = SeqNum;
      robotPose.header.stamp = cur_time_stamp;
      
      geometry_msgs::Pose pose;
      
      if(computePose(_particles, pose)){
	robotPose.pose = pose;
	
	double roll, pitch, yaw;
    	//calculate roll pitch yaw from quaternion
    	tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w);
    	q.normalized();
    	tf::Matrix3x3 m(q);
    	m.getRPY(roll, pitch, yaw);
	
// 	
	float z_v = pose.position.z;  float error_z = 0;
	if( z_v < 0.65 &&z_v > 0.5){ error_z = 0; }
	else if( z_v < 0.5 ){error_z = 0.5 -z_v; }
	else { error_z = z_v - 0.65; }
 
        float error_roll = 0;
	if( roll < 0.16 &&roll > -0.16){ error_roll = 0; }//(-30,30))
	else if( roll < -0.16 ){error_roll = -0.16 -roll; }
	else { error_roll = roll - 0.16; }
	
	float error_pitch = 0;
	if( pitch < 0.06 &&pitch > -0.06){ error_pitch = 0; }//(-10,10))
	else if( pitch < -0.06 ){error_pitch = -0.06 -pitch; }
	else { error_pitch = pitch - 0.06; }
//         std::cout.precision(2);
// 	cout<< " x= "<< pose.position.x<< " y= "<< pose.position.y<< " z= "<<  pose.position.z <<" roll= " 
// 	<<m_Math::Radian2Degree(roll)<< " pitch=  "<<m_Math::Radian2Degree(pitch) << "  yaw= "<<m_Math::Radian2Degree(yaw)<<endl;
	
	if( (error_z  + error_roll + error_pitch)==0 ){
	   params.particleFilter.RandomSamplePct->set(std::max(0.0, params.particleFilter.RandomSamplePct->get()-0.01));
	  
	}
	else{
	   params.particleFilter.RandomSamplePct->set(std::min(0.3, 
							       params.particleFilter.RandomSamplePct->get() + (error_z/10.0  +error_roll +error_pitch)));}
	
      }
      else{cout<<"No update on robot pose."<<endl;}
 
}


void ParticleFilter::run(){
   if(particleNum == 0 ){ InitSampling(); }
   

        particleNum =  params.particleFilter.ParticleNum->get();
     	particleNumKeepUnchanged = (1-MeasConf) * particleNum;
        particleNumForUpdate = particleNum - particleNumKeepUnchanged;
      
	particlesKeepUnchanged.clear();
	particlesForUpdate.clear();
	
// 	cout<<"particleNumKeepUnchanged; "<<  particleNumKeepUnchanged <<endl;
// 	cout<<"particleNumForUpdate; "<<  particleNumForUpdate <<endl;

	
      for(int j =0; j< particleNumKeepUnchanged; ++j){
	   int randv = (particles_ego_rot.poses.size()-1) * gsl_rng_uniform(m_rng);
	   
           particlesKeepUnchanged.push_back(particles_ego_rot.poses[randv]);
      }
      
      for(int j =0; j< particleNumForUpdate; ++j){
	   int randv = (particles_ego_rot.poses.size()-1) * gsl_rng_uniform(m_rng);
           particlesForUpdate.push_back(particles_ego_rot.poses[randv]);
      }
     
     
     if(particleNumForUpdate >0  ){
        updateWeights();
	
// 	if(reSamplingAccordingWeights){
	    ReSampling();
	    predict();
// 	   particles_ego_rot = particles_tmp; 
// 	}
	robotPoseUpdate(particles_ego_rot);
	lineMatcher.backw_Proj.TfRobotPose2CamPose(particles_ego_rot, particles_camera);
	lineMatcher.backw_Proj.TfRobotPose2CamPose(robotPose, cameraPose);
	
     }
     
 
	  
	      //reject the most unlikely particles
// 	    double roll, pitch, yaw;
// 	    //calculate roll pitch yaw from quaternion
// 	    tf::Quaternion q(headingData.orientationPR.y, headingData.orientationPR.y, headingData.orientationPR.z,headingData.orientationPR.w);
// 	    q.normalized();
// 	    tf::Matrix3x3 m(q);
// 	    m.getRPY(roll, pitch, yaw);
//             cout<< "Heading "<<roll<<"   "<<pitch<<"   "<<yaw<<"   "<<headingData.heading<<endl;
}


double ParticleFilter::getHeading()
{       headingOffset = params.particleFilter.HeadingOffset->get() ;
  //TODO heading info
	return m_Math::CorrectAngleRadian360(0 + headingOffset );
//        return m_Math::CorrectAngleRadian360(headingData.heading + headingOffset );
}


// bool ParticleFilter::isOutOfField(const ParticleT& particle) const
// {
// 	const double MARGIN = 0.5;
// 	double hw =fieldInfo.A / 2.0 + MARGIN;
// 	double hl = m_field->length() / 2.0 + MARGIN;
// 
// 	Eigen::Array<float, 2, 1> abspos = particle.getState().head<2>().array().abs();
// 
// 	return abspos.x() > hl || abspos.y() > hw;
// }


// void LocalizationPF::publishTransform()
// {
// 	tf::StampedTransform t;
// 	t.frame_id_ = "/map";
// 	t.child_frame_id_ = "/odom";
// 	t.stamp_ = timestamp() + ros::Duration(0.1);
// 
// 	// subtracting base to odom from map to base and send map to odom instead
// 	tf::Stamped<tf::Pose> odom_to_map;
// 	try
// 	{
// 		tf::Transform tmp_tf(
// 			tf::createQuaternionFromYaw(m_mean.z()),
// 			tf::Vector3(m_mean.x(), m_mean.y(), 0.0)
// 		);
// 		tf::Stamped<tf::Pose> tmp_tf_stamped(
// 			tmp_tf.inverse(),
// 			timestamp(),
// 			"/ego_floor"
// 		);
// 		m_tf.waitForTransform("/odom", "/odom", tmp_tf_stamped.stamp_, ros::Duration(0.5));
// 		m_tf.transformPose("/odom", tmp_tf_stamped, odom_to_map);
// 	}
// 	catch(tf::TransformException& e)
// 	{
// 		fprintf(stderr, "Failed to subtract base to odom transform: %s\n", e.what());
// 		return;
// 	}
// 
// 	tf::Transform inv = odom_to_map.inverse();
// 
// 	t.setOrigin(inv.getOrigin());
// 	t.setRotation(inv.getRotation());
// 
// 	m_pub_tf.sendTransform(t);
// }

// ************************************** //
// ********* Private Methods *********** // 	
// ************************************** //

double ParticleFilter::random(double start,double end)
{
    return start + (end - start)*gsl_rng_uniform(m_rng);
//     return start + (end - start) * rand() / (RAND_MAX + 1.0);
}
float ParticleFilter::random(float start,float end)
{
    return start + (end - start)*gsl_rng_uniform(m_rng);
//     return start + (end - start) * rand() / (RAND_MAX + 1.0);
}

double ParticleFilter::randomab(double a,double b)
{
//     if( rand() / (RAND_MAX + 1.0) > 0.5){return a;}
//     else{return b;}
      if( gsl_rng_uniform(m_rng) > 0.5){return a;}
      else{return b;}
}
