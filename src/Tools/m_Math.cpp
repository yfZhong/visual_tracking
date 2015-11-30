 
// #include "m_Math.h"
#include <visual_tracking/Tools/m_Math.h>

 using namespace vision;
 using namespace m_Math;
 
 
Line::Line( int _id,  float l, Vec2i  _s, Vec2i _e, float _k,float _ang,float _conf)
	: id( _id ), len( l ) , s( _s ), e( _e ) ,k(_k),ang(_ang), conf( _conf ) {   
}

Line::Line( int _id,  Vec2i  _s, Vec2i _e): id( _id ), s( _s ), e( _e ) {
    len  = m_Math::TwoPointDistance(s,e);
    k    = m_Math::getKValue(s,e);
    ang  = m_Math::getAngle(s,e);
    conf = 1.0;

}
Line::Line( int _id,  Vec2i  _s, Vec2i _e, int _nodeId_s, int _nodeId_e): id( _id ), s( _s ), e( _e ), nodeId_s(_nodeId_s ), nodeId_e(_nodeId_e){
  
    len  = m_Math::TwoPointDistance(s,e);
    k    = m_Math::getKValue(s,e);
    ang  = m_Math::getAngle(s,e);
    conf = 1.0;
}

Vec2i Line::getMidPoint(){
  return Vec2i( (s[0] + e[0])/2 ,(s[1] + e[1])/2 );
}

Line Line::PerpendicularLineSegment( Vec2i mid, float len)
{
	float angle = ang;
	angle += M_PI / 2.0;
	float x1 = mid[0] + cos(angle)*len;
	float y1 = mid[1] + sin(angle)*len;
	float x2 = mid[0] - cos(angle)*len;
	float y2 = mid[1] - sin(angle)*len;
	return Line(0, Vec2i(x1, y1), Vec2i(x2, y2));
}

bool Line::Clip(Rect boundry)
{
// 	s[0] = std::min((int) boundry.width-1,  std::max((int) boundry.x, s[0]));
// 	s[1] = std::min((int) boundry.height-1, std::max((int) boundry.y, s[1]));
// 	e[0] = std::min((int) boundry.width-1,  std::max((int) boundry.x, e[0]));
// 	s[1] = std::min((int) boundry.height-1, std::max((int) boundry.y, s[1]));
  if( s[0]< boundry.x  ){ s[0] = boundry.x;  s[1] = k*boundry.x +  e[1] - k * e[0]; }
  if( e[0]< boundry.x  ){ e[0] = boundry.x;  e[1] = k*boundry.x +  s[1] - k * s[0]; }
  if( s[1]<boundry.y  ){ 
      if(k==0){return false;}
      s[1] = boundry.y;  s[0] = (boundry.y -(e[1] - k * e[0]))/k; }
      
  if( e[1]<boundry.y  ){ 
      if(k==0){return false;}
      e[1] = boundry.y;  e[0] = (boundry.y -(s[1] - k * s[0]))/k; }
  
  
  if( s[0]> boundry.width-1 ){ s[0] =boundry.width-1;  s[1] = k*(boundry.width-1) +  e[1] - k * e[0]; }
  if( e[0]> boundry.width-1 ){ e[0] =boundry.width-1;  e[1] = k*(boundry.width-1) +  s[1] - k * s[0]; }
  if( s[1]> boundry.height-1  ){ 
      if(k==0){return false;}
      s[1] = boundry.height-1;  s[0] = (boundry.height-1 -(e[1] - k * e[0]))/k; }
      
  if( e[1]> boundry.height-1  ){ 
      if(k==0){return false;}
      e[1] = boundry.height-1;  e[0] = (boundry.height-1 -(s[1] - k * s[0]))/k; }
      
  return true;
}



Line_w::Line_w( int id_,  float l_w, Vec3f _s_w, Vec3f _e_w, float _k_w,float _ang_w, float _conf):  
      id(id_), len_w( l_w ) , s_w( _s_w ), e_w( _e_w ),k_w(_k_w),ang_w(_ang_w), conf( _conf ){ 
	
}  
      
Line_w::Line_w( int _id,  Vec3f _s_w, Vec3f _e_w):id(_id), s_w( _s_w ), e_w( _e_w ){
  len_w = m_Math:: TwoPointDistance(_s_w, _e_w);
  k_w   = m_Math::getKValue(_s_w, _e_w);
  ang_w = m_Math::getAngle(_s_w, _e_w);
  conf  = 1.0;
  
}

Vec3f Line_w::getMidPoint(){
  return Vec3f( (s_w[0] + e_w[0])/2.0 ,(s_w[1] + e_w[1])/2.0 ,  (s_w[2] + e_w[2])/2.0 );
}



Line_w Line_w::PerpendicularLineSegment( Vec3f mid, float len)
{
	float angle = ang_w;
	angle += M_PI / 2.0;
	float x1 = mid[0] + cos(angle)*len;
	float y1 = mid[1] + sin(angle)*len;
	float x2 = mid[0] - cos(angle)*len;
	float y2 = mid[1] - sin(angle)*len;
	return Line_w(0, Vec3f(x1, y1), Vec3f(x2, y2));
}


 
 
 
void m_Math::GetProjectivePoint( Vec2i p, Line line, Vec2i & pProject){
     GetProjectivePoint( p, line.k, line.s, pProject);
  }
 
void m_Math::GetProjectivePoint( Vec2i p, float line_k, Vec2i pOnLine, Vec2i & pProject){
       Vec2f p1(p[0],p[1]) ;
       Vec2f pOnLine1(pOnLine[0],pOnLine[1]) ;
       Vec2f pProject1;
       GetProjectivePoint( p1 ,  line_k,  pOnLine1,   pProject1);
       pProject[0] = pProject1[0];
       pProject[1] = pProject1[1];
      
  }
  
void m_Math::GetProjectivePoint( Vec2f p, float line_k, Vec2f pOnLine, Vec2f & pProject){
    if (line_k  == 0) //horizontal line
      {
	  pProject[0] = p[0];
	  pProject[1] = pOnLine[1];
      }
      else if(line_k ==std::numeric_limits<float>::max()){//vertical line
	  pProject[0] = pOnLine[0];
	  pProject[1] = p[1];
      }
      else
      {
	  float b = pOnLine[1] - line_k* pOnLine[0];
	  pProject[0] = (line_k* (p[1] - b) + p[0]  )/ (1.0 + line_k * line_k);
	  pProject[1] = (float)(-1.0 / line_k * (pProject[0] - p[0]) + p[1]);
      } 
  }
  
float m_Math::GetProjectiveDistance(Vec2i p, Line line){
        float dist;
	
	if(line.k ==std::numeric_limits<float>::max() ){
	  dist = fabs(p[0] - line.s[0]);
	}
	else{
	  float a = line.k, b = -1, c = line.s[1] -line.k *line.s[0];
	  
	    dist = fabs(((a*p[0]  + b*p[1] + c)/sqrt(a*a +b*b)));
	  
	}
	return dist;    
  }
  
bool m_Math::isProjectedInsideLineSeg( Vec2i p, Line line){
  Vec2i pProject;
  GetProjectivePoint(  p, line, pProject);
  Vec2i PS(line.s[0]-pProject[0],line.s[1]- pProject[1]);
  Vec2i PE(line.e[0]-pProject[0],line.e[1]- pProject[1]);
  
  if(PS[0]*PE[0] + PS[1]*PE[1] <= 0){
    return true; 
    
  }
  else{return false;}
  
  
}

float m_Math::Point2LineSegDistance(Vec2i p, Line line){
  float dist;
  
  if(isProjectedInsideLineSeg( p,line)){
     dist = GetProjectiveDistance(p,line);
  }
  else{
    dist = std::min(TwoPointDistance( p, line.s), TwoPointDistance( p, line.e));
  }
  return dist;

}

float m_Math::TwoPointDistance( Vec2i p1, Vec2i p2){

  return sqrt(pow(p1[0]- p2[0],2) + pow(p1[1] - p2[1],2));
  
}

float m_Math::dot(Vec2f c1, Vec2f c2)
{
    return (c1[0] * c2[0] + c1[1] * c2[1] );
}

float m_Math::getShortestDistance(Line line1, Line line2)
{ 
    if(line1.id>=0 && line2.id >=0){
	double EPS = 0.00000001;
    
    //     point delta21 = new point();
	Vec2f delta21, delta41, delta13;
	delta21[0] = line1.e[0] - line1.s[0];
	delta21[1] = line1.e[1] - line1.s[1];
      
    
	delta41[0]  = line2.e[0] - line2.s[0];
	delta41[1]  = line2.e[1] - line2.s[1];
      
    
	delta13[0] = line1.s[0] - line2.s[0];
	delta13[1] = line1.s[1] - line2.s[1];
	
    
	double a = dot(delta21, delta21);
	double b = dot(delta21, delta41);
	double c = dot(delta41, delta41);
	double d = dot(delta21, delta13);
	double e = dot(delta41, delta13);
	double D = a * c - b * b;
    
	double sc, sN, sD = D;
	double tc, tN, tD = D;
    
	if (D < EPS)
	{
	    sN = 0.0;
	    sD = 1.0;
	    tN = e;
	    tD = c;
	}
	else
	{
	    sN = (b * e - c * d);
	    tN = (a * e - b * d);
	    if (sN < 0.0)
	    {
		sN = 0.0;
		tN = e;
		tD = c;
	    }
	    else if (sN > sD)
	    {
		sN = sD;
		tN = e + b;
		tD = c;
	    }
	}
    
	if (tN < 0.0)
	{
	    tN = 0.0;
    
	    if (-d < 0.0)
		sN = 0.0;
	    else if (-d > a)
		sN = sD;
	    else
	    {
		sN = -d;
		sD = a;
	    }
	}
	else if (tN > tD)
	{
	    tN = tD;
	    if ((-d + b) < 0.0)
		sN = 0;
	    else if ((-d + b) > a)
		sN = sD;
	    else
	    {
		sN = (-d + b);
		sD = a;
	    }
	}
    
	if (abs(sN) < EPS) sc = 0.0;
	else sc = sN / sD;
	if (abs(tN) < EPS) tc = 0.0;
	else tc = tN / tD;
    
	Vec2f dP;
      
	dP[0] = delta13[0] + (sc * delta21[0]) - (tc * delta41[0]);
	dP[1] = delta13[1] + (sc * delta21[1]) - (tc * delta41[1]);
      
    
	return sqrt(dot(dP, dP)); 
      
    }
    else return 100000.0;
}

float m_Math::getKValue(Vec2f c1, Vec2f c2){

      float k;
      
      if( (c2[0] - c1[0]) ==0 ){ 
	  k = std::numeric_limits<float>::max();
      }
      else{  
	  k = (c2[1] -c1[1])/(c2[0] - c1[0]); }
      return k;

}
float m_Math::getAngle(Vec2f c1, Vec2f c2){
  
      if((c2[0]-c1[0]) ==0){
	  return -M_PI/2.0;
      }
      else{
          return atan(  (c2[1]-c1[1])/ (c2[0]-c1[0]) );
      }
}

float m_Math::getKValue(Vec2i c1, Vec2i c2){
     return getKValue(Vec2f(c1[0],c1[1]) , Vec2f(c2[0],c2[1]) );
}
float m_Math::getAngle(Vec2i c1, Vec2i c2){
   return getAngle(Vec2f(c1[0],c1[1]) , Vec2f(c2[0],c2[1]) );
}

float m_Math::getKValueFromAngle(float ang){
      float k;
      if(ang == -M_PI/2.0){k = std::numeric_limits<float>::max(); }
      else{k = tan(ang);}
     
      return k;
}

 // in degree
float m_Math::AngDif(float ang1, float ang2){//in:radian, out degree
      float dif;
      dif = fabs(ang1-ang2);
      if(dif > M_PI/2.){ dif =M_PI - dif; }
      return dif/M_PI * 180.0 ;
 
}







// bool DiagonalAng( float ang1, float ang2){
//   float DIAGONAL_ANGLE_RNAGE = 0.08;
//   float dif;
//   
//   dif = abs(ang1-ang2);
//   
//   if(dif > M_PI/2.){ dif =M_PI - dif; }
//   
//   if(abs(dif-M_PI/2) < DIAGONAL_ANGLE_RNAGE){return true;}
//   else{return  false;}
//   
// }



void m_Math::GetProjectivePoint( Vec3f p, Line_w line, Vec3f & pProject){
      if (line.k_w  == 0) //horizontal line
      {
	  pProject[0] = p[0];
	  pProject[1] = line.s_w[1];
      }
      else if(line.k_w ==std::numeric_limits<float>::max()){//vertical line
	  pProject[0] = line.s_w[0];
	  pProject[1] = p[1];
      }
      else
      {
	  float b = line.s_w[1] - line.k_w* line.s_w[0];
	  pProject[0] = (line.k_w * (p[1] - b) + p[0]  )/ (1 + line.k_w * line.k_w);
	  pProject[1] = (float)(-1 / line.k_w * (pProject[0] - p[0]) + p[1]);
      }
      pProject[2]= 0;
  }
  
  
void m_Math::GetProjectivePoint( Vec3f p, float line_k, Vec3f pOnLine, Vec3f& pProject){
      if (line_k  == 0) //horizontal line
      {
	  pProject[0] = p[0];
	  pProject[1] = pOnLine[1];
      }
      else if(line_k ==std::numeric_limits<float>::max()){//vertical line
	  pProject[0] = pOnLine[0];
	  pProject[1] = p[1];
      }
      else
      {
	  float b = pOnLine[1] - line_k* pOnLine[0];
	  pProject[0] = (line_k* (p[1] - b) + p[0]  )/ (1 + line_k * line_k);
	  pProject[1] = (float)(-1 / line_k * (pProject[0] - p[0]) + p[1]);
	  pProject[2] =0;
      }
       pProject[2] =0;
  }
  

float m_Math::GetProjectiveDistance(Vec3f p, Line_w line){
   
      float dist;
	    
      if(line.k_w == std::numeric_limits<float>::max()){
	  dist = fabs(p[0] - line.s_w[0]);
      }
      else{
	float a = line.k_w, b = -1, c = line.s_w[1] -line.k_w *line.s_w[0];
	
	  dist = fabs(((a*p[0]  + b*p[1] + c)/sqrt(a*a +b*b)));
	
      }
      return dist;    
   
}

float m_Math::PointToLineSegDistance(Vec3f p, Line_w line){
  float dist;
  
  if(isProjectedInsideLineSeg( p,line)){
     dist = GetProjectiveDistance(p,line);
  }
  else{
    dist = std::min(TwoPointDistance( p, line.s_w), TwoPointDistance( p, line.e_w));
  }
  return dist;

}

bool m_Math::isInsideLineSeg( Vec3f p, Line_w line){
//     Vec3f pProject;
//     GetProjectivePoint(  p, line, pProject);
    Vec3f PS(line.s_w[0]-p[0],line.s_w[1]- p[1]);
    Vec3f PE(line.e_w[0]-p[0],line.e_w[1]- p[1]);
   
    if(PS[0]*PE[0] + PS[1]*PE[1] <= 0){
      return true; 
      
    }
    else{return false;}

}

bool m_Math::isProjectedInsideLineSeg( Vec3f p, Line_w line){
    Vec3f pProject;
    GetProjectivePoint(  p, line, pProject);
    Vec3f PS(line.s_w[0]-pProject[0],line.s_w[1]- pProject[1]);
    Vec3f PE(line.e_w[0]-pProject[0],line.e_w[1]- pProject[1]);
   
    if(PS[0]*PE[0] + PS[1]*PE[1] <= 0){
      return true; 
      
    }
    else{return false;}

}
  


float m_Math::TwoPointDistance( Vec3f p1, Vec3f p2){ //z = 0
  
  return sqrt(pow(p1[0]- p2[0],2)+pow(p1[1] - p2[1],2));
  
}

bool m_Math::getLineProjectToLineSeg(Line_w line1, Line_w line2 ,Line_w & seg ){
    Vec3f start_w, end_w;
    Vec3f  pProjectS, pProjectE;
    GetProjectivePoint( line1.s_w, line2, pProjectS);
    GetProjectivePoint( line1.e_w, line2, pProjectE);
    
    Line_w projectedLine(0, TwoPointDistance(pProjectS, pProjectE) , pProjectS, pProjectE, line2.k_w, line2.ang_w, 1.0) ;;
    bool Sin = isInsideLineSeg(pProjectS,line2);
    bool Ein = isInsideLineSeg(pProjectE,line2);
    
    if( Sin && Ein ){
      start_w = pProjectS;
      end_w = pProjectE;
    }
    else if(Sin || Ein ){
      if(Sin){start_w = pProjectS;}
      else{ start_w = pProjectE;}
      
      if(isInsideLineSeg(line2.s_w, projectedLine)){
	end_w= line2.s_w;
      }
      else{
	end_w= line2.e_w;
      }

    }
    else{return false;}
  
    seg = Line_w (0 , start_w, end_w ) ;
    
    return true;
}

void m_Math::getClosestPointOnLineSeg(Vec3f p1, Line_w line ,Vec3f &pClosest){
  
    if(isProjectedInsideLineSeg( p1,line)){
       GetProjectivePoint(p1,line,pClosest );
    }
    else if ( TwoPointDistance( p1, line.s_w) <= TwoPointDistance( p1, line.e_w) ){
       pClosest =  line.s_w;
    }
    else  pClosest =  line.e_w;
}





float m_Math::getKValue(Vec3f c1, Vec3f c2){
      float k;
      if( (c2[0] - c1[0]) ==0 ){ 
	  k = std::numeric_limits<float>::max();
      }
      else{  
	   k = (c2[1] -c1[1])/(c2[0] - c1[0]);}
      return k;
    
  }
float m_Math::getAngle(Vec3f c1, Vec3f c2){
       if((c2[0]-c1[0]) ==0){
	  return -M_PI/2.0;
      }
      else{
          return atan(  (c2[1]-c1[1])/ (c2[0]-c1[0]) );
      }
    
  }


float m_Math::getShortestDistance(Line_w line1, Line_w line2)
{
	double EPS = 0.00000001;
    
    //     point delta21 = new point();
	Vec2f delta21, delta41, delta13;
	delta21[0] = line1.e_w[0] - line1.s_w[0];
	delta21[1] = line1.e_w[1] - line1.s_w[1];
      
    
	delta41[0]  = line2.e_w[0] - line2.s_w[0];
	delta41[1]  = line2.e_w[1] - line2.s_w[1];
      
    
	delta13[0] = line1.s_w[0] - line2.s_w[0];
	delta13[1] = line1.s_w[1] - line2.s_w[1];
	
    
	double a = dot(delta21, delta21);
	double b = dot(delta21, delta41);
	double c = dot(delta41, delta41);
	double d = dot(delta21, delta13);
	double e = dot(delta41, delta13);
	double D = a * c - b * b;
    
	double sc, sN, sD = D;
	double tc, tN, tD = D;
    
	if (D < EPS)
	{
	    sN = 0.0;
	    sD = 1.0;
	    tN = e;
	    tD = c;
	}
	else
	{
	    sN = (b * e - c * d);
	    tN = (a * e - b * d);
	    if (sN < 0.0)
	    {
		sN = 0.0;
		tN = e;
		tD = c;
	    }
	    else if (sN > sD)
	    {
		sN = sD;
		tN = e + b;
		tD = c;
	    }
	}
    
	if (tN < 0.0)
	{
	    tN = 0.0;
    
	    if (-d < 0.0)
		sN = 0.0;
	    else if (-d > a)
		sN = sD;
	    else
	    {
		sN = -d;
		sD = a;
	    }
	}
	else if (tN > tD)
	{
	    tN = tD;
	    if ((-d + b) < 0.0)
		sN = 0;
	    else if ((-d + b) > a)
		sN = sD;
	    else
	    {
		sN = (-d + b);
		sD = a;
	    }
	}
    
	if (fabs(sN) < EPS) sc = 0.0;
	else sc = sN / sD;
	if (fabs(tN) < EPS) tc = 0.0;
	else tc = tN / tD;
    
	Vec2f dP;
      
	dP[0] = delta13[0] + (sc * delta21[0]) - (tc * delta41[0]);
	dP[1] = delta13[1] + (sc * delta21[1]) - (tc * delta41[1]);
      
    
	return sqrt(dot(dP, dP));
          
  
}


float m_Math::dot(Vec3f c1, Vec3f c2)
{
    return (c1[0] * c2[0] + c1[1] * c2[1] );
}


 
  
void m_Math::GetProjectivePointOnCircle( Vec3f p, float r, Vec3f & pProject){
     if(p[0] == 0 && p[1] ==0 ){//atan2 undefined
        pProject[0] = 0;
        pProject[1] = r;
    }
    else{
        float ang = atan2(p[1], p[0] ); //(-PI, PI)
        pProject[0] = (r * cos(ang));
	pProject[1] = (r * sin(ang));
      
    }
      pProject[2]= 0;
  }
  
float m_Math::GetProjectiveDistanceOnCircle( Vec3f p, float r){
     Vec3f  pProject;
     GetProjectivePointOnCircle(  p,  r,  pProject);
     return TwoPointDistance( p, pProject );
}







//Normalize to [-180,180):
double m_Math::CorrectAngleDegree360(double x) {
    x = fmod(x, 360);
    if (x < 0){x += 360;}
    
    if(x >=180){x-=360;}
    return x;
}

//Normalize to [-90,90]:
double m_Math::CorrectAngleDegree180(double x) {
    x = fmod(x, 360);//(-360, 360)
    if (x < 0){x += 360;}
    
    if(x>90&& x<270){
              x-=180;
    }
    else if(x >=270){
              x -= 360;
    }
    
    return x;

}

//Normalize to [-PI,PI):
double m_Math::CorrectAngleRadian360(double x) {
  
	 double ang = Degree2Radian((CorrectAngleDegree360((Radian2Degree(x)))));
// 	 if (ang > M_PI){ang -= 2*M_PI;}
	 return ang;
}

//Normalize to [-Pi/2,PI/2]:
double m_Math::CorrectAngleRadian180(double x) {
	return Degree2Radian((CorrectAngleDegree180((Radian2Degree(x)))));
}

double m_Math::Radian2Degree(double r) {
	return (r / M_PI) * (180);
}


double m_Math::Degree2Radian(double d) {
	return (d * M_PI) / (180);
}

double m_Math::RadianAngleDiff(double yaw1, double yaw2){
         yaw1 = CorrectAngleRadian360(yaw1);
         yaw2 = CorrectAngleRadian360(yaw2);
	 
	 return std::min( fabs(yaw1-yaw2),fabs( 2*M_PI -fabs(yaw1-yaw2)) );
  
}



Vec2i startP;
bool SortFunc(const Vec2i  a, const Vec2i b){
    float x = abs(startP[0] - a[0]);
    float y = abs(startP[1] - a[1]);
    float distanceA = sqrt(x * x + y * y);

    x = abs(startP[0] - b[0]);
    y = abs(startP[1]- b[1]);
    float distanceB = sqrt(x * x + y * y);

    return distanceA < distanceB; 
}
vector<Vec2i> m_Math:: GetMidPoints(Vec2i p1, Vec2i p2, int count  /*Means that lines count will be 2^count*/){ 
  
        vector<Line> lsList, lsListTmp;
	int id = 0;
	lsList.push_back(Line(id++, p1, p2));
        
	    for (int counter = 0; counter < count; counter++)
	    {
		    lsListTmp.clear();
		    for (size_t i = 0; i < lsList.size(); i++)
		    {
			    Line tmp = lsList[i];
			    Vec2i midPoint = tmp.getMidPoint();
			    lsListTmp.push_back(Line(id++, tmp.s, midPoint));
			    lsListTmp.push_back(Line(id++, tmp.e, midPoint));
		    }
		    lsList = lsListTmp;
	    }
	    
      vector<Vec2i> res;
      startP = p1;
      for (size_t i = 0; i < lsList.size(); i++)
      {
	  res.push_back( lsList[i].s);
	  res.push_back( lsList[i].e);
      }
      sort(res.begin(), res.end(), SortFunc);
      //TODO: remove 
      return res;
}


Line m_Math::MergeTwoLinesOnImg(Line line1, Line line2, int id)
{ 
      float ang = 0;
      Vec2f  line1MidP((line1.s[0] + line1.e[0])/2.0, (line1.s[1] + line1.e[1])/2.0);
      Vec2f  line2MidP((line2.s[0] + line2.e[0])/2.0, (line2.s[1] + line2.e[1])/2.0);
      float min_value = 0.000001;
      float r = line1.len/(std::max(line1.len + line2.len, min_value));
  
      Vec2f newMidP(r*line1MidP[0] + (1-r) * line2MidP[0] , r*line1MidP[1] + (1-r) * line2MidP[1] );

      //(-M_PI/2, M_PI/2)
      if(fabs(line1.ang -line2.ang)>M_PI/2.0 ){  
	if(line1.ang<0 ){ line1.ang = line1.ang +M_PI;}
	else{ line2.ang = line2.ang + M_PI;}
      }
      ang = r*line1.ang + (1-r) *line2.ang; 
      if(ang >=M_PI/2.0){ang = ang - M_PI; }
      
      
      
      float k = m_Math::getKValueFromAngle(ang);
      vector<Vec2f> pProject; 
      Vec2f pp;
      m_Math::GetProjectivePoint( Vec2f(line1.s[0],line1.s[1]), k, newMidP,  pp); pProject.push_back(pp);
      m_Math::GetProjectivePoint( Vec2f(line1.e[0],line1.e[1]), k, newMidP,  pp); pProject.push_back(pp);
      m_Math::GetProjectivePoint( Vec2f(line2.s[0],line2.s[1]), k, newMidP,  pp); pProject.push_back(pp);
      m_Math::GetProjectivePoint( Vec2f(line2.e[0],line2.e[1]), k, newMidP,  pp); pProject.push_back(pp);
      
      Vec2f startf = newMidP, endf = newMidP;
      
      for(unsigned int i = 0; i< pProject.size(); ++i){
	   
	    if(fabs(ang - M_PI/2.0)<0.000001 ){//vertical line
		if(pProject[i][1]< startf[1]){ startf[0] =pProject[i][0];startf[1] = pProject[i][1]; }
		if(pProject[i][1]>   endf[1]){   endf[0] =pProject[i][0];  endf[1] = pProject[i][1]; }

	    }
	    else
	    {  
	      if(pProject[i][0]< startf[0]){ startf[0] = pProject[i][0]; startf[1] = pProject[i][1]; }
	      if(pProject[i][0]>   endf[0]){   endf[0] = pProject[i][0];   endf[1] = pProject[i][1]; }
	      
	    }
        }  
       
        Vec2i start(startf[0],startf[1]), end (endf[0],endf[1]);
        float length = m_Math::TwoPointDistance(start, end);
      
      Line Merged_line(id, length, start, end, k, ang, 1.0) ;
  
      return Merged_line;
  
}
void m_Math::MergeLinesOnImg(std::vector< Line > m_LineBuffer_Before_Merge, std::vector< Line >& m_LineBuffer_After_Merge, double maxDegree, double maxDistance){
  
//    cv::Mat vis_houghLines(siY,siX,CV_8UC3,cv::Scalar(150, 150, 150));
      m_LineBuffer_After_Merge.clear();
      if(m_LineBuffer_Before_Merge.size()<1){return;}
      int AngleToMerge = maxDegree;
      int MinLineSegDistance = maxDistance;
      int MinProjectedDistance = maxDistance;
      
//       cout<<"Merge Line: ";
      std::vector< Line >  m_LineBuffer_Before_Merge_tmp = m_LineBuffer_Before_Merge ;
      std::vector< Line >  m_LineBuffer_After_Merge_tmp;
//       int loopNum=0;
      while (true){
//           loopNum++;
	  int id=0;
	  for(unsigned int i=0; i < m_LineBuffer_Before_Merge_tmp.size(); ++i ){
	      if(m_LineBuffer_Before_Merge_tmp[i].id !=-1){
		    Line a = m_LineBuffer_Before_Merge_tmp[i];
		    for(unsigned int j= i + 1; j < m_LineBuffer_Before_Merge_tmp.size(); ++j ){
			if(m_LineBuffer_Before_Merge_tmp[j].id != -1 ){
			  Line b = m_LineBuffer_Before_Merge_tmp[j];
			  Vec2i Midpoint_a((a.s[0]+a.e[0])/2.0, (a.s[1]+a.e[1])/2.0);
			  Vec2i Midpoint_b((b.s[0]+b.e[0])/2.0, (b.s[1]+b.e[1])/2.0);
			  if(m_Math::AngDif( a.ang, b.ang) < AngleToMerge
			    &&  m_Math::getShortestDistance(a, b) <MinLineSegDistance
			    &&  m_Math::GetProjectiveDistance(Midpoint_a, b) < MinProjectedDistance
			    &&  m_Math::GetProjectiveDistance(Midpoint_b, a) < MinProjectedDistance){
			     Line c = m_Math::MergeTwoLinesOnImg(a, b, id++);
			     if( c.s != c.e ){
			      a = c;
			      m_LineBuffer_Before_Merge_tmp[j].id = -1;}
			    
//                             cout<<"Merge Line: "<<a.id<<" and "<< b.id<<", "<<endl;
// 			    cout<<"        "<< a.ang_w<<"  "<< b.ang_w<<"  "<< a.k_w<<"  "<< b.k_w <<"    "<<getShortestDistance_WorldCord(a, b)
// 			    <<"  " <<GetProjectiveDistance_WorldCord(Vec2f((a.s_w[0]+a.e_w[0])/2.0,(a.s_w[1]+a.e_w[1])/2.0), b)<<endl;
			  
			  }
			}
		      
		    }
		    m_LineBuffer_After_Merge_tmp.push_back(a);

	      }
	  
	  }
	  if(m_LineBuffer_After_Merge_tmp.size() == m_LineBuffer_Before_Merge_tmp.size() ){break;}
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


void m_Math::SamplePointsOnLine(Line_w line, float dist, vector<tf::Vector3>  &PointsOnLine){ 
      PointsOnLine.clear();
      
      double num = ceil(line.len_w/dist);
      num= std::max(num,2.);
      for(int i = 0; i<num; ++i){
	  PointsOnLine.push_back(tf::Vector3( line.s_w[0] + float((line.e_w[0] -line.s_w[0])* i)/num,
				                    line.s_w[1] + float((line.e_w[1] - line.s_w[1])* i)/num, 0 ));
      }
      
     return;
}

void m_Math::SamplePointsOnLines(Vector<Line_w> lines, float dist, std::vector<pair<tf::Vector3, int > > &PointsOnLines){ 
      PointsOnLines.clear();
      
       for(unsigned int i =0; i < lines.size(); ++i ){
	    double num = ceil(lines[i].len_w/dist);
	    num= std::max(num,2.);
	    for(int j = 0; j<num; ++j){
	    PointsOnLines.push_back( make_pair( tf::Vector3( lines[i].s_w[0] + float((lines[i].e_w[0] -lines[i].s_w[0])* j)/num,
							lines[i].s_w[1] + float((lines[i].e_w[1] - lines[i].s_w[1])* j)/num, 0   ), lines[i].id));
	    }
      }

     return;
}


void m_Math::SamplePointsOnLine(Line line, float dist, vector<cv::Point>  &PointsOnLine){ 
      PointsOnLine.clear();
      
      double num = ceil(line.len/dist);
      num= std::max(num,2.);
      for(int i = 0; i<num; ++i){
	  PointsOnLine.push_back(cv::Point( line.s[0] + float((line.e[0] -line.s[0])* i)/num,
				                    line.s[1] + float((line.e[1] - line.s[1])* i)/num));
      }
      
     return;
}

void m_Math::SamplePointsOnLines(Vector<Line> lines, float dist, vector<cv::Point>  &PointsOnLines){ 
      PointsOnLines.clear();
      
       for(unsigned int i =0; i < lines.size(); ++i ){
	    double num = ceil(lines[i].len/dist);
	    num= std::max(num,2.);
	    for(int j = 0; j<num; ++j){
	    PointsOnLines.push_back(  cv::Point( lines[i].s[0] + float((lines[i].e[0] -lines[i].s[0])* j)/num,
							lines[i].s[1] + float((lines[i].e[1] - lines[i].s[1])* j)/num  ));
	    }
      }

     return;
}




// ****************************************************************************** //
bool m_Math::intersect( Line a, Line b ,Vec2i& intersetP)
// ****************************************************************************** //
{
    float xa, xb, xc, xd;
    float ya, yb, yc, yd;
    float dx1, dx2, dy1, dy2;
    float nenn;
    float beta;
    float alpha;

//     if ( ( a == b ) || ( c == d ) || ( a == c ) || ( a == d ) || ( b == c ) || ( b == d ) ) {
// 	return false;
//     }

    xa =  a.s[0];
    ya =  a.s[1];
    xb =  a.e[0]; 
    yb =  a.e[1];
    xc =  b.s[0];
    yc =  b.s[1];
    xd =  b.e[0]; 
    yd =  b.e[1];

//     if ( ( max( xa, xb ) < min( xc, xd ) ) || ( max( xc, xd ) < min( xa, xb ) ) ||
// 	  ( max( ya, yb ) < min( yc, yd ) ) || ( max( yc, yd ) < min( ya, yb ) )    ) {
// 	    return false;
//     }

    dx1 = xb - xa;
    dy1 = yb - ya;
    dx2 = xd - xc;
    dy2 = yd - yc;

    nenn = dy2 * dx1 - dx2 * dy1;

    if ( nenn == 0 ) {
	return false;
    }

    beta = ( float ) ( ya * dx1 - yc * dx1 + xc * dy1 - xa * dy1 ) / ( float ) ( nenn );

//     if ( ( beta <= 0 ) || ( beta >= 1 ) ) {
// 	return false;
//     }

    if ( dx1 != 0 ) {
	alpha = ( xc - xa + dx2 * beta ) / ( float ) ( dx1 );
    }
    else if ( dy1 != 0 ) {
	alpha = ( yc - ya + dy2 * beta ) / ( float ) ( dy1 );
    }
    else {
	alpha = 0;
    }

//     if ( ( alpha <= 0 ) || ( alpha >= 1 ) ) {
// 	return false;
//     }

    intersetP[0] = xc + ( float ) ( dx2 ) * beta;
    intersetP[1]= yc + ( float ) ( dy2 ) * beta;

    return true;

}; // END of intersect METHOD





// std::pair<Eigen::Matrix4d, Eigen::Vector4d> ParticleFilter::computeEigenValuesAndVectors(Eigen::Matrix4d mat ){
//     
//     Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(mat);
//     Eigen::Vector4d eigenValues = Eigen::Vector4d::Identity();
//     Eigen::Matrix4d eigenVectors = Eigen::Matrix4d::Zero();	
//     
//     if (eigensolver.info () == Eigen::Success)
//     {
// 	eigenValues = eigensolver.eigenvalues();
// 	eigenVectors = eigensolver.eigenvectors();
// // 	cout<<eigenValues.transpose() <<endl;
// 	
//     }
//     else
// 	{ROS_WARN_THROTTLE(1, "failed to compute eigen vectors/values. Is the covariance matrix correct?");}
//     return std::make_pair (eigenVectors, eigenValues);
// 
// }


// cv::Point2f ParticleFilter::RotateCoordinateAxis(double alpha, cv::Point2f p) {
// 	alpha = Degree2Radian(alpha);
// 	return cv::Point2f((float) (p.x * cos(alpha) - p.y * sin(alpha)),
// 			  (float) (p.x * sin(alpha) + p.y * cos(alpha)));
// }

