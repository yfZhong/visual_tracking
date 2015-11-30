

// #include "findGoalPosts.h"
#include <visual_tracking/FieldObjects/findGoalPosts.h>
using namespace vision;

FindGoalPosts::FindGoalPosts(){
     H = params.camera.height->get();
     W = params.camera.width->get();
}


void FindGoalPosts::findGoalPosts(Mat rawHSV, Mat B_Channel, Mat goalBinaryImg, vector<Point> fieldHull){
  
    
    Mat cannyImg;
    blur(B_Channel.clone(), cannyImg, Size(3, 3));
    int cannyThreadshold = params.goal.cannyThreadshold->get();
    Canny(cannyImg, cannyImg, cannyThreadshold, cannyThreadshold * 3, 3);
    
//     Mat goalBinaryImgErode,goalBinaryImgDilate;
    erode(goalBinaryImg, goalBinaryImg, Mat(), Point(-1, -1), params.goal.erode->get());
    dilate(goalBinaryImg, goalBinaryImg, Mat(), Point(-1, -1), params.goal.dilate->get());
       
    
    cv::Rect rec;
    rec.x = 0;
    rec.y = 0;
    rec.width = params.camera.width->get();
    rec.height = params.camera.height->get();
    
    const int MinLineLength = params.goal.MinLineLength->get();
    const int DistanceToMerge = params.goal.DistanceToMerge->get();
    const int MaxOutField = params.goal.MaxOutField->get();
    const int MinNearFieldUpPoint = -20;
    
    
    vector<cv::Vec4i> linesP;
    HoughLinesP(cannyImg, linesP, 1, M_PI / 45, 10, MinLineLength, 20);
    
    vector<Line> allVerLines;
    
    Mat allVerLinesImg(H,W,CV_8UC3,cv::Scalar(150, 150, 150));
    vector<vector<cv::Point> > tmphulls = vector<vector<cv::Point> >(1,fieldHull);
    drawContours(allVerLinesImg, tmphulls, -1,  cv::Scalar(200, 0, 50), 2, 8);
    
    int id = 0;

	for (size_t i = 0; i < linesP.size(); i++)
	{
		cv::Vec4i lP = linesP[i];
		Line tmpLine( id++, Vec2i(lP[0], lP[1]),  Vec2i(lP[2], lP[3]));

		double leftAvg = 0;
		double rightAvg = 0;
		if (m_Math::AngDif(tmpLine.ang, -M_PI/2.0 ) <15)
// 					&& (pointPolygonTest(fieldHull, Point(lP[0], lP[1]), false)
// 							|| pointPolygonTest(fieldHull, Point(lP[2], lP[3]) , false)))
		{
			
			vector<Vec2i> midds = m_Math::GetMidPoints(Vec2i(lP[0], lP[1]),  Vec2i(lP[2], lP[3]), 5); //2^5+1 = 64
			int maxMidPoint = midds.size();
			int vote_for_doubleLeft = 0;
			int vote_for_doubleRight = 0;
			
			cv::line(allVerLinesImg, Point(lP[0], lP[1]), Point(lP[2], lP[3]), cv::Scalar( 0, 0, 100), 1);

			cv::Point down =(tmpLine.s[1] < tmpLine.e[1]) ?cv::Point( tmpLine.e[0],tmpLine.e[1] ) : cv::Point( tmpLine.s[0],tmpLine.s[1] );
			double jumpDouble = params.goal.jumpMax->get(); //pixel
// 				cv::Point2f downReal;
// 				if (!distortionModel.GetOnRealCordinate(down, downReal))
// 				{
// 					ROS_ERROR("Erorr in programming!");
// 					return false;
// 				}
			double downDistance2Field = pointPolygonTest(fieldHull, down, true);
			if (downDistance2Field > MaxOutField || downDistance2Field< 0) //down point should be near inside
				continue;
// 				double distance = GetDistance(downReal);
// 				if (distance < 2)
// 				{
// 					jumpDouble = 40;
// 				}
// 				else if (distance >= 2 && distance < 3)
// 				{
// 					jumpDouble = 23;
// 				}
// 				else
// 				{
// 					jumpDouble = 15;
// 				}
//                                jumpDouble = 25;
			for (size_t j = 0; j < midds.size(); j++)
			{

				Line tocheck = tmpLine.PerpendicularLineSegment( midds[j],jumpDouble);
				cv::Point tocheck_s( tocheck.s[0],tocheck.s[1]);
				cv::Point tocheck_e( tocheck.e[0],tocheck.e[1]);

				cv::Point left = (tocheck_s.x < tocheck_e.x) ? tocheck_s : tocheck_e;
				cv::Point right = (tocheck_s.x < tocheck_e.x) ? tocheck_e : tocheck_s;
				cv::LineIterator itLeft(cannyImg, midds[j], left, 8);
				cv::LineIterator itRight(cannyImg, midds[j], right, 8);
				cv::LineIterator itHSVLeft(rawHSV, midds[j], left, 8);
				cv::LineIterator itHSVRight(rawHSV, midds[j], right,8);

				cv::LineIterator itBinaryLeft(goalBinaryImg, midds[j], left, 8);
				cv::LineIterator itBinaryRight(goalBinaryImg, midds[j], right,8);
				
				
				int safeToShow = 0;
				if (tocheck_s.x >= 0 && tocheck_s.y >= 0
						&& tocheck_s.x < params.camera.width->get()
						&& tocheck_s.y < params.camera.height->get())
				{
					safeToShow++;
				}
				if (tocheck_e.x >= 0 && tocheck_e.y >= 0
						&& tocheck_e.x < params.camera.width->get()
						&& tocheck_e.y < params.camera.height->get())
				{
					safeToShow++;
				}
// 
// 					if (safeToShow >= 2) //Both ends are in the picture rectangle
// 					{
// 						if (SHOWGUI && params.debug.showGoalD->get())
// 						{
// 
// 							cv::line(guiImg, tocheck.P1, tocheck.P2,
// 									yellowColor(), 1);
// 						}
// 					}

				for (int k = 0; k < itLeft.count;
						k++, ++itLeft, ++itHSVLeft)
				{
					if (k < 2)
						continue;
					uchar val = *(*itLeft);
// 					cv::Vec3b hsvC = (cv::Vec3b) *itHSVLeft;
					uchar BinaryC =  *(*itBinaryLeft);

					if (val > 0 && k > params.goal.minDoubleLength->get())
					{
							if (safeToShow >= 2 /*&& SHOWGUI
									&& params.debug.showGoalD->get()*/)
							{
								cv::line(allVerLinesImg, midds[j], itHSVLeft.pos(),
										cv::Scalar( 100, 0, 0), 1);
							}
						leftAvg += k;
						vote_for_doubleLeft++;
						break;
					}

// 					if (hsvC[0] >= params.goalhsv.h0->get()
// 							&& hsvC[0] <= params.goalhsv.h1->get()
// 							&& hsvC[1] >= params.goalhsv.s0->get()
// 							&& hsvC[1] <= params.goalhsv.s1->get()
// 							&& hsvC[2] >= params.goalhsv.v0->get()
// 							&& hsvC[2] <= params.goalhsv.v1->get())
// 					{
// 						//color is ok
// 					}
// 					else
// 					{
// 						break;
// 					}


					if (BinaryC >0)
					{
						//color is ok
					}
					else
					{
						break;
					}




				}

				for (int k = 0; k < itRight.count;
						k++, ++itRight, ++itHSVRight)
				{
					if (k < 2)
						continue;
					uchar val = *(*itRight);
// 					cv::Vec3b hsvC = (cv::Vec3b) *itHSVRight;
					uchar BinaryC =  *(*itBinaryRight);
					if (val > 0 && k > params.goal.minDoubleLength->get())
					{
// 							if (safeToShow >= 2 && SHOWGUI
// 									&& params.debug.showGoalD->get())
// 							{
// 								cv::line(guiImg, midds[j], itHSVRight.pos(),
// 										redColor(), 1);
// 							}
					  
						  	if (safeToShow >= 2 /*&& SHOWGUI
									&& params.debug.showGoalD->get()*/)
							{
								cv::line(allVerLinesImg, midds[j], itHSVRight.pos(),
										cv::Scalar( 0, 100, 0), 1);
							}
						rightAvg += k;
						vote_for_doubleRight++;
						break;
					}

// 					if (hsvC[0] >= params.goalhsv.h0->get()
// 							&& hsvC[0] <= params.goalhsv.h1->get()
// 							&& hsvC[1] >= params.goalhsv.s0->get()
// 							&& hsvC[1] <= params.goalhsv.s1->get()
// 							&& hsvC[2] >= params.goalhsv.v0->get()
// 							&& hsvC[2] <= params.goalhsv.v1->get())
// 					{
// 						//color is ok
// 					}
// 					else
// 					{
// 						break;
// 					}
					if (BinaryC >0)
					{
						//color is ok
					}
					else
					{
						break;
					}
				}

			}

			bool leftOK = (vote_for_doubleLeft / (float) maxMidPoint) * 100. > params.goal.lineVoteDouble->get();

			bool rightOk = (vote_for_doubleRight / (float) maxMidPoint) * 100. > params.goal.lineVoteDouble->get();

			if (leftOK || rightOk)
			{

				{
// 						LineSegment tmpLineChanged = tmpLine;
					
					Line tmpLineChanged = tmpLine;

					if (leftOK)
					{
						int amount = abs(leftAvg / vote_for_doubleLeft) / 2.;
						tmpLineChanged.s[0] -= amount;
						tmpLineChanged.e[0] -= amount;
					}
					else if (rightOk)
					{
						int amount = abs(rightAvg / vote_for_doubleRight) / 2.;
						tmpLineChanged.s[1]+= amount;
						tmpLineChanged.e[1] += amount;
					}
					if(tmpLineChanged.Clip(rec)){
					    allVerLines.push_back(tmpLineChanged);}
				}
			}
			  
		}
	}


	vector<Line> allVerLines2;
	m_Math::MergeLinesOnImg(allVerLines, allVerLines2,  30, DistanceToMerge);

	vector<Line> finalPosts;
	for (size_t i = 0; i < allVerLines2.size(); i++)
	{
		Line tmpLine = allVerLines2[i];
		
		cv::Point P1(tmpLine.s[0],tmpLine.s[1]);
		cv::Point P2(tmpLine.e[0],tmpLine.e[1]);
		
		cv::Point up   = (P1.y > P2.y) ? P2 : P1;
		cv::Point down = (P1.y < P2.y) ? P2 : P1;

// 		double verLen = tmpLine.len;
		if (pointPolygonTest(fieldHull, up, true) > MinNearFieldUpPoint){continue;}

		
		
		
		
		//fine tune

		cv::LineIterator ittmpLine(rawHSV, down, up, 8);
		cv::Point lastDown = down;
		int btnCounter = 0;
		for (int k = 0; k < ittmpLine.count; k++, ++ittmpLine)
		{

			cv::Vec3b hsvC = (cv::Vec3b) *ittmpLine;

			if (hsvC[0] >= params.goalhsv.h0->get()
					&& hsvC[0] <= params.goalhsv.h1->get()
					&& hsvC[1] >= params.goalhsv.s0->get()
					&& hsvC[1] <= params.goalhsv.s1->get()
					&& hsvC[2] >= params.goalhsv.v0->get()
					&& hsvC[2] <= params.goalhsv.v1->get())
			{
				if (btnCounter == 0)
				{
					lastDown = ittmpLine.pos();
				}
				btnCounter++;
				if (btnCounter > params.goal.minContinuesColor->get())
				{
					down = lastDown;
					break;
				}
			}
			else
			{
				btnCounter = 0;
			}

		}

		if (down == lastDown)
		{
			btnCounter = 0;
			cv::LineIterator ittmpLine2(rawHSV, down, up, 8);
			for (int k = 0; k < ittmpLine2.count; k++, ++ittmpLine2)
			{

				cv::Vec3b hsvC = (cv::Vec3b) *ittmpLine2;

				if (hsvC[0] >= params.fieldhsv.h0->get()
						&& hsvC[0] <= params.fieldhsv.h1->get()
						&& hsvC[1] >= params.fieldhsv.s0->get()
						&& hsvC[1] <= params.fieldhsv.s1->get()
						&& hsvC[2] >= params.fieldhsv.v0->get()
						&& hsvC[2] <= params.fieldhsv.v1->get())
				{
					btnCounter = 0;
				}
				else
				{
					if (btnCounter == 0)
					{
						lastDown = ittmpLine2.pos();
					}
					btnCounter++;
					if (btnCounter
							> params.goal.minContinuesColor->get())
					{
						break;
					}
				}

			}
		}
		
			
// 		cv::Point2f downReal;
// 		if (!projection.GetOnRealCordinate(down, downReal))
// 		{
// 			ROS_ERROR("Erorr in programming!");
// 			return false;
// 		}
// 
// 		if (!checkDistance_Box(downReal, verLen, params.goal))
// 			continue;
		double downDistance2Field = pointPolygonTest(fieldHull, down,
				true);
		if (downDistance2Field < MaxOutField) //down point should be near inside
			continue;
// 		if(GetDistance(downReal)>5)
// 				continue;
// 		goalPosition.push_back(downReal);
		finalPosts.push_back(Line(0,Vec2i(down.x, down.y), Vec2i(up.x, up.y)));


		
		
		
		
			  			  
// 		if((down.y-up.y)!=0){
// 		  
// 		    cv::LineIterator ittmpLine(rawHSV, down, up, 8);
// 		    int count1 =0; int count2=0;
// 		    for (int k = 0; k < ittmpLine.count; k++, ++ittmpLine)
// 		    {
// 			  count1++;
// 			  uchar BinaryC =  *(*ittmpLine);
// 			    if (BinaryC>0)
// 			    {
// 				    count2++;
// 			    }
// 
// 		    }
// 		    //get enouth pct of white color
// 		    if (float(count2)/float(count1) *100 > params.goal.minContinuesColor->get()){ 
// // 				  if (1){ 
// 			while(true)
// 		      {
// // 				uchar goalB = goalBinaryImg.at(down);
// 			      if (goalBinaryImg.data[down.y*W + down.x] >0)
// 			      {
// 				      if(down.y + 1 < H){ down = cv::Point( down.x, down.y + 1);}
// 				      else{break;}
// 			      }
// 			      else
// 			      {
// 				      break;
// 			      }
// 		      }
// 		      
// 
// 		      while(true)
// 		      {
// 
// // 				uchar goalBUp = goalBinaryImg.at(up);
// 			      if (goalBinaryImg.data[up.y*W + up.x  ] >0)
// 			      {
// 				      if(down.y -1  > 0){ up = cv::Point( up.x, up.y - 1);}
// 				      else{break;}
// 			      }
// 			      else
// 			      {
// 				      break;
// 			      }
// 		      }
// 		    
// 		    
// 		    double downDistance2Field = pointPolygonTest(fieldHull, down,true);
// 		    if (downDistance2Field < MaxOutField) //down point should be near inside
// 			continue;
// 		    
// 		    finalPosts.push_back(Line(0,Vec2i(down.x, down.y), Vec2i(up.x, up.y)));
// 		    
// 		    
// 		    
// 		    } 
// 		      
// 		      
// 		}
		      
		

		
	}
  
  
          

    Mat allVerLines2Img(H,W,CV_8UC3,cv::Scalar(150, 150, 150));
    drawContours(allVerLines2Img, tmphulls, -1,  cv::Scalar(200, 0, 50), 2, 8);
    Mat finalPostsImg(H,W,CV_8UC3,cv::Scalar(150, 150, 150));
    drawContours(finalPostsImg, tmphulls, -1,  cv::Scalar(200, 0, 50), 2, 8);
    
//          cout<<"allVerLines.size    "<<allVerLines.size()<<endl;
    
    	for(size_t k =0; k< allVerLines.size(); ++k ){
	  cv::Point p1( allVerLines[k].s[0], allVerLines[k].s[1]);
	  cv::Point p2( allVerLines[k].e[0], allVerLines[k].e[1]);
	  if(   p1.x >= 0 && p1.y >= 0 && p1.x < W && p1.y < H
	     && p2.x >= 0 && p2.y >= 0 && p2.x < W && p2.y < H ){
	      cv::line( allVerLinesImg, p1, p2, cv::Scalar( 199, 100, 0), 2 );
	    
	  }
	  
	}
    
    
     	for(size_t k =0; k< allVerLines2.size(); ++k ){
	  cv::Point p1( allVerLines2[k].s[0], allVerLines2[k].s[1]);
	  cv::Point p2( allVerLines2[k].e[0], allVerLines2[k].e[1]);
	  if(   p1.x >= 0 && p1.y >= 0 && p1.x < W && p1.y < H
	     && p2.x >= 0 && p2.y >= 0 && p2.x < W && p2.y < H ){
	      cv::line( allVerLines2Img, p1, p2, cv::Scalar( 199, 100, 0), 1 );
	    
	  }
	  
	}
    
       for(size_t k =0; k< finalPosts.size(); ++k ){
	  cv::Point p1( finalPosts[k].s[0], finalPosts[k].s[1]);
	  cv::Point p2( finalPosts[k].e[0], finalPosts[k].e[1]);
	  if(   p1.x >= 0 && p1.y >= 0 && p1.x < W && p1.y < H
	     && p2.x >= 0 && p2.y >= 0 && p2.x < W && p2.y < H ){
	      cv::line( finalPostsImg, p1, p2, cv::Scalar( 1, 100, 100), 2 );
	    
	  }
	  
	}
    
    
    
    
  
  
    imshow("cannyImg", cannyImg );
    waitKey(1);
  
  
    imshow("goalBinaryImg", goalBinaryImg );
    waitKey(1);
    
    imshow("allVerLinesImg", allVerLinesImg );
    waitKey(1);
    
    imshow("allVerLines2Img", allVerLines2Img );
    waitKey(1);
    
    imshow("finalPosts", finalPostsImg );
    waitKey(1);
    

/*    
    imshow("binaryEdge2", binaryEdge2 );
    waitKey(1);
    */
    
    
    
    
//     imshow("goalBinaryImgErode", goalBinaryImgErode );
//     waitKey(1);
//     
//         
//     imshow("goalBinaryImgDilate", goalBinaryImgDilate );
//     waitKey(1);
  
}


