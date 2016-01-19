
// #include "findField.h"
#include <visual_tracking/FieldObjects/findField.h>

  cv::Point2f LineSegment::GetMiddle(){
    return Point2f((P1.x + P2.x) / 2., (P1.y + P2.y) / 2.);
  }
  
  bool LineSegment::SortbyDistance(const Point2f & a, const Point2f &b){
      float x = abs(P1.x - a.x);
      float y = abs(P1.y - a.y);
      float distanceA = sqrt(x * x + y * y);

      x = abs(P1.x - b.x);
      y = abs(P1.y - b.y);
      float distanceB = sqrt(x * x + y * y);

      return distanceA < distanceB;
    }
  vector<cv::Point2f> LineSegment::GetMidPoints( int count /*Means that lines count will be 2^count*/){
	    vector<LineSegment> lsList, lsListTmp;
	    lsList.push_back(LineSegment(P1, P2));
	    vector<Point2f> res;
	    for (int counter = 0; counter < count; counter++)
	    {
		    lsListTmp.clear();
		    for (size_t i = 0; i < lsList.size(); i++)
		    {
			    LineSegment tmp = lsList[i];
			    Point2f midPoint = tmp.GetMiddle();
			    lsListTmp.push_back(LineSegment(tmp.P1, midPoint));
			    lsListTmp.push_back(LineSegment(tmp.P2, midPoint));
		    }
		    lsList = lsListTmp;
	    }

	    for (size_t i = 0; i < lsList.size(); i++)
	    {
		    res.push_back(lsList[i].P1);
		    res.push_back(lsList[i].P2);
	    }


		    sort(res.begin(), res.end(),bind(&LineSegment::SortbyDistance, this, _1, _2));

	    return res;
    }



FindField::FindField(){
    H = params.camera.height->get();
    W = params.camera.width->get();
    m_Top = 0;
    MAX_DESTANCE_FROM_BOTTOM_OF_IMAGE= params.field.MaxDistanceFromBottomOfImage->get();

}




bool FindField::FindFieldConvexHull(/* in */  cv::Mat GreenBinary, /* out */Mat &fieldConvectHullMat,vector<cv::Point> &fieldConvexHullPoints,/* out */int&  m_Top ){
   fieldConvexHullPoints.clear();
   vector<cv::Point> fieldConvexHullPointsUndistort;
   MAX_DESTANCE_FROM_BOTTOM_OF_IMAGE= params.field.MaxDistanceFromBottomOfImage->get();
   //generating the binary image of green pixels
//     Mat fieldBinary(H,W,CV_8UC1);
    
// 	memcpy(fieldBinary.data, greenPixels.data(),greenPixels.size()*sizeof(int));   
//       for ( int j =0; j< H; j++){
// 	for ( int i = 0; i< W; i++){  
// 	   if(GreenBinary.at<uchar>(j,i)== 255){
// 	     fieldBinary.at<uchar>(j,i) = 1;
// 	   } 
// 	   else{fieldBinary.at<uchar>(j,i) = 0;}
// 	  }	     
// 	}
	
//        Mat imgToshow(H,W,CV_8UC3,cv::Scalar(150, 150, 150));
      
      //finding all contours on this binary image
      vector<vector<cv::Point> > allContours;
      vector<Vec4i> hierarchy;

      cv::findContours( GreenBinary.clone(), allContours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
//       drawContours(imgToshow, allContours, -1,  cv::Scalar(200, 0, 50), 1, 8);
//       vector<vector<cv::Point> > ChosedContours;
	    
      //sort the contours into descending order of the area.
      std::sort(allContours.begin(), allContours.end(), bind(&FindField::SortFuncDescending, this, _1, _2));
      bool ret = false;
      int totalResult = 0;
      vector<cv::Point> resPoints;
	
	for (size_t i = 0; i < allContours.size(); i++) // iterate through each contour.
	{
		if (totalResult >= params.field.maxContourCount->get())
		{
			break;
		}
		Rect rec = boundingRect(allContours[i]);
		double area = contourArea(allContours[i]);
		if (std::abs(H - Bottom(rec))
				<= MAX_DESTANCE_FROM_BOTTOM_OF_IMAGE
				&& area >= params.field.minArea->get())
		{
			vector<cv::Point> tmpContour = allContours[i];
			approxPolyDP(tmpContour, tmpContour,
					cv::arcLength(tmpContour, true) * 0.003, true);
// 			ChosedContours.push_back(tmpContour);
			for (size_t pIt = 0; pIt < tmpContour.size(); pIt++)
			{
				resPoints.push_back(tmpContour[pIt]);
			}

			ret = true;
		}
		totalResult++;
	}
//         drawContours(imgToshow, ChosedContours, -1,  cv::Scalar(0, 200, 50), 2, 8);
	
// 	convexHull(resPoints,fieldConvexHullPoints, false);

        if(ret == false){ return false;}
	
	vector<cv::Point> fieldContourUndistort;
	distortionModel.UndistortP(resPoints, fieldContourUndistort);
      
	cv::convexHull(fieldContourUndistort,fieldConvexHullPointsUndistort, false);


        vector<cv::Point>  hullUndistortMidP;
	for (size_t i = 0; i < fieldConvexHullPointsUndistort.size(); i++)
	{
		size_t cur = i;
		size_t next = (i >= fieldConvexHullPointsUndistort.size() - 1) ? 0 : i + 1;
		LineSegment ls(fieldConvexHullPointsUndistort[cur], fieldConvexHullPointsUndistort[next]);
		vector<Point2f> resMP = ls.GetMidPoints(4); //16 points
		for (size_t j = 0; j < resMP.size(); j++)
		{
			hullUndistortMidP.push_back(cv::Point(resMP[j].x, resMP[j].y));

		}
	}
	
	distortionModel.DistortP(hullUndistortMidP, fieldConvexHullPoints);

      
      

      m_Top = boundingRect(fieldConvexHullPoints).y;

      
      vector<vector<cv::Point> > hulls = vector<vector<cv::Point> >(1,fieldConvexHullPoints);
      
      fieldConvectHullMat = Mat::zeros(GreenBinary.size(), CV_8UC1);
      
 
//       CV_FILLED:the contour interiors are drawn
//       drawContours(fieldConvectHullMat, hulls, -1, cv::Scalar(200, 0, 50),CV_FILLED, 8); // Draw the convexhull of the field
       drawContours(fieldConvectHullMat, hulls, -1, cv::Scalar(255),CV_FILLED, 8); // Draw the convexhull of the field
//       
//       cout<<int(fieldConvectHullMat.at<uchar>(0,0) )<<"   "<<int(fieldConvectHullMat.at<uchar>(400,100))<<endl;
//       cv::imshow("undistImg" ,distImg);
//       cv::waitKey(1);
      
//      cv::imshow("imgToshow",imgToshow);
// 	cv::waitKey(1);

     return true;
  
}


void FindField::ColorClassification(const Mat &srcHsvImg, const Mat &tmplateGrayImg,Mat *dstGrayImgs, hsvRangeC *ranges, bool *inTemplate, int size)
{
	const int srcSize = srcHsvImg.rows * srcHsvImg.cols;
	int* indexs = new int[4];
	for (int k = 0; k < size; k++)
	{
		indexs[k] = 0;
	}
	uchar* srcHsvImg_D = srcHsvImg.data;
	uchar* tmplateGrayImg_D = tmplateGrayImg.data;

	for (int i = 0; i < srcSize; i++)
	{
		ushort h = srcHsvImg_D[0], s = srcHsvImg_D[1], v = srcHsvImg_D[2];
		if (tmplateGrayImg_D[0] >= 254)
		{
			for (int k = 0; k < size; k++)
			{

				if (h >= ranges[k].h0->get() && h <= ranges[k].h1->get()
						&& s >= ranges[k].s0->get() && s <= ranges[k].s1->get()
						&& v >= ranges[k].v0->get() && v <= ranges[k].v1->get())
				{
					dstGrayImgs[k].data[indexs[k]] = 255;
				}
				else
				{
					dstGrayImgs[k].data[indexs[k]] = 0;
				}
			}
		}
		else
		{
			for (int k = 0; k < size; k++)
			{

				if (inTemplate[k])
					continue;
				if (h >= ranges[k].h0->get() && h <= ranges[k].h1->get()
						&& s >= ranges[k].s0->get() && s <= ranges[k].s1->get()
						&& v >= ranges[k].v0->get() && v <= ranges[k].v1->get())
				{
					dstGrayImgs[k].data[indexs[k]] = 255;
				}
				else
				{
					dstGrayImgs[k].data[indexs[k]] = 0;
				}
			}
		}

		tmplateGrayImg_D += 1;
		srcHsvImg_D += 3;
		for (int k = 0; k < size; k++)
		{
			indexs[k]++;
		}
	}

	delete[] indexs;
}





  
    // *************************** //
    // * Private Utility Methods * //
    // *************************** //

    
    
//by Hafez
bool FindField::SortFuncDescending(vector<cv::Point> i, vector<cv::Point> j){
	return contourArea(i, false) > contourArea(j, false);
}

int FindField::Bottom(Rect rec) {

  return rec.y + rec.height;
}
    
    
// // ****************************************************************************** //
//     
// void FindField:: bresenham(/* int */ int x1, int y1, int x2, int y2, /* out */std::vector< int > & m_FieldBoundary )
// // ****************************************************************************** //
//     {
// 	// * Use Bresenham algorithm to generate straight lines. * //
// 
// // 	int slope;
// 	int dx, dy, x;
// 	// Reverse lines where x1 > x2
// 	if ( x1 > x2 ) {
// 	    bresenham( x2, y2, x1, y1,  m_FieldBoundary);
// 	    return;
// 	}
// 	dx = x2 - x1;
// 	dy = y2 - y1;
// 	
// 	// Blit
// 	for ( x = x1; x <= x2; x++ ) {
//             
// 	    m_FieldBoundary[ x ] = y1 + (x- x1)* double(dy)/double(dx);
// //            cout<<x<<"  "<<  m_FieldBoundary[ x ] <<endl;
// 	}
// 	
// 	return;
// 
//     }; // END of bresenham
//     
//     
 
 
      
      

    