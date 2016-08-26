/*

Authors: Yongfeng

*/

#ifndef FINDNODES_H
#define FINDNODES_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>    // std::min_element, std::max_element
#include <vector>
#include <math.h>
#include <boost/timer/timer.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <queue>
#include <utility>      // std::pair
#include <limits>
#include <sstream>
#include <fstream>      // std::fstream

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <time.h>       /* time */


#include <visual_tracking/frameGrabber.h>
#include <visual_tracking/Tools/tools.h>
#include <visual_tracking/Projection/DistortionModel.h>
#include <visual_tracking/Projection/BackwardProjection.h>
#include <visual_tracking/Tools/m_Math.h>
#include <visual_tracking/Parameters/globaldefinitions.h>
#include <visual_tracking/IcpPointMatcher/kdtree.h>

using namespace std;
using namespace vision;
using namespace cv;



namespace vision
{        
  	class FindNodes
	{ 
	
	public: 	
	    
	    enum{
		    H = ORG_IMAGE_HEIGHT,
		    W = ORG_IMAGE_WIDTH,
		    siX = UNDISTOTED_IMAGE_WIDTH, 
		    siY = UNDISTOTED_IMAGE_HEIGHT,
		    subW = SUB_SAMPLING_WIDTH,
		    subH = SUB_SAMPLING_HEIGHT,
		    MAX_CONN = 4,
		    MAX_CLOSE = 100,
		    MAX_COS = 512,
		    
	    };
	    
	    // * Node Structure * //
	    struct Node {
		  
		    int    x_pos;   //<- pixel-coordinates of the node (integer)
		    int    y_pos;
		    int    undistorted_x_pos;
		    int    undistorted_y_pos;
		    
		    Vec2i  undist_pos;
		    float  lambda;
		    float  f_x_pos; //<- float-coordinates ...
		    float  f_y_pos;

		    int    connected[ MAX_CONN ];  //<- edges of the graph, contains num_connected
//     		        // * indices of nodes (undirected, stored twice)
		    int    num_connected;          //<- number of edges
		    int    close_id [ MAX_CLOSE ]; //<- node-id's of candidates for new connections
		    int    close_adj[ MAX_CLOSE ]; //<- eveluation of candidates for new connections
		    int    num_close;              //<- number of candidates for new connections
		    float  conf;                   // confidence in the crossing_type decision
		    enum CrossingType{
			CT_Unknown=0,
			CT_LXing=1,
			CT_TXing=2,
			CT_XXing=3,
		    };
		      CrossingType    crossing_type;

// 			int    transition_xy[ TRANSITION_TYPE ][ MAX_TRANSITIONS ][ 2 ];
		    bool is_connected_to(int nid){
			    int* begin = connected;
			    int* end   = connected + num_connected;
			    return find(begin,end,nid) != end;
		    }
		    bool is_close_to(int nid){
			    int* begin = close_id;
			    int* end   = close_id + num_close;
			    return find(begin,end,nid) != end;
		    }
		    
		    int get_close_idx(int nid){
			    int k_id = -1;
			    for(int k=0; k<num_close; ++k ){
			        if(close_id [ k ]  ==  nid ){k_id = k; break;}
			    }
			    return k_id;
			    
		    }
		    
		    

	    };
	      
	      int UNIT;
	      int m_Top;
	    
	    // * Node_Buffer Vector * //
	    typedef std::vector< Node > Node_Buffer;
	    Node_Buffer * m_NodeBuffer;
	    Node_Buffer *m_NodeBuffer_tmp;
	    // * Trans_Buffer Structure * //
	    struct Trans_Buffer {

		int arc_cos_sqare[ MAX_COS ]; //<- acos lookup table.

	    };

	    Trans_Buffer * m_TransBuffer;
	    
	    LinearGraph_Buffer  * m_LinearGraph_Buffer;
	    
	    
// 	    std::vector<cv::Point> NodeSamplePoins;
	    std::vector<cv::Point> undistortedNodeSamplePoins;

	    vector<cv::Scalar> colorPool;

	    
	      FindNodes();
	  
	      ~FindNodes(){
		  m_NodeBuffer->clear();
		  m_NodeBuffer_tmp->clear();
		  m_LinearGraph_Buffer->clear();
		  delete m_NodeBuffer;
		  delete m_NodeBuffer_tmp;
		  delete m_LinearGraph_Buffer;
		  delete m_TransBuffer;
	      };
	    
	      
	      
	      
	    Rectangle_Buffer Rectangles;
	    void getRectangle( Rectangle_Buffer &_Rectangles  );
	    void findNodeGraph(FrameGrabber & CamFrm);
	    void findNodes( /* in_out */ Rectangle_Buffer &_Rectangles /* OUT Node_Buffer * m_NodeBuffer, */ );
	    void FindCloseNodes2 ( /* in */ const float ( & weightedWhiteValues )[ H ][ W ] /* IN_OUT Node_Buffer * m_NodeBuffer, */ );
	    void ConnectCloseNodes2 ( /* in */ const float ( & weightedWhiteValues )[ H ][ W ]/* IN const Trans_Buffer * m_TransBuffer, */ /* IN_OUT Node_Buffer * m_NodeBuffer, */ );
	    void SmoothNodes2 ( /* IN_OUT Node_Buffer * m_NodeBuffer, */ );
	    void findReachableNodes2(Node_Buffer * nodeBuffer);
	    
	    void connectComps();
	    void connectSingleNodes();
	    void updateUndistoredLines();
	    void updateNodeAngle();
	    void updateNodeTangentLine();
	    void updateNodeWeights();
	    void MergeMoreComps();
	    void removeSmallComp ();
	    void updateCompPoints();

	
	
	private:
            void init( /* OUT Trans_Buffer * m_TransBuffer */ );
	    bool is_connected ( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j );
	    bool connect_nodes ( /* IN_OUT Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j );
	    bool disconnect_nodes ( /* IN_OUT Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j ,Node_Buffer * nodeBuffer);
	    int angle_nodes ( /* IN const Trans_Buffer * m_TransBuffer, */ /* IN const Node_Buffer *  m_NodeBuffer, */ /* in */ const int i, /* in */ const int j, /* in */ const int k );// 0~180
	    float f_angle_nodes ( /* IN const Trans_Buffer * m_TransBuffer, */ /* IN const Node_Buffer *  m_NodeBuffer, */ /* in */ const int i, /* in */ const int j, /* in */ const int k );
	 
	    int distance_nodes ( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j );
	    float f_distance_nodes( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j ,Node_Buffer * nodeBuffer);
	    int closes_loop ( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j );
	    int parallelogram_closes_loop ( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j );
	    int black_line_value ( /* in */ const float ( & weightedWhiteValues )[ H ][ W ], /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j );
	    int black_line_value2 ( /* in */ const float ( & weightedWhiteValues )[ H ][ W ], /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j );
	    void  update_close_nodes( /* in */ const float ( & weightedWhiteValues )[ H ][ W ], /* IN const Trans_Buffer * m_TransBuffer, */ /* IN_OUT Node_Buffer * m_NodeBuffer, */ /* in */ const int i );
	    void  update_close_nodes2( /* in */ const float ( & weightedWhiteValues )[ H ][ W ], /* IN const Trans_Buffer * m_TransBuffer, */ /* IN_OUT Node_Buffer * m_NodeBuffer, */ /* in */ const int i );
	    bool intersect( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int a, /* in */ const int b, /* in */ const int c, /* in */ const int d, /* out */ float * x, /* out */ float * y );
	    void PrintNodeBuffer (string filename, Node_Buffer * nodeBuffer);
	    void PrintReacherableComp (string filename);
	    bool is_reacherable( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j );
	    void update_reachable(  const int i, /* in */ const int j );
	    
	    int distance_nodes2 ( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j );
	    float f_distance_nodes2( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j ,Node_Buffer * nodeBuffer);
	    
	    
	};
	
	
	
}


#endif