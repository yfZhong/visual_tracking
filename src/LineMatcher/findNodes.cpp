
/*

Authors: Yongfeng

*/
// #include "findNodes.h"
#include <visual_tracking/LineMatcher/findNodes.h>


  FindNodes::FindNodes(){
      m_NodeBuffer  = new Node_Buffer;
      m_NodeBuffer_tmp  = new Node_Buffer;
      m_LinearGraph_Buffer  = new LinearGraph_Buffer;
      m_TransBuffer = new Trans_Buffer;
      init();
      srand((unsigned)time(NULL));
      
//       for(int i=0; i<10; ++i){
      colorPool.clear();
	      colorPool.push_back(cv::Scalar(255, 0, 0));
	      colorPool.push_back(cv::Scalar(0, 255, 0));
	      colorPool.push_back(cv::Scalar(0, 0, 255));
	      colorPool.push_back(cv::Scalar(255, 255, 0));
	      colorPool.push_back(cv::Scalar(255, 0, 255));
	      colorPool.push_back(cv::Scalar(0, 255, 255));
	      
	      colorPool.push_back(cv::Scalar(100, 100, 0));
	      colorPool.push_back(cv::Scalar(100, 0, 100));
	      colorPool.push_back(cv::Scalar(0, 100, 100));
	      colorPool.push_back(cv::Scalar(0, 0, 0));
	      
	      
	      colorPool.push_back(cv::Scalar(100, 800, 0));
	      colorPool.push_back(cv::Scalar(0, 100, 60));
	      colorPool.push_back(cv::Scalar(0, 0, 100));
	      colorPool.push_back(cv::Scalar(100, 150, 700));
	      colorPool.push_back(cv::Scalar(150, 0, 10));
	      colorPool.push_back(cv::Scalar(0, 200, 100));
	      
	      colorPool.push_back(cv::Scalar(100, 20, 70));
	      colorPool.push_back(cv::Scalar(10, 0, 100));
	      colorPool.push_back(cv::Scalar(0, 10, 250));
	      colorPool.push_back(cv::Scalar(0, 90, 300));
	      
	
// 	      float r =  rand() *255; 
// 	      float g =  rand() *255;
// 	      float b =  rand() *255;
// 	      colorPool.push_back(cv::Scalar(r,g,b));
//       }
      
	m_Top = H-1;

  };






void FindNodes::getRectangle( Rectangle_Buffer &_Rectangles  ){
  
     Rectangles = _Rectangles;
  
}

void FindNodes::findNodeGraph(FrameGrabber & CamFrm){
      m_Top =  CamFrm.m_Top ;

      m_NodeBuffer->clear( ); 
      
      //initialize nodes as the centers of rectangles
      findNodes( Rectangles  );
      //find the candidate connections for each nodes and weight them
      FindCloseNodes2 (CamFrm.weightedWhiteValues );
      //iteratively connect the candidate connection that has largest weight
      ConnectCloseNodes2 (CamFrm.weightedWhiteValues );
      //smooth the nodes according to their neighborhoods
      SmoothNodes2();
      
//        PrintNodeBuffer("NodeBuffer.dat", m_NodeBuffer );
      
      
      // make a copy of current node graph
      m_NodeBuffer_tmp->clear();
      Node_Buffer::iterator it;
      
      for ( it = m_NodeBuffer->begin( ); it != m_NodeBuffer->end(); it++ ) { // [1]
	  Node node= (*it);
	  m_NodeBuffer_tmp->push_back(node);

      }
       m_LinearGraph_Buffer->clear();
      
       
       
      // cluster the node clusters and save the clusters in m_LinearGraph_Buffer
      findReachableNodes2(m_NodeBuffer_tmp);
      std::sort(m_LinearGraph_Buffer->begin(), m_LinearGraph_Buffer->end(),  
      boost::bind(&LinearGraphComponent::sumOfLength, _1) > boost::bind(&LinearGraphComponent::sumOfLength, _2));
    
      //try to connect more nodes
      connectSingleNodes();
      
      
      //calculating and saving some usefull information of the line clusters
      updateUndistoredLines();

      updateNodeAngle();
      
      updateNodeTangentLine();
      
      updateNodeWeights();
      
      MergeMoreComps();
      
      removeSmallComp();
      
      updateCompPoints();
      
// 	 PrintReacherableComp("reachableComp");

  
}

void FindNodes::findNodes( /* in_out */ Rectangle_Buffer & Rectangles /* OUT Node_Buffer * m_NodeBuffer, */ ){

  
    for(unsigned int i=0; i<  Rectangles.size(); i++){
		  Node node;
		  node.x_pos = Rectangles[i].avg_x;
		  node.y_pos = Rectangles[i].avg_y ;
		  node.undistorted_x_pos=  Rectangles[i].undistorted_x_pos;
		  node.undistorted_y_pos=  Rectangles[i].undistorted_y_pos; 
		  node.f_x_pos = Rectangles[i].avg_x;
		  node.f_y_pos = Rectangles[i].avg_y ;

		  node.num_connected = 0;
		  node.num_close = 0;
		  node.crossing_type = Node::CT_Unknown;
		  m_NodeBuffer->push_back(node);
    }

}



// ****************************************************************************** //
void FindNodes::FindCloseNodes2 ( /* in */const float ( & weightedWhiteValues )[ H ][ W ] /* IN const Trans_Buffer * m_TransBuffer, */ /* IN_OUT Node_Buffer * m_NodeBuffer, */ )
// ****************************************************************************** //
{
  

    int i = -1, j = -1;
    int rad;
//     int fac; //<- Indicates the size of the gaps, which may be closed.
    int ang, dist;
    
    rad = max<int>( 5, params.graphNode._NeighborRadius->get() );
    int MaxCloseNum = params.graphNode._MaxCloseNum->get();
    int  MinAngle = params.graphNode._MinAngle->get();
    
    
    Node_Buffer::iterator it;
    for ( it = m_NodeBuffer->begin( ); it != m_NodeBuffer->end( ); it++ ) { // [0]
	i++;
	j = -1;

	it->num_close = 0;

	// * Skip all nodes with degree >2. * //
	if ( it->num_connected > 2 ) { // [1]
	    continue;
	}
// 	if ( it->num_connected > MAX_CONN ) { // [1]
// 	    continue;
// 	}
// 	
	// * Continue, if degree(node)=2 AND angle too large. * //
	if ( ( it->num_connected == 2 ) &&
	    ( angle_nodes( it->connected[ 0 ], i, it->connected[ 1 ] ) > 90 ) ) { // [1]
		continue;
	}

	// * Examine all other nodes. * //
	Node_Buffer::iterator it2;
	for ( it2 = m_NodeBuffer->begin( ); it2 != m_NodeBuffer->end( ); it2++ ) { // [1*]
	    j++;

	    if ( it2 == it ) { // [2]
		continue;
	    }
	    if ( it2->num_connected == MAX_CONN ) { // [2]
		continue;
	    }

	    // * If distance between node_i and node_j is too large or if they are connected, continue. * //
	   
	    dist = distance_nodes2( i, j );
// 	    float dist_closest =  m_Math::getTwoRectClosestDist(Rectangles[i],Rectangles[j]);
	    float dist_closest = dist - std::max(Rectangles[i].rect.width /2, Rectangles[i].rect.height/2) 
	                                       - std::max(Rectangles[j].rect.width /2, Rectangles[j].rect.height/2);
	    if(dist_closest<0){ dist_closest =0; }
// 	    dist = distance_nodes( i, j );
	  
	     
	    float rad_tmp = rad;
	    if(it->y_pos < m_Top + 0.3*(H-m_Top) ){rad_tmp -= 3;}
// 	    if ( dist > rad_tmp ) { // [2]
// 		continue;
// 	    }
	    
	    if ( dist_closest > rad_tmp ) { // [2]
		continue;
	    }
	    
	    if ( is_connected( i, j ) ) { // [2]
		continue;
	    }
	    // * If a small loop would be produced, continue. * //
	    if ( closes_loop( i, j ) ) { // [2]
		continue;
	    }
	    if ( parallelogram_closes_loop( i, j ) ) { // [2]
		continue;
	    }	    
	    
	    
	    
	    if ( it->num_connected == 1 ) { // [2]
		// * If angle (node_k->node-i) with new edge (node_i->node-j) is too small, continue. * //
		if ( ( ang = angle_nodes( it->connected[ 0 ], i, j ) ) < MinAngle ) { // [2]
		    continue;
		}
	    }
	    else if ( it->num_connected == 2 ) { // [2]
		// * If angle MIN((node_k1->node-i), (node_k2->node-i)) with new edge
		// * (node_i->node-j) is too small, continue. * //
		if ( ( ang = min( angle_nodes( it->connected[ 0 ], i, j ), angle_nodes( it->connected[ 1 ], i, j ) ) ) < MinAngle ) { // [3]
			continue;
		}

	    }
// 	    else if ( it->num_connected == 3 ) { // [2]
// 		// * If angle MIN((node_k1->node-i), (node_k2->node-i)) with new edge
// 		// * (node_i->node-j) is too small, continue. * //
// 		if ( ( ang = min(angle_nodes( it->connected[ 0 ], i, j ), min( angle_nodes( it->connected[ 1 ], i, j ), angle_nodes( it->connected[ 2 ], i, j ) ) )) < MinAngle ) { // [3]
// 			continue;
// 		}
// 
// 	    }
	    
	    else { // [2]
		ang = 120;
	    }
	    
// 	    float dist =std::max(double (m_Math::getTwoRectClosestDist(Rectangles[i], Rectangles[j])), 0.5);

	    float areaj = Rectangles[j].area ;
	    float num_pointsj = Rectangles[j].num_points ;
	    
	    float areai = Rectangles[i].area ;
	    float num_pointsi = Rectangles[i].num_points ;

	   
// 	    if((areai > 40 || num_pointsi >10) && (areaj > 40 || num_pointsj >10)){ dist /=4.0; }
// 	    if((areai > 40 || num_pointsi >10) || (areaj > 40 || num_pointsj >10)){ dist /=2.0; }
	    
// 	    else if(area > 25 || num_points >6 ){ dist /=3.0; }
	    

	    it->close_id[ it->num_close ] = j;
// 	    it->close_adj[ it->num_close ] = /*fac **/ ( ang - MinAngle ) * black_line_value( weightedWhiteValues, i, j ) / ( dist + 1 );//prefer large ang, small dist, high brightness

//          it->close_adj[ it->num_close ] = num_pointsi * areai *num_pointsj * areaj * ( ang - MinAngle ) 
// 	                                  * black_line_value2( weightedWhiteValues, i, j ) / ( dist + 1  );
					  
	    //weight for this candidate connection
            it->close_adj[ it->num_close ] = num_pointsi * areai *num_pointsj * areaj * ( ang - MinAngle ) 
	                                  * black_line_value2( weightedWhiteValues, i, j ) / ( dist_closest + 1  ) *0.0001;
// 					  //prefer large ang, small dist, high brightness
            if(  it->close_adj[ it->num_close ]<0){ it->close_adj[ it->num_close ] =  std::numeric_limits<int>::max(); }			  		  
	    
	    it->num_close++;

	    if ( it->num_close == MaxCloseNum ) { // [2]
		break;
	    }

	} // [1*]

    } // [0]

    return;

}; // END of FindCloseNodes METHOD


// ****************************************************************************** //

void FindNodes::ConnectCloseNodes2 ( /* in */ const float ( & weightedWhiteValues )[ H ][ W ]/* IN const Trans_Buffer * m_TransBuffer, */ /* IN_OUT Node_Buffer * m_NodeBuffer, */ )
// ****************************************************************************** //
{

    int min_adj,adj, max_adj;
    int i, j, k, l, max_i = 0, max_j = 0;
    int num_j, angle, max_angle, min_angle;
    int iteration = 0;
//     int num_ends = 0;
    
     min_adj = params.graphNode._Min_adj->get();
     int  MinAngle = params.graphNode._MinAngle->get();
    // * compute the number of open ends. * //
     Node_Buffer::iterator it;
     
     int iter_count =0;
     do {
       
        iter_count++;
	min_adj = max(1, min_adj);
	max_adj = min_adj;
        
	i = -1;
	// * for all nodes ... * //
	for ( it = m_NodeBuffer->begin( ); it != m_NodeBuffer->end( ); it++ ) {
	    i++;
             
	    if(it->num_connected == MAX_CONN){continue;}
	    // * for all candidates ... * //
	    for ( k = 0; k < it->num_close; k++ ) {

		j = it->close_id[ k ];
		adj = it->close_adj[ k ];
//                 cout<<"adj: "<< adj<<endl;
		num_j = ( * m_NodeBuffer )[ j ].num_connected;

// 		if ( num_j != 2 ) {
// 		    adj += adj / 2;
// 		}
// 		if ( num_j < 2 ) {
// 		    adj += adj / 2;
// 		}
// 		
		
		max_angle = 90;
		min_angle = 90;

		for ( l = 0; l < num_j; l++ ) {
		    angle = angle_nodes( i, j, ( * m_NodeBuffer )[ j ].connected[ l ] );
		    if ( angle > max_angle ) {
			max_angle = angle;
		    }
		    else if ( angle < min_angle ) {
			min_angle = angle;
		    }
		}
// 		adj += ( adj * ( max_angle - 90 ) ) / 90;
// 		adj += ( adj * min_angle ) / 90;
		
// 		adj += ( adj * ( 180-max_angle  ) ) / 90;
// 		adj += ( adj * min_angle ) / 90;

                if(min_angle <MinAngle ){continue;}
		if ( closes_loop( i, j ) ) {
		    continue;
		}
	        if ( parallelogram_closes_loop( i, j ) ) { // [2]
		    continue;
	        }
		
		if(  it->num_connected==2 && ( * m_NodeBuffer )[ j ].num_connected ==2){
		      if(
			   angle_nodes(  ( * m_NodeBuffer )[ i ].connected[ 0 ] , i , ( * m_NodeBuffer )[ i ].connected[ 1 ]) >160
			&& angle_nodes(  ( * m_NodeBuffer )[ j ].connected[ 0 ] ,j,( * m_NodeBuffer )[ j].connected[ 1 ]) >160
			&& abs(angle_nodes(  ( * m_NodeBuffer )[ i ].connected[ 0 ] ,i, j) -90) <20
			&& abs(angle_nodes(  ( * m_NodeBuffer )[ j ].connected[ 0 ] ,j, i) -90) <20
		      ){
    		        
			  continue;
// 			  adj -= adj / 2;
		    
		      }
		}
//                 cout<<adj<<endl;
		if ( adj > max_adj ) {
		    max_adj = adj, max_i = i, max_j = j;
		}
	    }	    
	    
	}
//  cout<<"                                             "<<max_adj<<endl;
//         cout<<"max_adj       "<<max_adj<<endl;
	
	// * If the best candidate is good enough, insert it. * //
	if ( max_adj > min_adj ) {

// 	    // *  update the number of open ends. * //
// 	    if ( ( * m_NodeBuffer )[ max_i ].num_connected < 2 ) {
// 		num_ends--;
// 	    }
// 	    if ( ( * m_NodeBuffer )[ max_j ].num_connected < 2 ) {
// 		num_ends--;
// 	    }
	    connect_nodes( max_i, max_j );

	    update_close_nodes2( weightedWhiteValues, max_i );

	    
	    if ( ( * m_NodeBuffer )[ max_j ].num_connected < 4 ) {
		update_close_nodes2( weightedWhiteValues, max_j );
	    }
// 	   if ( ( * m_NodeBuffer )[ max_j ].num_connected < 4 && ( * m_NodeBuffer )[ max_j ].is_close_to(max_i)) {
// 	         int k_j = ( * m_NodeBuffer )[ max_j ].get_close_idx(max_i);
// 		 ( * m_NodeBuffer )[ max_j ].num_close--;
// 	         ( * m_NodeBuffer )[ max_j ].close_id[ k_j ] = ( * m_NodeBuffer )[ max_j ].close_id[ ( * m_NodeBuffer )[ max_j ].num_close ];
// 	    }
	     
	     
	    iteration = 0;
	}
	else {
	    if ( iteration == 0 ) {
		FindCloseNodes2( weightedWhiteValues );
	    }
	    iteration++;
	}
    } while ( (max_adj > min_adj || iteration < 2 ) && iter_count<10000000);
    
//       cout<<"iter_count"<<iter_count<<"            "<<max_adj<<endl;

    return;

}; // END of ConnectCloseNodes METHOD


// ****************************************************************************** //
  void FindNodes::SmoothNodes2 ( /* IN_OUT Node_Buffer * m_NodeBuffer, */ )
  // ****************************************************************************** //
  {
      int i;
      int j, k;
      i=-1;
      Node_Buffer::iterator it;
      for ( it = m_NodeBuffer->begin( ); it != m_NodeBuffer->end( ); it++ ) {
	  if ( it->num_connected > 0){  
	    it->x_pos =  it->f_x_pos;
	    it->y_pos =  it->f_y_pos;
	  }
      }
      
      
      
      for ( it = m_NodeBuffer->begin( ); it != m_NodeBuffer->end( ); it++ ) {
          ++i;
	  
	  if ( it->num_connected != 2  || (it->num_connected==2 &&  (angle_nodes(it->connected[ 0 ], i, it->connected[ 1 ])-90)<40)) {
	      continue;
	  }

	  it->f_x_pos = ( float ) it->x_pos * 2;
	  it->f_y_pos = ( float ) it->y_pos * 2;

	  for ( j = 0; j < it->num_connected; j++ ) {
	      k = it->connected[ j ];

	      it->f_x_pos += ( * m_NodeBuffer )[ k ].x_pos;
	      it->f_y_pos += ( * m_NodeBuffer )[ k ].y_pos;
	  }
      }
      i=-1;
      for ( it = m_NodeBuffer->begin( ); it != m_NodeBuffer->end( ); it++ ) {
          ++i;
	  if ( it->num_connected != 2 || (it->num_connected==2 &&  (angle_nodes(it->connected[ 0 ], i, it->connected[ 1 ])-90)<40)) {
	      continue;
	  }
	  
	  it->f_x_pos /= 4.0;
	  it->f_y_pos /= 4.0;
      }

      return;

  }; // END of SmoothNodes METHOD


void FindNodes::findReachableNodes2( Node_Buffer * nodeBuffer)
{     
     
//       lComp.reachableNodeIds.push_back(startNode);
       int MinAngleForSameComp= params.graphNode._MinAngleForSameComp->get();
       int i=-1; 
       int l_id =0;
       Node_Buffer::iterator it;
       
       int visited[m_NodeBuffer_tmp->size()];
       
       std::fill(visited, visited + m_NodeBuffer_tmp->size(), 0); 
       
       for ( it =  m_NodeBuffer_tmp->begin( ); it !=  m_NodeBuffer_tmp->end(); it++ ) { // [1]
	    ++i;
	      
      
	      int StartIdx1 = i;
	      int StartIdx2 = i;
	      Node& n = (*it);
	      
	      int expandFromDoubleSide = 0;
	      int NextId1, NextId2;
	      
	      LinearGraphComponent  lComp;
	      lComp.sumOfLength=0;
// 	      lComp.sumOfAngle =0;
	      lComp.reachableNodeIds.push_back(StartIdx1);
	      int previousAng = 175;
	      
	      
	      int hasNext=1;
	      
	      if( n.num_connected == 0  && visited[i] ==1 ){continue;}
	      else if(n.num_connected == 0 ){hasNext =0;}
	      else if(n.num_connected ==1){
// 		  expandFromDoubleSide = 0;
		  NextId1 = n.connected[0];
		  
	      }
	      else if(n.num_connected >1){
		    
		    int max_angle = 0;
		    for(int j=0; j<n.num_connected; ++j ){
		      for(int k=j; k<n.num_connected; ++k ){
			  int angle = angle_nodes( n.connected[ j ], StartIdx1, n.connected[ k ] );
			  if(max_angle <angle ){ max_angle = angle; NextId1 = n.connected[ k ],NextId2 = n.connected[ j ];}
			
		      }

		    }
		    if( max_angle >MinAngleForSameComp ){   
		      previousAng = max_angle;
		      expandFromDoubleSide = 1; 
		    }
		    else{
		      NextId1 = n.connected[ 0 ];
		      expandFromDoubleSide = 0;
		    }
	      }
	      
	      visited[StartIdx1]=1;
	      
	      int previousAng1= previousAng;
	      int iter_count =0;
	      while(hasNext >0 &&iter_count <10000){
		    iter_count ++;
		    lComp.reachableNodeIds.push_back(NextId1);
		    visited[NextId1]=1;
		    Vec2i p1( (* m_NodeBuffer_tmp)[StartIdx1].f_x_pos, (* m_NodeBuffer_tmp)[StartIdx1].f_y_pos );
		    Vec2i p2( (* m_NodeBuffer_tmp)[NextId1].f_x_pos, (* m_NodeBuffer_tmp)[NextId1].f_y_pos );
		    
		    lComp.Lines.push_back(Line(l_id++ ,  p1, p2, StartIdx1, NextId1));
// 		    lComp.sumOfLength += f_distance_nodes2(StartIdx1,NextId1,m_NodeBuffer_tmp);
		    lComp.sumOfLength += lComp.Lines.back().len;
// 		    if(lComp.Lines.size>=2){
// 		             lComp.deltaAngle +=  m_Math::AngDif(lComp.Lines.back().ang, lComp.Lines[lComp.Lines.size()].ang);}
	      
		
		    if(  (*m_NodeBuffer_tmp)[NextId1].num_connected == 1){//num_connected >=1
			hasNext =0; 
			disconnect_nodes(StartIdx1, NextId1, m_NodeBuffer_tmp); continue;  }
			
		    int max_angle = 0;
		    
		    
		    Node& n_Tmp = (*m_NodeBuffer_tmp)[NextId1];
		    int max_neighbor_id_1, max_neighbor_id_2;
		    
		    for(int j=0; j<n_Tmp.num_connected; ++j ){
			for(int k= j + 1; k<n_Tmp.num_connected; ++k ){
			    int angle = angle_nodes( n_Tmp.connected[ j ], NextId1, n_Tmp.connected[ k ] );
			    if(max_angle < angle ){ 
				  max_angle = angle; 
				  max_neighbor_id_1 = n_Tmp.connected[ k ];
				  max_neighbor_id_2 = n_Tmp.connected[ j ];
			    }
			}
		      }
		      
		    
		    disconnect_nodes(StartIdx1, NextId1, m_NodeBuffer_tmp);
		    
		    int threshold =MinAngleForSameComp;
// 		    int threshold = std::max(previousAng1-10, MinAngleForSameComp);
		    if( max_angle > threshold ){ 
		      
			  if( max_neighbor_id_1 == StartIdx1 ){  
				StartIdx1 =NextId1;   
				NextId1 = max_neighbor_id_2 ; 
				previousAng1  = max_angle;
			  }
			  else if( max_neighbor_id_2 == StartIdx1){
				StartIdx1 =NextId1;   
				NextId1 = max_neighbor_id_1; 
				previousAng1  = max_angle;
			  }
			  else{ hasNext =0;  }

		    }
		    else{ hasNext =0;  }
		
	      }
		    
		    
	      if(expandFromDoubleSide ==1){
		    std::reverse(lComp.reachableNodeIds.begin(),lComp.reachableNodeIds.end());
		    std::reverse(lComp.Lines.begin(),lComp.Lines.end());
		    int hasNext=1;
		    while(hasNext >0){
		    
			lComp.reachableNodeIds.push_back(NextId2);
			visited[NextId2]=1;
			Vec2i p1( (* m_NodeBuffer_tmp)[StartIdx2].f_x_pos, (* m_NodeBuffer_tmp)[StartIdx2].f_y_pos );
			Vec2i p2( (* m_NodeBuffer_tmp)[NextId2].f_x_pos, (* m_NodeBuffer_tmp)[NextId2].f_y_pos );
			
			lComp.Lines.push_back(Line(l_id++ ,  p1, p2, StartIdx2, NextId2));
// 			lComp.sumOfLength += f_distance_nodes2(StartIdx2,NextId2,m_NodeBuffer_tmp);
			lComp.sumOfLength += lComp.Lines.back().len;
// 		        if(lComp.Lines.size>=2){
// 		             lComp.deltaAngle +=  m_Math::AngDif(lComp.Lines.back().ang, lComp.Lines[lComp.Lines.size()].ang);}
		  
		        Node& n_Tmp = (*m_NodeBuffer_tmp)[NextId2];
			if(  n_Tmp.num_connected == 1){hasNext =0;
			    disconnect_nodes(StartIdx2, NextId2, m_NodeBuffer_tmp);continue;  }
			
			int max_angle = 0;
			
			
			int max_neighbor_id_1, max_neighbor_id_2;
			
			for(int j=0; j<n_Tmp.num_connected; ++j ){
			    for(int k=j +1; k<n_Tmp.num_connected; ++k ){
				int angle = angle_nodes( n_Tmp.connected[ j ], NextId2, n_Tmp.connected[ k ] );
				if(max_angle < angle ){ 
				    max_angle = angle; 
				    max_neighbor_id_1 = n_Tmp.connected[ k ];
				    max_neighbor_id_2 = n_Tmp.connected[ j ];
				}
			    }
			}
			
			
			disconnect_nodes(StartIdx2, NextId2, m_NodeBuffer_tmp);
			
		        int threshold =  MinAngleForSameComp;
// 			int threshold = std::max(previousAng-10, MinAngleForSameComp);
			if( max_angle > threshold ){ 
			  
			      if( max_neighbor_id_1 == StartIdx2 ){  
				    StartIdx2 =NextId2;   
				    NextId2 = max_neighbor_id_2 ; 
				    previousAng  = max_angle;
			      }
			      else if( max_neighbor_id_2 == StartIdx2){
				    StartIdx2 =NextId2;   
				    NextId2 = max_neighbor_id_1; 
				    previousAng  = max_angle;
			      }
			      else{ hasNext =0;  }

		        }//end of if( max_angle > threshold ){
		        else{ hasNext =0;  }
		      
	            } //end of  while(hasNext >0)
		  
	    }//end of  if(expandFromDoubleSide ==1)
	   
	    
	    m_LinearGraph_Buffer->push_back( lComp ); 
      }// end of [1]
}

       
 void FindNodes::connectComps(){
  
   int MaxAngleDiff = params.graphNode._MaxAngleDiff->get();
   float MaxLineSegDistance = params.graphNode._MaxLineSegDistance->get();
   float MaxProjectedDistance = params.graphNode._MaxProjectedDistance->get();
   int Min_angle = params.graphNode._MinAngleForSameComp->get();
//    int SmallAngle = params.graphNode.SmallAngle->get();
   
   bool NoUpdate = false;
   
   int iter_count=0;
   while (!NoUpdate && iter_count<100){
       iter_count++;
//         std::sort(m_LinearGraph_Buffer->begin(), m_LinearGraph_Buffer->end(),  
// 	boost::bind(&LinearGraphComponent::sumOfLength, _1) > boost::bind(&LinearGraphComponent::sumOfLength, _2));
     
	LinearGraph_Buffer::iterator it_1, it_2;
	int changeTimes= 0;
	int connect=0;
	
	
	for ( it_1 =  m_LinearGraph_Buffer->begin( ); it_1 != m_LinearGraph_Buffer->end( ); it_1++ ) { // 
	        if((*it_1).reachableNodeIds.size() <=1){continue;} 
	        
	        vector<int> ends1;
	        ends1.push_back((*it_1).reachableNodeIds.front());
		ends1.push_back((*it_1).reachableNodeIds.back());
		
		for ( it_2 = it_1+1; it_2 != m_LinearGraph_Buffer->end( ); it_2++ ) { // 
		  
		      if((*it_2).reachableNodeIds.size() <=0 ){continue;} 
		      
// 		      if(m_Math::AngDif((*it_1).deltaAngle/(*it_1).reachableNodeIds.size(),   (*it_2).deltaAngle/(*it_2).reachableNodeIds.size())
// 			>MaxAngleDiff){continue;}
		      
		      int overlap=0;
		      for(unsigned int k=1; k<(*it_1).reachableNodeIds.size()-1; k++){
			    for(unsigned int l=1; l<(*it_2).reachableNodeIds.size()-1; l++){
			      if(  (*it_1).reachableNodeIds[k] == (*it_2).reachableNodeIds[l] ){overlap =1;break;}  
			    }
			    if(overlap ==1){break;}
		      }
		      if(overlap==1){ continue; }

		      
		      vector<int> ends2;
		      ends2.push_back((*it_2).reachableNodeIds.front());
		      if((*it_2).reachableNodeIds.size()>=2){ 
			ends2.push_back((*it_2).reachableNodeIds.back());}
		      
		      
			
		      connect = 0;
		      
		      
		      
		      int close_i, close_j; 
		      float dist= std::numeric_limits<float>::max();
		      for(unsigned int i =0; i< ends1.size(); ++i){
			
// 			     Vec2i P1( (* m_NodeBuffer)[ends1[i]].f_x_pos, (* m_NodeBuffer)[ends1[i]].f_y_pos);
			    for(unsigned int j =0; j< ends2.size(); ++j){
			      
// 				Vec2i P2( (* m_NodeBuffer)[ends2[j]].f_x_pos, (* m_NodeBuffer)[ends2[j]].f_y_pos);
				
// 				
				float dist_tmp = f_distance_nodes2( ends1[i], ends2[j], m_NodeBuffer );
				if( dist_tmp < dist /*&&  projected_dist < MaxProjectedDistance*/){
				  close_i = i;
				  close_j = j;
				  dist =dist_tmp;
				}
	
			    }    
		      }
		       
		      if(dist > MaxLineSegDistance ){continue;}

		      
		      int nodeIda = ends1[close_i];
		      int nodeIdb = ends2[close_j];
		      Vec2i Pa( (* m_NodeBuffer)[nodeIda].f_x_pos, (* m_NodeBuffer)[nodeIda].f_y_pos);
		      Vec2i Pb( (* m_NodeBuffer)[nodeIdb].f_x_pos, (* m_NodeBuffer)[nodeIdb].f_y_pos);
		     
		      Line ab(0, Pa,Pb);
		      
		      Line a;
		      if(close_i==0){ a = (*it_1).Lines.front();
		 
// 			  if(angle_nodes((*it_1).reachableNodeIds[1], nodeIda,nodeIdb)<Min_angle){continue;}
		      }
		      else{a = (*it_1).Lines.back();
// 			  if(angle_nodes((*it_1).reachableNodeIds[(*it_1).reachableNodeIds.size()-2], nodeIda,nodeIdb)<Min_angle){continue;}
		      }

		      
		      if(m_Math::GetProjectiveDistance(Pb, a) > MaxProjectedDistance){continue;}  
		      if(m_Math::AngDif(a.ang, ab.ang) > MaxAngleDiff ){ continue;}
		      if(m_Math::isProjectedInsideLineSeg(Pb, a)){continue;}
		      
		      
		      
		      
		      if((*it_2).reachableNodeIds.size()>=2 ){
			   Line b;
			   if(close_j==0){ b = (*it_2).Lines.front();
// 			       if(angle_nodes((*it_2).reachableNodeIds[1], nodeIdb,nodeIda)<Min_angle){continue;}
			   }
		           else{b = (*it_2).Lines.back(); 
// 			       if(angle_nodes((*it_2).reachableNodeIds[(*it_2).reachableNodeIds.size()-2], nodeIdb,nodeIda)<Min_angle){continue;}
			  }
		           if(m_Math::GetProjectiveDistance(Pa, b) > MaxProjectedDistance){continue;}  
			   if(m_Math::AngDif(b.ang, ab.ang) > MaxAngleDiff ){ continue;}
			   if(m_Math::AngDif(a.ang, b.ang) > MaxAngleDiff ){ continue;}
			   if(m_Math::isProjectedInsideLineSeg(Pa, b)){continue;} 
		      }
		      
		      


		      if(close_i==0){ 
		      std::reverse((*it_1).reachableNodeIds.begin(), (*it_1).reachableNodeIds.end());
		      }
		      if(close_j==1){
			std::reverse((*it_2).reachableNodeIds.begin(), (*it_2).reachableNodeIds.end());
  // 				      std::reverse((*it_2).Lines.begin(), (*it_2).Lines.end());
			
		      }

		      (*it_1).Lines.push_back(Line(0 ,Pa, Pb, nodeIda, nodeIdb));
		      (*it_1).sumOfLength += (*it_1).Lines.back().len;
		      
		      (*it_1).reachableNodeIds.insert((*it_1).reachableNodeIds.end(), (*it_2).reachableNodeIds.begin(), (*it_2).reachableNodeIds.end());
		      (*it_1).Lines.insert((*it_1).Lines.end(), (*it_2).Lines.begin(), (*it_2).Lines.end());
		      (*it_1).sumOfLength += (*it_2).sumOfLength;
		      (*it_2).reachableNodeIds.clear();
		      (*it_2).Lines.clear();
		      (*it_2).sumOfLength = 0;
					      
		      connect=1; changeTimes++;

		      
		}
	 
	}
	if(changeTimes==0){NoUpdate=1;}
   }
   
   
  
}
  
  
void FindNodes::connectSingleNodes(){
  
   int MaxAngleDiff = params.graphNode._MaxAngleDiff->get();
   float MaxLineSegDistance = params.graphNode._MaxLineSegDistance->get();
   float MaxProjectedDistance = params.graphNode._MaxProjectedDistance->get();
//    int Min_angle = params.graphNode._MinAngleForSameComp->get();
//    int SmallAngle = params.graphNode.SmallAngle->get();
   
   bool NoUpdate = false;
   
   int iter_count=0;
   while (!NoUpdate && iter_count<100){
       iter_count++;
     
	LinearGraph_Buffer::iterator it_1, it_2;
	int changeTimes= 0;
	int connect=0;
	
	
	for ( it_1 =  m_LinearGraph_Buffer->begin( ); it_1 != m_LinearGraph_Buffer->end( ); it_1++ ) { // 
	        if((*it_1).reachableNodeIds.size() <=1){continue;} 
	        
	        vector<int> ends1;
	        ends1.push_back((*it_1).reachableNodeIds.front());
		ends1.push_back((*it_1).reachableNodeIds.back());
		
		for ( it_2 = it_1+1; it_2 != m_LinearGraph_Buffer->end( ); it_2++ ) { // 
		  
		      if((*it_2).reachableNodeIds.size() != 1 ){continue;} 

			
		     connect = 0;
                     float dist;
		     int nodeIda;
		     int nodeIdb =(*it_2).reachableNodeIds.front();
		     Line a;
		     float dist1 = f_distance_nodes2( ends1[0], nodeIdb, m_NodeBuffer );
		     float dist2 = f_distance_nodes2( ends1[1], nodeIdb, m_NodeBuffer );
		     if(dist1 > dist2){nodeIda = ends1[1]; dist = dist2; a = (*it_1).Lines.back();}
		     else {nodeIda = ends1[0]; dist = dist1; a = (*it_1).Lines.front();}
		      
		      if(dist > MaxLineSegDistance ){continue;}

		      
		     
		      Vec2i Pa( (* m_NodeBuffer)[nodeIda].f_x_pos, (* m_NodeBuffer)[nodeIda].f_y_pos);
		      Vec2i Pb( (* m_NodeBuffer)[nodeIdb].f_x_pos, (* m_NodeBuffer)[nodeIdb].f_y_pos);
		     
		      Line ab(0, Pa,Pb);

		      if(m_Math::GetProjectiveDistance(Pb, a) > MaxProjectedDistance){continue;}  
		      if(m_Math::AngDif(a.ang, ab.ang) > MaxAngleDiff ){ continue;}
		      if(m_Math::isProjectedInsideLineSeg(Pb, a)){continue;}
		      
		      
                    

		      if(nodeIda == ends1[0]){ 
                        (*it_1).reachableNodeIds.insert ( (*it_1).reachableNodeIds.begin() , nodeIdb );
			(*it_1).Lines.insert ( (*it_1).Lines.begin() ,   Line(0 ,Pb, Pa, nodeIdb, nodeIda) );
			(*it_1).sumOfLength += (*it_1).Lines.front().len;
		      }
		      else{
			 (*it_1).reachableNodeIds.push_back( nodeIdb );
			 (*it_1).Lines.push_back(Line(0 ,Pa, Pb, nodeIda, nodeIdb) );
			 (*it_1).sumOfLength += (*it_1).Lines.back().len;
		      }


		      (*it_2).reachableNodeIds.clear();
		      (*it_2).Lines.clear();
		      (*it_2).sumOfLength = 0;
					      
		      connect=1; changeTimes++;
		      

		}
	 
	}
// 	cout<<"iterCound "<< iter_count <<"changeTimes  "<<changeTimes<<endl;
	if(changeTimes==0){NoUpdate=1;}
   }
   
   
  
}
     

void FindNodes::updateUndistoredLines(){//vector<Line>  UndistortedLines;
  
  
  LinearGraph_Buffer::iterator it_;
  for ( it_ = m_LinearGraph_Buffer->begin( ); it_ != m_LinearGraph_Buffer->end( ); it_++ ) { // 
        vector<Line> &ls = it_->Lines;
	vector<Line> &undistortedLs = it_->UndistortedLines;
	undistortedLs.clear();
        it_->sumOfLengthUndistorted= 0;
	for(unsigned int lidx=0; lidx<ls.size();++lidx){
	  
	  Vec2i us( (*m_NodeBuffer)[ls[lidx].nodeId_s].undistorted_x_pos, (*m_NodeBuffer)[ls[lidx].nodeId_s].undistorted_y_pos );
	  Vec2i ue( (*m_NodeBuffer)[ls[lidx].nodeId_e].undistorted_x_pos, (*m_NodeBuffer)[ls[lidx].nodeId_e].undistorted_y_pos );
	  undistortedLs.push_back(Line(ls[lidx].id ,us,ue, ls[lidx].nodeId_s, ls[lidx].nodeId_e) );
	  it_->sumOfLengthUndistorted += undistortedLs.back().len;
	}
    
    
  }
}
  
  
void FindNodes::updateNodeAngle(){//vector<float>  NodeAngles;
  
   LinearGraph_Buffer::iterator it_;
  for ( it_ = m_LinearGraph_Buffer->begin( ); it_ != m_LinearGraph_Buffer->end( ); it_++ ) { // 
  
  	     vector<int> &nodes = it_->reachableNodeIds;
	     int num_nodes = it_->reachableNodeIds.size();
	     if(num_nodes<2){continue;}

	      vector<float>  & NodeAngles = it_->UndistortedNodeAngles;// 0.5pi~ 0.5pi
	     
	      vector<Line> &undistortedLs = it_->UndistortedLines;
	      
	      NodeAngles.clear();

	      
	       NodeAngles.push_back( undistortedLs.front().ang); 
	      
	      for(unsigned int nidx=1; nidx < nodes.size()-1;++nidx){
		  float ang;
		  float ang1, ang2;
		  Vec2i p1, p0, p2;
		  
		  p0 = Vec2i((*m_NodeBuffer)[nodes[nidx]].undistorted_x_pos,(*m_NodeBuffer)[nodes[nidx]].undistorted_y_pos);
		  

// 		  if(nidx ==0){ 
// 		      p2 = Vec2i((*m_NodeBuffer)[nodes[1]].undistorted_x_pos,(*m_NodeBuffer)[nodes[1]].undistorted_y_pos);
// 		      ang = m_Math::getAngle( p0, p2 );
//                  
// 		      
// 		}
// 		  else if(nidx == num_nodes -1){ 
// 		      p1 = Vec2i((*m_NodeBuffer)[nodes[ num_nodes -2]].undistorted_x_pos,(*m_NodeBuffer)[nodes[ num_nodes -2]].undistorted_y_pos);
// 		      ang = m_Math::getAngle( p0, p1 );
// // 		      ang = m_Math::CorrectAngleRadian180(ang  + 0.5*M_PI);
// 		  
// 		}
// 		  else{
		      p1 = Vec2i((*m_NodeBuffer)[nodes[nidx -1]].undistorted_x_pos,(*m_NodeBuffer)[nodes[nidx -1]].undistorted_y_pos);
		      p2 = Vec2i((*m_NodeBuffer)[nodes[nidx +1]].undistorted_x_pos,(*m_NodeBuffer)[nodes[nidx +1]].undistorted_y_pos);
		      ang1 = m_Math::getAngle2PI( p0, p1 );
		      ang2 = m_Math::getAngle2PI( p0, p2 );
		      float avg_ang =  m_Math::AngleAvg2PI (ang1,ang2);
		      
		      ang = avg_ang + 0.5*M_PI;
	    // 		       if(ang> 2*M_PI  ){ ang-= 2*M_PI;}
		      
		      ang = m_Math::CorrectAngleRadian180(ang);
		      
//                       ang = m_Math::getAngle( p0, p1 );
// 		  }
		  
		  

		  NodeAngles.push_back( ang); 
	      }
	      
	      
              NodeAngles.push_back( undistortedLs.back().ang); 


  }
  

}
  
void FindNodes::updateNodeTangentLine(){//vector<Line>  TangentLines;
  
  
   LinearGraph_Buffer::iterator it_;
  for ( it_ = m_LinearGraph_Buffer->begin( ); it_ != m_LinearGraph_Buffer->end( ); it_++ ) { // 
        vector<int> &nodes = it_->reachableNodeIds;
	int num_nodes = nodes.size();
	vector<float>  & NodeAngles = it_->UndistortedNodeAngles;
	vector<Line> & TangentLines = it_->TangentLines;
	
	float &AngleAvg = it_->undistortedAngleAvg;  
	 
	float &AngleChangeAvg = it_->undistortedAngleChangeAvg;
	TangentLines.clear();
	AngleAvg=0;
	AngleChangeAvg=0;
	if(num_nodes>=2){
	  
	      for(unsigned int nidx=0; nidx< (it_->reachableNodeIds.size()); ++nidx){
	      
		  int nodeId = nodes[nidx];
		  int x = (*m_NodeBuffer)[nodeId].undistorted_x_pos;
		  int y = (*m_NodeBuffer)[nodeId].undistorted_y_pos;
		  float ang = NodeAngles[ nidx ];		    
	      
		  cv::Point p1(x + cos(ang)*10 ,  y +  (sin(ang)*10 ));
		  cv::Point p2(x - cos(ang)*10 ,  y -  (sin(ang)*10 ));
	  
		  Line line( 0, p1,  p2 );
		  TangentLines.push_back( line );
		  
// 		  AngleAvg+= ang;
		  
		  if(nidx +1 < it_->reachableNodeIds.size()){
		     AngleChangeAvg += m_Math::AngDif(ang, NodeAngles[ nidx +1]  );
		  }
		  
		  
		  //change ang ,  in order to calculate average angle
		  if( fabs(ang - NodeAngles[ 0] ) > 0.5*M_PI){
		      if(ang > NodeAngles[ 0] ){ang -= M_PI;}
		      else{ang += M_PI;}
		  }
		  
		  AngleAvg+=ang;
		  
		  
	      }
	      AngleAvg /= float(num_nodes);
	      AngleAvg = m_Math::CorrectAngleRadian180(AngleAvg);
	      AngleChangeAvg/= float(num_nodes-1 );

	   
	   
	}
	
    
    
  }
  
  
  
  
  
  
  
}
  

  void FindNodes::updateNodeWeights(){//vector<Line>  TangentLines;
  LinearGraph_Buffer::iterator it_;
  for ( it_ = m_LinearGraph_Buffer->begin( ); it_ != m_LinearGraph_Buffer->end( ); it_++ ) { // 
        vector<int> &nodes = it_->reachableNodeIds;
	int num_nodes = nodes.size();
	vector<Line>  & lines = it_->Lines;
	vector<float> & NodeWeights = it_->NodeWeights;
	
	NodeWeights.clear();
	
	if(num_nodes==1){
	  NodeWeights.push_back(0.0);;
	}
	else if(num_nodes>=2){
	  
	        
	      for(unsigned int nidx=0; nidx < nodes.size();++nidx){
		    
		    if(nidx ==0){ 
		      NodeWeights.push_back( lines.front().len); 
		      
		      
		    }
		    else if(nidx == num_nodes -1){
		      
		       NodeWeights.push_back( lines.back().len); 
		      
		    }
		    else{
		      float len1= lines[nidx-1].len;
		      float len2= lines[nidx].len;

        	      NodeWeights.push_back((len1 + len2)/2.0); 
		      
		    }
// 		    if(NodeWeights.back()==0){
// 		      cout<<" num_nodes "<< num_nodes<<" lines.size "<<lines.size() <<"oh-----------------------"<<endl;
// 		      cout<<lines[0].s[0]<<", "<<lines[0].s[1]<<", "<<lines[0].e[0]<<", "<<lines[0].e[1]<<endl;
// 		      cout<<nodes.front()<<"  "<<nodes.back()<<endl;
// 		    }
		
	      }
	}
    
  }
  
}
  
  
void FindNodes::MergeMoreComps(){  
   
      
      int _AvgAngleDiff = params.graphNode._AvgAngleDiff->get();
      float _MaxAvgProjectedDistance = params.graphNode._MaxAvgProjectedDistance->get();
      float _MaxDistanceForMerge = params.graphNode._MaxDistanceForMerge->get();
      
      
      bool NoUpdate = false;
      
      int iter_count=0;
      while (!NoUpdate && iter_count<10000){
	    iter_count++;
	
	    LinearGraph_Buffer::iterator it_1, it_2;
	    int changeTimes= 0;
	    
	    
	    for ( it_1 =  m_LinearGraph_Buffer->begin( ); it_1 != m_LinearGraph_Buffer->end( ); it_1++ ) { // 
		    if((*it_1).reachableNodeIds.size() <=1){continue;} 
		    
    /*	   
		    vector<int> ends1;
		    ends1.push_back((*it_1).reachableNodeIds.front());
		    ends1.push_back((*it_1).reachableNodeIds.back())*/;
		    
		    for ( it_2 = it_1 +1; it_2 != m_LinearGraph_Buffer->end( ); it_2++ ) { // 
		      
			  if((*it_2).reachableNodeIds.size() <=1){continue;} 

			  if (m_Math::AngDif( (*it_1).undistortedAngleAvg, (*it_2).undistortedAngleAvg) >_AvgAngleDiff  ){continue;}

			  
			  float _MaxDistance_tmp=_MaxDistanceForMerge;
			  
			  if( it_1->reachableNodeIds.size()<=2|| it_2->reachableNodeIds.size()<=2 ){_MaxDistance_tmp *=0.4;}
			  else if( it_1->reachableNodeIds.size()<= 3 && it_2->reachableNodeIds.size()<=3 ){_MaxDistance_tmp *=0.7;}
			  else if( max(it_1->reachableNodeIds.size(), it_2->reachableNodeIds.size()) > 8
			        && min(it_1->reachableNodeIds.size(), it_2->reachableNodeIds.size())> 4 ){_MaxDistance_tmp += _MaxDistance_tmp;}
			  
			  
			  
			  
			  
			  float projectedDistAvg = 0;
			  float closestDist = std::numeric_limits<float>::max();
			  Vec2i closestP1,closestP2;
			  Vec2i closestP1_undistorted,closestp2_undistorted;
			  
			  int count=0;
			  int nodeId1, nodeId2;
		
			  for(unsigned int i=0; i< it_1->reachableNodeIds.size();++i){
			    
			    int id1 = it_1->reachableNodeIds[i];
			    Vec2i P1((*m_NodeBuffer)[id1].undistorted_x_pos, (*m_NodeBuffer)[id1].undistorted_y_pos);
			    
				for(unsigned int j=0; j< it_2->TangentLines.size();++j){
				    
				    Line l2 = it_2->TangentLines[j];
				    
				    projectedDistAvg += m_Math::GetProjectiveDistance(P1, l2);
				    int id2 = it_2->reachableNodeIds[j];
				    
				    Vec2i midP((*m_NodeBuffer)[id2].undistorted_x_pos, (*m_NodeBuffer)[id2].undistorted_y_pos);
				    
				    float dist_tmp = m_Math::TwoPointDistance(P1,midP);
				    if( dist_tmp< closestDist ){
				        closestDist = dist_tmp ;
					nodeId1 =id1; 
                                        nodeId2 =id2;
				    }
				    if(m_Math::isProjectedInsideLineSeg(P1, l2)){count++;}
				  
				}
			  
			  }
			  
			  projectedDistAvg = projectedDistAvg/float( it_1->reachableNodeIds.size() * it_2->TangentLines.size() );
			  if( count == (  it_1->reachableNodeIds.size())){ continue;}
			  if( it_1->reachableNodeIds.size() > 5 && count>= ( 0.6 * it_1->reachableNodeIds.size())){ continue;}
			  
			  if(projectedDistAvg >_MaxAvgProjectedDistance || closestDist >_MaxDistance_tmp ){ continue;}
			 
			  
			  
			 
			  projectedDistAvg=0; count =0;
			  closestDist = std::numeric_limits<float>::max();
			  
					  
			  for(unsigned int i=0; i< it_2->reachableNodeIds.size();++i){
			    
			    int nodeId = it_2->reachableNodeIds[i];
			    Vec2d P2((*m_NodeBuffer)[nodeId].undistorted_x_pos, (*m_NodeBuffer)[nodeId].undistorted_y_pos);
			    
				for(unsigned int j=0; j< it_1->TangentLines.size();++j){
// 				  
				    Line l1 = it_1->TangentLines[j];
				    projectedDistAvg += m_Math::GetProjectiveDistance(P2, l1);
				  
  // 				  Vec2i midP = l1.getMidPoint();
				    int  nidx= it_1->reachableNodeIds[j];
				    Vec2i midP((*m_NodeBuffer)[nidx].undistorted_x_pos, (*m_NodeBuffer)[nidx].undistorted_y_pos);
				    float dist_tmp = m_Math::TwoPointDistance(P2,midP);
				    if( dist_tmp< closestDist ){closestDist = dist_tmp ;}
				    if(m_Math::isProjectedInsideLineSeg(P2, l1)){count++;}
				  }
			  
			  }
			  
			  projectedDistAvg = projectedDistAvg/float(  it_2->reachableNodeIds.size() * it_1->TangentLines.size() );
			  if( count == (  it_2->reachableNodeIds.size())){ continue;}
			   if( it_2->reachableNodeIds.size() > 5 && count>= ( 0.6 * it_2->reachableNodeIds.size())){ continue;}
			  if(projectedDistAvg >_MaxAvgProjectedDistance || closestDist >_MaxDistance_tmp ){ continue;}

			    
			    
			    
			    			  //avg angle
			  if(fabs((*it_1).undistortedAngleAvg -(*it_2).undistortedAngleAvg) > 0.5*M_PI){
			      if( (*it_1).undistortedAngleAvg <(*it_2).undistortedAngleAvg) {  
				  (*it_1).undistortedAngleAvg +=M_PI;}
			      else{ (*it_2).undistortedAngleAvg +=M_PI;  }
			  }
			  int num_node_1 =  (*it_1).reachableNodeIds.size();
			  int num_node_2 =  (*it_2).reachableNodeIds.size();
			  
			  
			  (*it_1).undistortedAngleAvg =  
				      ((*it_1).undistortedAngleAvg *num_node_1 +  (*it_2).undistortedAngleAvg * num_node_2)/ float(num_node_1 +num_node_2);
	           
			  (*it_1).undistortedAngleAvg = m_Math::CorrectAngleRadian180( (*it_1).undistortedAngleAvg);
			  

/*
			  if((*it_2).reachableNodeIds.size() >  (*it_1).reachableNodeIds.size()){
			     (*it_1).undistortedAngleAvg  = (*it_1).undistortedAngleAvg ;
			  }*/

			  
			  (*it_1).reachableNodeIds.insert((*it_1).reachableNodeIds.end(), (*it_2).reachableNodeIds.begin(), (*it_2).reachableNodeIds.end());
			  (*it_1).Lines.insert((*it_1).Lines.end(), (*it_2).Lines.begin(), (*it_2).Lines.end());
			  (*it_1).UndistortedLines.insert((*it_1).UndistortedLines.end(), (*it_2).UndistortedLines.begin(), (*it_2).UndistortedLines.end());
			  (*it_1).UndistortedNodeAngles.insert((*it_1).UndistortedNodeAngles.end(), (*it_2).UndistortedNodeAngles.begin(), (*it_2).UndistortedNodeAngles.end());
			  (*it_1).TangentLines.insert((*it_1).TangentLines.end(), (*it_2).TangentLines.begin(), (*it_2).TangentLines.end());
			  (*it_1).NodeWeights.insert((*it_1).NodeWeights.end(), (*it_2).NodeWeights.begin(), (*it_2).NodeWeights.end());
			  (*it_1).sumOfLength += (*it_2).sumOfLength;
			  (*it_1).sumOfLengthUndistorted += (*it_2).sumOfLengthUndistorted;

			  closestP1 =  Vec2i ((*m_NodeBuffer)[nodeId1].x_pos, (*m_NodeBuffer)[nodeId1].y_pos);
			  closestP2 =  Vec2i ((*m_NodeBuffer)[nodeId2].x_pos, (*m_NodeBuffer)[nodeId2].y_pos);
			  
			  (*it_1).Lines.push_back(Line(0 ,closestP1, closestP2 ));
			  (*it_1).sumOfLength +=  (*it_1).Lines.back().len;
			  
			  
			  closestP1_undistorted =  Vec2i ((*m_NodeBuffer)[nodeId1].undistorted_x_pos, (*m_NodeBuffer)[nodeId1].undistorted_y_pos);
			  closestp2_undistorted = Vec2i ((*m_NodeBuffer)[nodeId2].undistorted_x_pos, (*m_NodeBuffer)[nodeId2].undistorted_y_pos);
			  (*it_1).UndistortedLines.push_back(Line(0 ,closestP1_undistorted, closestp2_undistorted ));
			  (*it_1).sumOfLengthUndistorted += (*it_1).UndistortedLines.back().len;
			  
					  
			  
			  (*it_2).reachableNodeIds.clear();
			  (*it_2).Lines.clear();
			  (*it_2).UndistortedLines.clear();
			  (*it_2).UndistortedNodeAngles.clear();
			  (*it_2).TangentLines.clear();
			  (*it_2).sumOfLength = 0;
			  (*it_2).sumOfLengthUndistorted = 0;
			  (*it_2).undistortedAngleAvg = 0;
// 			  
			
			  changeTimes++;
			      
		    }
	    }
	    if(changeTimes==0){NoUpdate=1;}
      }
      
  
}


void FindNodes::removeSmallComp (){
     
  
    float MinCmpLineLength = params.graphNode._MinCmpLineLength->get(); 
    std::sort(m_LinearGraph_Buffer->begin(), m_LinearGraph_Buffer->end(),  
    boost::bind(&LinearGraphComponent::sumOfLength, _1) > boost::bind(&LinearGraphComponent::sumOfLength, _2));
       
	LinearGraph_Buffer::iterator it;
	for ( it = m_LinearGraph_Buffer->begin( ); it != m_LinearGraph_Buffer->end( ); it++ ) {
	    if( (*it).sumOfLength < MinCmpLineLength || (*it).reachableNodeIds.size()<=1 ){break;}

	}
	m_LinearGraph_Buffer->erase( it, m_LinearGraph_Buffer->end( ));
      
	
}

 void FindNodes::updateCompPoints(){//vector<float>  NodeAngles;
  
     LinearGraph_Buffer::iterator it_;
    for ( it_ = m_LinearGraph_Buffer->begin( ); it_ != m_LinearGraph_Buffer->end( ); it_++ ) { // 
  	     vector<int> &nodes = it_->reachableNodeIds;
	     it_->Points.clear();
	     for(int i= 0; i < nodes.size(); ++i){
	       
	         Vec2i p((*m_NodeBuffer)[nodes[i]].undistorted_x_pos,(*m_NodeBuffer)[nodes[i]].undistorted_y_pos);
	         it_->Points.push_back(p);
	    }
    }
  

}    























// *************************************************************************  //
// ************************  Private Utility Methods ***********************  //
// *************************************************************************  //
      
        

// ****************************************************************************** //
void FindNodes::init( /* OUT Trans_Buffer * m_TransBuffer */ )
// ****************************************************************************** //
{
    // * Initialize acos lookup table. * //
    int i;
    float f;
    //[0,180]
    for ( i = 0; i < MAX_COS; i++ ) {
	f = ( float ) ( i * 1.0 / ( float ) MAX_COS );
	m_TransBuffer->arc_cos_sqare[ i ] = ( int ) ( ( 180.0 / 3.14159265 ) * acos( sqrt( f ) ) );
    }

    
    
    
    return;

}; // END of init_m_TransBufferfer METHOD
        
        
    // ****************************************************************************** //
  bool FindNodes::is_connected ( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j )
  // ****************************************************************************** //
  {
      /*
      * Cancel this safty check for higher efficiency.

	  if ( m_NodeBuffer->empty( ) ) {
	  return false;
      }
      */

      for ( int k = 0; k < ( * m_NodeBuffer )[ i ].num_connected; k++ ) {
	  if ( ( * m_NodeBuffer )[ i ].connected[ k ] == j ) {
	      return true;
	  }
      }
      return false;

  }; // END of is_connected METHOD


  // ****************************************************************************** //
bool FindNodes::connect_nodes ( /* IN_OUT Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j )
  // ****************************************************************************** //
  {
      /*
      * Cancel this safty check for higher efficiency.

      if ( m_NodeBuffer->empty( ) ) {
	  return false;
      }
      */

      if ( i == j ) {
	  return false;
      }

      if ( ( ( * m_NodeBuffer )[ i ].num_connected == MAX_CONN ) || ( ( * m_NodeBuffer )[ j ].num_connected == MAX_CONN ) ) {
	  return false;
      }
      ( * m_NodeBuffer )[ i ].connected[ ( * m_NodeBuffer )[ i ].num_connected++ ] = j;
      ( * m_NodeBuffer )[ j ].connected[ ( * m_NodeBuffer )[ j ].num_connected++ ] = i;

      return true;

  }; // END of connect_nodes METHOD
  
  
// ****************************************************************************** //
bool FindNodes::disconnect_nodes ( /* IN_OUT Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j , Node_Buffer * nodeBuffer )
// ****************************************************************************** //
{
    int ip, jp;

    if ( i == j ) {
	return false;
    }

    /* Detect Positions (in the connectio-array) of the edges to delete */
    for ( ip = 0; ip < ( * nodeBuffer )[ i ].num_connected; ip++ ) {
	if ( ( * nodeBuffer )[ i ].connected[ ip ] == j ) {
	    break;
	}
    }
    if ( ip == ( * nodeBuffer )[ i ].num_connected ) {
	return false;
    }
    for ( jp = 0; jp < ( * nodeBuffer )[ j ].num_connected; jp++ ) {
	if ( ( * nodeBuffer )[ j ].connected[ jp ] == i ) {
	    break;
	}
    }
    if ( jp == ( * nodeBuffer )[ j ].num_connected ) {
	return false;
    }
    /* Overwrite edge-indicators to delete with last entry in the array, decrement its length */
    ( * nodeBuffer )[ i ].connected[ ip ] = ( * nodeBuffer )[ i ].connected[ --( ( * nodeBuffer )[ i ].num_connected ) ];
    ( * nodeBuffer )[ j ].connected[ jp ] = ( * nodeBuffer )[ j ].connected[ --( ( * nodeBuffer )[ j ].num_connected ) ];

    return true;

}; // END of disconnect_nodes METHOD
  
  

 // ****************************************************************************** //
 int FindNodes::angle_nodes ( /* IN const Trans_Buffer * m_TransBuffer, */ /* IN const Node_Buffer *  m_NodeBuffer, */ /* in */ const int i, /* in */ const int j, /* in */ const int k )
 // ****************************************************************************** //
{
    int xl;
    int yl;
    int xr;
    int yr;
    long int xlq;
    long int ylq;
    long int xrq;
    long int yrq;
    long int res_z, res_c;
    long int res_n, res;

    xl = ( ( * m_NodeBuffer )[ i ].x_pos - ( * m_NodeBuffer )[ j ].x_pos );
    yl = ( ( * m_NodeBuffer )[ i ].y_pos - ( * m_NodeBuffer )[ j ].y_pos );
    xr = ( ( * m_NodeBuffer )[ k ].x_pos - ( * m_NodeBuffer )[ j ].x_pos );
    yr = ( ( * m_NodeBuffer )[ k ].y_pos - ( * m_NodeBuffer )[ j ].y_pos );

    res_z = xl * xr + yl * yr;
    xlq = xl * xl;
    ylq = yl * yl;
    xrq = xr * xr;
    yrq = yr * yr;
    res_n = ( ( xlq + ylq ) * ( xrq + yrq ) );

    if ( res_n > 0 ) {
	res_c = ( (MAX_COS-1) * ( res_z * res_z ) ) / res_n;
    }
    else {
	res_c = (MAX_COS-1);
    }
    if ( res_z >= 0 ) {
	res = ( m_TransBuffer->arc_cos_sqare[ res_c ] );
    }
    else {
	res = ( 180 - m_TransBuffer->arc_cos_sqare[ res_c ] );
    }
    return ( int ) res;

}; // END of angle_nodes METHOD


// ****************************************************************************** //
float FindNodes::f_angle_nodes ( /* IN const Trans_Buffer * m_TransBuffer, */ /* IN const Node_Buffer *  m_NodeBuffer, */ /* in */ const int i, /* in */ const int j, /* in */ const int k )
// ****************************************************************************** //
{
    float xl;
    float yl;
    float xr;
    float yr;
    double xlq;
    double ylq;
    double xrq;
    double yrq;
    double res_z, res_c;
    double res_n, res;

    xl = ( ( * m_NodeBuffer )[ i ].f_x_pos - ( * m_NodeBuffer )[ j ].f_x_pos );
    yl = ( ( * m_NodeBuffer )[ i ].f_y_pos - ( * m_NodeBuffer )[ j ].f_y_pos );
    xr = ( ( * m_NodeBuffer )[ k ].f_x_pos - ( * m_NodeBuffer )[ j ].f_x_pos );
    yr = ( ( * m_NodeBuffer )[ k ].f_y_pos - ( * m_NodeBuffer )[ j ].f_y_pos );

    res_z = xl * xr + yl * yr;
    xlq = xl * xl;
    ylq = yl * yl;
    xrq = xr * xr;
    yrq = yr * yr;
    res_n = ( ( xlq + ylq ) * ( xrq + yrq ) );

    if ( res_n > 0 ) {
	res_c = ( 511 * ( res_z * res_z ) ) / res_n;
    }
    else {
	res_c = 511;
    }
    if ( res_z >= 0 ) {
	res = ( m_TransBuffer->arc_cos_sqare[ ( int ) res_c ] );
    }
    else {
	res = ( 180 - m_TransBuffer->arc_cos_sqare[ ( int ) res_c ] );

    }
    return ( float ) res;

}; // END of f_angle_nodes METHOD



// ****************************************************************************** //
int FindNodes::distance_nodes ( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j )
// ****************************************************************************** //
{
    int x_dist, y_dist;

    x_dist = ( ( * m_NodeBuffer )[ i ].x_pos - ( * m_NodeBuffer )[ j ].x_pos );
    y_dist = ( ( * m_NodeBuffer )[ i ].y_pos - ( * m_NodeBuffer )[ j ].y_pos );

    return ( x_dist * x_dist ) + ( y_dist * y_dist );

}; // END of distance_nodes METHOD

// ****************************************************************************** //
int FindNodes::distance_nodes2 ( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j )
// ****************************************************************************** //
{
    int x_dist, y_dist;

    x_dist = ( ( * m_NodeBuffer )[ i ].x_pos - ( * m_NodeBuffer )[ j ].x_pos );
    y_dist = ( ( * m_NodeBuffer )[ i ].y_pos - ( * m_NodeBuffer )[ j ].y_pos );
    
    return  sqrt(( x_dist * x_dist ) + ( y_dist * y_dist ));
//     return m_Math::getTwoRectClosestDist(Rectangles[i], Rectangles[j]);



}; // END of distance_nodes METHOD




// ****************************************************************************** //
float FindNodes::f_distance_nodes( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j ,Node_Buffer * nodeBuffer )
// ****************************************************************************** //
{
    float x_dist, y_dist;

    x_dist = ( ( * nodeBuffer )[ i ].f_x_pos - ( * nodeBuffer )[ j ].f_x_pos );
    y_dist = ( ( * nodeBuffer )[ i ].f_y_pos - ( * nodeBuffer )[ j ].f_y_pos );

    return ( x_dist * x_dist ) + ( y_dist * y_dist );

}; // END of f_distance_nodes METHOD


// ****************************************************************************** //
float FindNodes::f_distance_nodes2( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j ,Node_Buffer * nodeBuffer )
// ****************************************************************************** //
{
      float x_dist, y_dist;

      x_dist = ( ( * nodeBuffer )[ i ].f_x_pos - ( * nodeBuffer )[ j ].f_x_pos );
      y_dist = ( ( * nodeBuffer )[ i ].f_y_pos - ( * nodeBuffer )[ j ].f_y_pos );

      return sqrt( x_dist * x_dist ) + ( y_dist * y_dist );

}; // END of f_distance_nodes METHOD



// ****************************************************************************** //
int FindNodes::closes_loop ( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j )
// ****************************************************************************** //
{
    int k, l;
    int num_i, num_j;

    num_i = ( * m_NodeBuffer )[ i ].num_connected;
    num_j = ( * m_NodeBuffer )[ j ].num_connected;

    for ( k = 0; k < num_i; k++ ) {
	for ( l = 0; l < num_j; l++ ) {
	    if ( ( * m_NodeBuffer )[ i ].connected[ k ] == ( * m_NodeBuffer )[ j ].connected[ l ] ) {
		return 1;
	    }
	}
    }
    return 0;

}; // END of closes_loop METHOD
// ****************************************************************************** //
int FindNodes::parallelogram_closes_loop ( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j )
// ****************************************************************************** //
{
    int k, l;
    int num_i, num_j;

    num_i = ( * m_NodeBuffer )[ i ].num_connected;
    num_j = ( * m_NodeBuffer )[ j ].num_connected;

    for ( k = 0; k < num_i; k++ ) {
	for ( l = 0; l < num_j; l++ ) {
	    if (  is_connected(  ( * m_NodeBuffer )[ i ].connected[ k ] , ( * m_NodeBuffer )[ j ].connected[ l ] )) {
		return 1;
	    }
	}
    }
    return 0;

}; // END of closes_loop METHOD



int FindNodes::black_line_value ( /* in */ const float ( & weightedWhiteValues )[ H ][ W ], /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j )
// ****************************************************************************** //
{
    int xm;
    int ym;
    int vm, vij;
    int res;
    
    vector<Point> contour, resCountour;

    // * Compute coordinates of pixel (and its grey-value) in between node_i and node_j * //
    xm = ( ( ( * m_NodeBuffer )[ i ].x_pos + ( * m_NodeBuffer )[ j ].x_pos + 1 ) >> 1 );
    ym = ( ( ( * m_NodeBuffer )[ i ].y_pos + ( * m_NodeBuffer )[ j ].y_pos + 1 ) >> 1 );

    if(SUB_SAMPLING_PARAMETER ==4){
      
      contour.push_back(Point( (*m_NodeBuffer)[i].x_pos << 2,  (*m_NodeBuffer)[i].y_pos << 2 ));
      contour.push_back(Point( xm <<2 , ym<<2));
      contour.push_back(Point( (*m_NodeBuffer)[j].x_pos << 2,  (*m_NodeBuffer)[j].y_pos << 2));
      
      
    }
    else{
     contour.push_back(Point( (*m_NodeBuffer)[i].x_pos * SUB_SAMPLING_PARAMETER,  (*m_NodeBuffer)[i].y_pos* SUB_SAMPLING_PARAMETER ));
     contour.push_back(Point( xm* SUB_SAMPLING_PARAMETER, ym* SUB_SAMPLING_PARAMETER));
     contour.push_back(Point( (*m_NodeBuffer)[j].x_pos* SUB_SAMPLING_PARAMETER,  (*m_NodeBuffer)[j].y_pos* SUB_SAMPLING_PARAMETER));
    }
    
     distortionModel.DistortP(contour,resCountour);
     
     vm = weightedWhiteValues[ resCountour[1].y ][ resCountour[1].x];

    // * Compute sum of grey-values of node_i and node_j and pixel in between * //
    vij =  weightedWhiteValues[ resCountour[0].y ][ resCountour[0].x ] + vm + weightedWhiteValues[ resCountour[2].y  ][ resCountour[2].x ];

    if ( vij == 0 ) {
	return 0;
    }
    // * Compute res in (0, 63). The larger res is, the darker is the middle pixel,
    // * compared to the average. (=Desired case) * //
    res = ( min( 255, ( vm * 765 ) / vij ) );

    return ( res * res ) >> 2;

}; // END of back_line_value METHOD



int FindNodes::black_line_value2 ( /* in */ const float ( & weightedWhiteValues )[ H ][ W ], /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j )
// ****************************************************************************** //
{
//     int xm;
//     int ym;
//     int vm, vij;
//     int res;
//     
//     vector<Point> contour, resCountour;
// 
//     // * Compute coordinates of pixel (and its grey-value) in between node_i and node_j * //
//     xm = ( ( ( * m_NodeBuffer )[ i ].x_pos + ( * m_NodeBuffer )[ j ].x_pos + 1 ) >> 1 );
//     ym = ( ( ( * m_NodeBuffer )[ i ].y_pos + ( * m_NodeBuffer )[ j ].y_pos + 1 ) >> 1 );
// 
// 
//      
//      vm = weightedWhiteValues[  ym][xm];
// 
//     // * Compute sum of grey-values of node_i and node_j and pixel in between * //
//     vij =  weightedWhiteValues[ ( * m_NodeBuffer )[ i ].y_pos ][ ( * m_NodeBuffer )[ i ].x_pos ] + vm 
//          + weightedWhiteValues[ ( * m_NodeBuffer )[ j ].y_pos ][ ( * m_NodeBuffer )[ j ].x_pos ];
// 
//     if ( vij == 0 ) {
// 	return 0;
//     }
//     // * Compute res in (0, 63). The larger res is, the darker is the middle pixel,
//     // * compared to the average. (=Desired case) * //
//     res = ( min( 255, ( vm * 765 ) / vij ) );
// 
//     return ( res * res ) >> 2;
     
     int WhiteValue=0;  
  
     Vec2i s(( * m_NodeBuffer )[ i ].x_pos,    ( * m_NodeBuffer )[ i ].y_pos );
     Vec2i e(( * m_NodeBuffer )[ j ].x_pos,    ( * m_NodeBuffer )[ j ].y_pos );
     
     Line line( 0,  s, e);
     
     vector<cv::Point>  PointsOnLine;
     
     m_Math::SamplePointsOnLine( line, 1.0,  PointsOnLine);
     
     if(PointsOnLine.size()<=2){
	  WhiteValue += weightedWhiteValues [  s[1] ][ s[0] ];
	  WhiteValue += weightedWhiteValues [  e[1] ][ e[0] ];
	  WhiteValue /= 2.0;
	 
     }
     else{
	for(unsigned int k=0; k< PointsOnLine.size(); ++k){
	   WhiteValue += weightedWhiteValues [ PointsOnLine[k].y ][ PointsOnLine[k].x ];
	}
	WhiteValue/= float(PointsOnLine.size());
     }
      return WhiteValue; 

}; // END of back_line_value METHOD



// ****************************************************************************** //
void  FindNodes:: update_close_nodes( /* in */ const float ( & weightedWhiteValues )[ H ][ W ], /* IN const Trans_Buffer * m_TransBuffer, */ /* IN_OUT Node_Buffer * m_NodeBuffer, */ /* in */ const int i )
// ****************************************************************************** //
{
    int j, k;
    int ang;
//     int fac;
    /* See: find_close_nodes */
    if ( ( * m_NodeBuffer )[ i ].num_connected > 2 ) {
	( * m_NodeBuffer )[ i ].num_close = 0;
	return;
    }
//     fac = ( H >> 2 ) + LINE_WIDTH * 2;

    if ( ( ( * m_NodeBuffer )[ i ].num_connected == 2 ) &&
	( angle_nodes( ( * m_NodeBuffer )[ i ].connected[ 0 ], i, ( * m_NodeBuffer )[ i ].connected[ 1 ] ) > 90 ) ) {
	    ( * m_NodeBuffer )[ i ].num_close = 0;
	    return;
    }
    k = 0;
    while ( k < ( * m_NodeBuffer )[ i ].num_close ) {
	j = ( * m_NodeBuffer )[ i ].close_id[ k ];
	ang = 120;
	if ( ( ( * m_NodeBuffer )[ j ].num_connected == MAX_CONN ) ||
	    is_connected( i, j ) ||
	    closes_loop( i, j ) ||
	    ( ( ( * m_NodeBuffer )[ i ].num_connected == 1 ) && ( ( ang = angle_nodes( ( * m_NodeBuffer )[ i ].connected[ 0 ], i, j ) ) < 60 ) ) ||
	    ( ( ( * m_NodeBuffer )[ i ].num_connected == 2 ) && ( ( ang = min( angle_nodes( ( * m_NodeBuffer )[ i ].connected[ 0 ], i, j ),
									    angle_nodes( ( * m_NodeBuffer )[ i ].connected[ 1 ], i, j ) ) ) < 60 ) ) ) {
		( * m_NodeBuffer )[ i ].num_close--;
		( * m_NodeBuffer )[ i ].close_id[ k ] = ( * m_NodeBuffer )[ i ].close_id[ ( * m_NodeBuffer )[ i ].num_close ];
	}
	else {
	    ( * m_NodeBuffer )[ i ].close_adj[ k ] =/* fac **/ ( ang - 60 ) *
		black_line_value( weightedWhiteValues, i, j ) / ( distance_nodes( i, j ) + 1 );
	    k++;
	}
    }

}; // END of update_close_nodes METHOD


// ****************************************************************************** //
void  FindNodes:: update_close_nodes2( /* in */ const float ( & weightedWhiteValues )[ H ][ W ], /* IN const Trans_Buffer * m_TransBuffer, */ /* IN_OUT Node_Buffer * m_NodeBuffer, */ /* in */ const int i )
// ****************************************************************************** //
{
    int j, k;
    int ang;
//     int fac;
    /* See: find_close_nodes */
    if ( ( * m_NodeBuffer )[ i ].num_connected > 2 ) {
	( * m_NodeBuffer )[ i ].num_close = 0;
	return;
    }
//     fac = ( H >> 2 ) + LINE_WIDTH * 2;

    if ( ( ( * m_NodeBuffer )[ i ].num_connected == 2 ) &&
	( angle_nodes( ( * m_NodeBuffer )[ i ].connected[ 0 ], i, ( * m_NodeBuffer )[ i ].connected[ 1 ] ) > 90 ) ) {
	    ( * m_NodeBuffer )[ i ].num_close = 0;
	    return;
    }
    k = 0;
    while ( k < ( * m_NodeBuffer )[ i ].num_close ) {
	j = ( * m_NodeBuffer )[ i ].close_id[ k ];
	ang = 120;
	if ( ( ( * m_NodeBuffer )[ j ].num_connected == MAX_CONN ) ||
	    is_connected( i, j ) ||
	    closes_loop( i, j ) ||
	    parallelogram_closes_loop( i, j )||
	    ( ( ( * m_NodeBuffer )[ i ].num_connected == 1 ) && ( ( ang = angle_nodes( ( * m_NodeBuffer )[ i ].connected[ 0 ], i, j ) ) < params.graphNode._MinAngle->get()  ) ) ||
	    ( ( ( * m_NodeBuffer )[ i ].num_connected == 2 ) && ( ( ang = min( angle_nodes( ( * m_NodeBuffer )[ i ].connected[ 0 ], i, j ),
									    angle_nodes( ( * m_NodeBuffer )[ i ].connected[ 1 ], i, j ) ) ) < params.graphNode._MinAngle->get() ) ) ) {
		( * m_NodeBuffer )[ i ].num_close--;
		( * m_NodeBuffer )[ i ].close_id[ k ] = ( * m_NodeBuffer )[ i ].close_id[ ( * m_NodeBuffer )[ i ].num_close ];
		( * m_NodeBuffer )[ i ].close_adj[ k ] = ( * m_NodeBuffer )[ i ].close_adj[ ( * m_NodeBuffer )[ i ].num_close ];
	}
	else {
	 
// 	    float dist =std::max(double (m_Math::getTwoRectClosestDist(Rectangles[i], Rectangles[j])), 0.5);
	    float dist = distance_nodes2( i, j );
// 	    float dist = distance_nodes( i, j );
	    float areaj = Rectangles[j].area ;
	    float num_pointsj = Rectangles[j].num_points ;
	    
	    float areai = Rectangles[i].area ;
	    float num_pointsi = Rectangles[i].num_points ;
	    
// 	    float dist_closest =  m_Math::getTwoRectClosestDist(Rectangles[i],Rectangles[j]);
	    float dist_closest = dist - std::max(Rectangles[i].rect.width /2, Rectangles[i].rect.height/2) 
	                                       - std::max(Rectangles[j].rect.width /2, Rectangles[j].rect.height/2);
	    if(dist_closest<0){ dist_closest =0; }
	    
// 	    if((areai > 40 || num_pointsi >10) && (areaj > 40 || num_pointsj >10)){ dist /=4.0; }
// 	    else if(area > 25 || num_points >6 ){ dist /=3.0; }
	    
	   
/*
	    ( * m_NodeBuffer )[ i ].close_adj[ k ] = num_pointsj *areaj * num_pointsi *areai *( ang-  params.graphNode._MinAngle->get() ) 
	                                            *black_line_value2( weightedWhiteValues, i, j ) / ( dist +1); */
	    ( * m_NodeBuffer )[ i ].close_adj[ k ] = num_pointsj *areaj * num_pointsi *areai *( ang-  params.graphNode._MinAngle->get() ) 
	                                            *black_line_value2( weightedWhiteValues, i, j ) / ( dist_closest +1) *0.0001;
              if(  ( * m_NodeBuffer )[ i ].close_adj[ k ]<0){  ( * m_NodeBuffer )[ i ].close_adj[ k ] =  std::numeric_limits<int>::max(); }
						    
						    
	    
	    k++;
	}
    }

}; // END of update_close_nodes METHOD



// ****************************************************************************** //
bool FindNodes::intersect( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int a, /* in */ const int b, /* in */ const int c, /* in */ const int d, /* out */ float * x, /* out */ float * y )
// ****************************************************************************** //
{
    float xa, xb, xc, xd;
    float ya, yb, yc, yd;
    float dx1, dx2, dy1, dy2;
    float nenn;
    float beta;
    float alpha;

    if ( ( a == b ) || ( c == d ) || ( a == c ) || ( a == d ) || ( b == c ) || ( b == d ) ) {
	return false;
    }

    xa = ( * m_NodeBuffer )[ a ].f_x_pos;
    ya = ( * m_NodeBuffer )[ a ].f_y_pos;
    xb = ( * m_NodeBuffer )[ b ].f_x_pos;
    yb = ( * m_NodeBuffer )[ b ].f_y_pos;
    xc = ( * m_NodeBuffer )[ c ].f_x_pos;
    yc = ( * m_NodeBuffer )[ c ].f_y_pos;
    xd = ( * m_NodeBuffer )[ d ].f_x_pos;
    yd = ( * m_NodeBuffer )[ d ].f_y_pos;

    if ( ( max( xa, xb ) < min( xc, xd ) ) || ( max( xc, xd ) < min( xa, xb ) ) ||
	  ( max( ya, yb ) < min( yc, yd ) ) || ( max( yc, yd ) < min( ya, yb ) )    ) {
	    return false;
    }

    dx1 = xb - xa;
    dy1 = yb - ya;
    dx2 = xd - xc;
    dy2 = yd - yc;

    nenn = dy2 * dx1 - dx2 * dy1;

    if ( nenn == 0 ) {
	return false;
    }

    beta = ( float ) ( ya * dx1 - yc * dx1 + xc * dy1 - xa * dy1 ) / ( float ) ( nenn );

    if ( ( beta <= 0 ) || ( beta >= 1 ) ) {
	return false;
    }

    if ( dx1 != 0 ) {
	alpha = ( xc - xa + dx2 * beta ) / ( float ) ( dx1 );
    }
    else if ( dy1 != 0 ) {
	alpha = ( yc - ya + dy2 * beta ) / ( float ) ( dy1 );
    }
    else {
	alpha = 0;
    }

    if ( ( alpha <= 0 ) || ( alpha >= 1 ) ) {
	return false;
    }

    * x = xc + ( float ) ( dx2 ) * beta;
    * y = yc + ( float ) ( dy2 ) * beta;

    return true;

}; // END of intersect METHOD


void FindNodes::PrintNodeBuffer (string filename,Node_Buffer * nodeBuffer){

     string pkg_path = ros::package::getPath("visual_tracking");
     const char* path= (pkg_path + "/tmp_file/"+ filename).c_str(); 
     
     std::ofstream nodeInfo;
     nodeInfo.open (path, ofstream::out|ofstream::trunc);
     
     if ( nodeInfo.is_open()){
           int i= -1;
		    
           Node_Buffer::iterator it;
           for ( it = nodeBuffer->begin( ); it != nodeBuffer->end( ); it++ ) {
	      ++i;
	      
// 	      nodeInfo<<i<<"["<< (*it).x_pos<< ", "<< (*it).y_pos<<"] ";
	      nodeInfo<<i<<", Num of Connected: "<< (*it).num_connected<<",    connected node id:  ";
	      for(int j=0; j<(*it).num_connected; ++j ){
		  nodeInfo<< (*it).connected[j]<<",  ";
		}
		nodeInfo<<endl;
	     
	     
	     
	   }
      
	    
      }
      else
      {
	  std::cout << "Error opening file";
      }

     nodeInfo.close();
 
  
  
  
}


void FindNodes::PrintReacherableComp (string filename){

     string pkg_path = ros::package::getPath("visual_tracking");
     const char* path= (pkg_path + "/"+ filename).c_str(); 
     
     std::ofstream CompInfo;
     CompInfo.open (path, ofstream::out|ofstream::trunc);
     
     if ( CompInfo.is_open()){
           int i= -1;
		    
           LinearGraph_Buffer::iterator it;
           for ( it = m_LinearGraph_Buffer->begin( ); it != m_LinearGraph_Buffer->end( ); it++ ) {
	      ++i;
// 	      if((*it).reachableNodeIds.size()==0){continue;}
	      CompInfo<<i<<":    ";
	      CompInfo<<"ids:  ";
	      for(unsigned int j=0; j<(*it).reachableNodeIds.size(); ++j ){
		  CompInfo<< (*it).reachableNodeIds[j]<<",  ";
	      }
		CompInfo<<endl;

	   }
      
	    
      }
      else
      {
	  std::cout << "Error opening file";
      }

     CompInfo.close();
}



bool FindNodes::is_reacherable( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j ){
    LinearGraph_Buffer::iterator it;
    if(i==j) {return true;}
//       int k=-1;
      for ( it = m_LinearGraph_Buffer->begin( ); it != m_LinearGraph_Buffer->end( ); it++ ) { // [1]
//           ++k;

	   for(unsigned int m = 0; m<it->reachableNodeIds.size();m++){
	     
		if(i == it->reachableNodeIds[m]){
		      for(unsigned int n = 0; n< it->reachableNodeIds.size();n++){
			  if(j == it->reachableNodeIds[n]){
			    return true;
			  }

		      }
		      return false;
		  
		}
	     
	   }
// 	   return false;
 
      }
       return false;

}



void FindNodes::update_reachable(  const int i, /* in */ const int j ){
  
      LinearGraph_Buffer::iterator it;
      unsigned int i_c_id =  m_LinearGraph_Buffer->size( );
      unsigned int j_c_id = m_LinearGraph_Buffer->size( );
      
      int k=-1;
      for ( it = m_LinearGraph_Buffer->begin( ); it != m_LinearGraph_Buffer->end( ); it++ ) { // [1]
          ++k;
	   for(unsigned int m = 0; m<it->reachableNodeIds.size();m++){
	     
		if(i == it->reachableNodeIds[m]){
		    i_c_id =k;
		    if( j_c_id < m_LinearGraph_Buffer->size( )){break;}
		}
		if(j == it->reachableNodeIds[m]){
		    j_c_id =k;
		    if( i_c_id < m_LinearGraph_Buffer->size( )){break;}
		}
		
	     
	   }
	    if( j_c_id < m_LinearGraph_Buffer->size( )&& i_c_id < m_LinearGraph_Buffer->size( )){break;}
      }
      
      if( j_c_id  == i_c_id ){ return; }
      
      int s1 = (*m_LinearGraph_Buffer)[i_c_id].Lines.size();
      int s2 = (*m_LinearGraph_Buffer)[j_c_id].Lines.size();
      
      
      Vec2i p1( (* m_NodeBuffer)[i].f_x_pos, (* m_NodeBuffer)[i].f_y_pos );
      Vec2i p2( (* m_NodeBuffer)[j].f_x_pos, (* m_NodeBuffer)[j].f_y_pos );
      Line tmpL(s1, p1, p2, i ,j );
      
      (*m_LinearGraph_Buffer)[i_c_id].Lines.push_back (tmpL);
      (*m_LinearGraph_Buffer)[i_c_id].sumOfLength += tmpL.len;
			  
//       for( int k=0; k< s2; ++k){
// 	    Line c =  (*m_LinearGraph_Buffer)[j_c_id].Lines[k];
// 	    c.id += (s1 +1);
// 	    (*m_LinearGraph_Buffer)[i_c_id].Lines.push_back (c);
// 	    
//       }
			  
// 	for(unsigned int k=0; k< (*m_LinearGraph_Buffer)[j_c_id].reachableNodeIds.size(); ++k){
// 	      int  nodeId =  (*m_LinearGraph_Buffer)[j_c_id].reachableNodeIds[k];
// 	      (*m_LinearGraph_Buffer)[i_c_id].reachableNodeIds.push_back (nodeId);
// 	      
// 	}
	
	(*m_LinearGraph_Buffer)[i_c_id].Lines.insert((*m_LinearGraph_Buffer)[i_c_id].Lines.end(),  
						     (*m_LinearGraph_Buffer)[j_c_id].Lines.begin(),
						     (*m_LinearGraph_Buffer)[j_c_id].Lines.end() );
	 (*m_LinearGraph_Buffer)[i_c_id].reachableNodeIds.insert( (*m_LinearGraph_Buffer)[i_c_id].reachableNodeIds.end(),  
								  (*m_LinearGraph_Buffer)[j_c_id].reachableNodeIds.begin(),  
								  (*m_LinearGraph_Buffer)[j_c_id].reachableNodeIds.end());

//         copy((*m_LinearGraph_Buffer)[j_c_id].reachableNodeIds.begin(), (*m_LinearGraph_Buffer)[j_c_id].reachableNodeIds.end(), 
// 	     (*m_LinearGraph_Buffer)[i_c_id].reachableNodeIds.end());
	(*m_LinearGraph_Buffer)[i_c_id].sumOfLength += (*m_LinearGraph_Buffer)[j_c_id].sumOfLength;
	(*m_LinearGraph_Buffer)[j_c_id].reachableNodeIds.clear();
	(*m_LinearGraph_Buffer)[j_c_id].Lines.clear();
	(*m_LinearGraph_Buffer)[j_c_id].sumOfLength =0;
  
}
	  