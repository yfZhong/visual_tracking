

// #include "findNodes.h"
#include <visual_tracking/LineMatcher/findNodes.h>

// ****************************************************************************** //
void  FindNodes::findNodes ( /* in_out */ int ( & matrix )[ H ][ W ] /* OUT Node_Buffer * m_NodeBuffer, */ )
// ****************************************************************************** //
{ 
    
    memset( tmp, 0, sizeof ( int ) * H * W );

    m_NodeBuffer->clear( ); //<- Clear Node_Buffer for a new turn.

    // * For each type of skeleton-pixels (peeks, 2 x ridges) do .... * //
    for ( int i = 0; i < 3; i++ ) { // [0]

	for ( int y = H - 2; y > 0; y-- ) { // [1]
	    for ( int x = 1; x < W - 1; x++ ) { // [2]


		// * UNIT+i indicates, that skeleton-pixel belongs to type 'i'. * //
		if ( matrix[ y ][ x ] == UNIT + i ) { // [3]

		    const int num = m_NodeBuffer->size( );
/*
		    if ( num == MAX_NODES_NUM - 1 ) {
			break;
		    }
*/
		    // * Create new node at position (x,y). * //
		    Node node;
		    node.x_pos = x;
		    node.y_pos = y;
		    node.num_connected = 0;
		    node.num_close = 0;
		    node.crossing_type = Node::CT_Unknown;
		    m_NodeBuffer->push_back(node);

		    // * Examine the 3*3 Neighborhood of the new node.  * //
		    for ( int dy = -1; dy < 2; dy++ ) { // [4]
			for ( int dx = -1; dx < 2; dx++ ) { // [5]

			    const int yp = y + dy;
			    const int xp = x + dx;
			    const int val = matrix[ yp ][ xp ];

			    // * Two node-neighborhoods are overlapping. * //
			    if ( ( val > 0 ) && ( val < UNIT ) ) { // [6]
				// * The corresponding indicator (val+UNIT*2) is set. * //
				matrix[ yp ][ xp ] += UNIT * 2;
				tmp[ yp ][ xp ] = num;
			    }
			    // * A 'clean' skeleton-pixel in the neighborhood of the new node is detected and marked. * //
			    else if ( val >= UNIT && val < UNIT * 2 ) { // [6]
				matrix[ yp ][ xp ] = num + 1;
			    }
			    // * Three node-neighborhoods are overlapping, the corresponding marker is set. * //
			    else if ( val >= UNIT * 2 ) { // [6]
				matrix[ yp ][ xp ] = UNIT * 3;
			    }
			} // [5]
		    } // [4]
		} // [3]
/*
		if ( num == MAX_NODES_NUM - 1 ) { // [3]
		    break;
		}
*/
	    } // [2]
	} // [1]

    } // [0]
    // * Connect all nodes where exactly 2 neighborhoods overlap. * //
    for ( int y = H - 2; y > 0; y-- ) { // [0]
	for ( int x = 1; x < W - 1; x++ ) { // [1]

	    const int val = matrix[ y ][ x ];

	    if ( ( val > UNIT * 2 ) && ( val < UNIT * 3 ) ) {
		if ( !is_connected( ( val - ( UNIT * 2 + 1 ) ), ( tmp[ y ][ x ] ) ) ) {
		      connect_nodes( ( val - ( UNIT * 2 + 1 ) ), ( tmp[ y ][ x ] ) );
		}
	    }

	} // [1]
    } // [0]

    return;

}; // END of FindNodes METHOD


// ****************************************************************************** //
void FindNodes::ConnectTouchingNodes ( /* in */ const  int ( & matrix )[ H ][ W ] /* IN_OUT Node_Buffer * m_NodeBuffer, */ )
// ****************************************************************************** //
{

    // * Examine the whole skeleton * //
    for ( int y = H - 2; y > 0; y-- ) { // [1]
	for ( int x = 1; x < W - 1; x++ ) { // [0]

	    int value = matrix[ y ][ x ];

	    // * Is (x,y) in a non-overlapping node-neighborhood ? * //
	    if ( ( value > 0 ) && ( value < UNIT ) ) { // [2]
		int last_val = 0;
		int value_id = value - 1;

		// * Examine parts of the neighborhood * //
		for ( int dy = 0; dy < 3; dy++ ) { // [3]
		    for ( int dx = 0; dx < 2; dx++ ) { // [4]

			if ( ( dx == 0 ) && ( dy < 2 ) ) { // [5]
			    continue;
			}
			int val = matrix[ y + dy - 1 ][ x + dx ];

			// * Is neighbor-pixel in a non-overlapping node-neighborhood AND
			// * neighbor-pixel != actual pixel AND no connection has just been established ? * //
			if ( ( val > 0 ) && ( val < UNIT ) && ( val != value ) && ( val != last_val ) ) { // [5]
			    last_val = val;
			    val--;
			    // * Connect them, if they're not already connected * //
			    if ( !is_connected( val, value_id ) ) { // [6]
				  connect_nodes( val, value_id );
			    }

			} // [5]

		    } // [4]
		} // [3]

	    } // [2]

	} // [1]
    } // [0]

    return;

}; // END of ConnectTouchingNodes METHOD

void FindNodes::FindMoreNodes ( /* in */ const int ( & matrix )[ H ][ W ] /* IN_OUT Node_Buffer * m_NodeBuffer, */ )
// ****************************************************************************** //
{
// * For all nodes do ... * //
    for ( unsigned int i = 0; i < m_NodeBuffer->size( ); i++ ) { // [0]

	// * 0. Skip for nodes of degree > 1 * //
	if ( ( * m_NodeBuffer )[ i ].num_connected > 1 ) { // [1]
	    continue;
	}

	// * 1. Try to split nodes with degree 0 into two with degree 1 * //
	if ( ( * m_NodeBuffer )[ i ].num_connected == 0 ) { // [1]

	    // * Examine the 'far' neighborhood of the node * //
	    for ( int dy = -1; dy < 2; dy += 2 ) { // [2]
		for ( int dx = -1; dx < 2; dx += 2 ) { // [3]

		    const int x = ( * m_NodeBuffer )[ i ].x_pos + dx;
		    const int y = ( * m_NodeBuffer )[ i ].y_pos + dy;

		    // * Is (x,y) a skeleton-pixel ? * //
		    if ( matrix[ y ][ x ] > 0 ) { // [4]

			// * Create new node at (x,y), connect it to the examined one * //
			Node node;
			node.x_pos = x;
			node.y_pos = y;
			node.num_connected = 0;
			node.num_close = 0;

			m_NodeBuffer->push_back( node );
			connect_nodes( i, m_NodeBuffer->size( ) - 1 );

			break;
		    } // [4]

		} // [3]

		if ( ( * m_NodeBuffer )[ i ].num_connected != 0 ) { // [3]
		    break;
		}
	    } // [2]

	} // [1]

	// * 2. Try to move nodes with degree 1 to the end of the skeleton - line * //
	if ( ( * m_NodeBuffer )[ i ].num_connected == 1 ) { // [1]

	    int j = ( * m_NodeBuffer )[ i ].connected[ 0 ];

	    // * Chose (x,y) in the continued direction of node_j -> node_i * //
	    int dx = ( * m_NodeBuffer )[ i ].x_pos - ( * m_NodeBuffer )[ j ].x_pos;
	    if ( abs( dx ) > 1 ) { // [2]
		dx /= 2;
	    }
	    int dy = ( * m_NodeBuffer )[ i ].y_pos - ( * m_NodeBuffer )[ j ].y_pos;
	    if ( abs( dy ) > 1 ) { // [2]
		dy /= 2;
	    }
	    int x = ( * m_NodeBuffer )[ i ].x_pos + dx;
	    int y = ( * m_NodeBuffer )[ i ].y_pos + dy;
	    x = min( W - 1, x );    
	    x = max( 0,                x );
	    y = min( H - 1, y );
	    y = max( 0,                 y );

	    // * Is (x,y) skeleton-pixel ? Move node_i to new position * //
	    if ( matrix[ y ][ x ] > 0 ) { // [2]
		( * m_NodeBuffer )[ i ].x_pos = x;
		( * m_NodeBuffer )[ i ].y_pos = y;
	    }
	    // * If not, examine a small neighborhood of (x,y) * //
	    else { // [2]

		for ( int ddy = -1; ddy < 2; ddy++ ) { // [3]
		    for ( int ddx = -1; ddx < 2; ddx++ ) { // [4]

			if ( abs( ddx ) + abs( ddy ) != 1 ) { // [5]
			    continue;
			}
			x = ( * m_NodeBuffer )[ i ].x_pos + dx + ddx;
			y = ( * m_NodeBuffer )[ i ].y_pos + dy + ddy;
			x = min( W - 1, x );
			x = max( 0,     x );
			y = min( H - 1, y );
			y = max( 0,     y );

			// * Don't move to the same position * //
			if ( ( x == ( * m_NodeBuffer )[ i ].x_pos ) && ( y == ( * m_NodeBuffer )[ i ].y_pos ) ) { // [5]
			    continue;
			}
			// * Is (x_n, y_n) a skeleton-pixel ? Move node_i to (x_n, y_n) * //
			if ( matrix[ y ][ x ] > 0 ) { // [5]
			    ( * m_NodeBuffer )[ i ].x_pos = x;
			    ( * m_NodeBuffer )[ i ].y_pos = y;
			    break;
			}
		    } // [4]
		    if ( ddy < 2 ) { // [4]
			break;
		    }
		} // [3]

	    } // [2]

	} // [1]

    } // [0]

    return;

}; // END of FindMoreNodes METHOD


// ****************************************************************************** //
void FindNodes::FindCloseNodes ( /* in */const float ( & weightedWhiteValues )[ H ][ W ] /* IN const Trans_Buffer * m_TransBuffer, */ /* IN_OUT Node_Buffer * m_NodeBuffer, */ )
// ****************************************************************************** //
{

    int i = -1, j = -1;
    int rad;
//     int fac; //<- Indicates the size of the gaps, which may be closed.
    int ang, dist;
    
    
//     fac = ( H >> 2 ) + LINE_WIDTH * 2;
    rad = max<int>( 10, ( LINE_WIDTH ) );
    rad = rad * rad;

    // * For all nodes do .... * //
    Node_Buffer::iterator it;
    for ( it = m_NodeBuffer->begin( ); it != m_NodeBuffer->end( ); it++ ) { // [0]
	i++;
	j = -1;

	it->num_close = 0;

	// * Skip all nodes with degree >2. * //
	if ( it->num_connected > 2 ) { // [1]
	    continue;
	}
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
	    dist = distance_nodes( i, j );
	    if ( dist > rad ) { // [2]
		continue;
	    }
	    if ( is_connected( i, j ) ) { // [2]
		continue;
	    }
	    if ( it->num_close == MAX_CLOSE ) { // [2]
		break;
	    }
	    // * If a small loop would be produced, continue. * //
	    if ( closes_loop( i, j ) ) { // [2]
		continue;
	    }
	    if ( it->num_connected == 1 ) { // [2]
		// * If angle (node_k->node-i) with new edge (node_i->node-j) is too small, continue. * //
		if ( ( ang = angle_nodes( it->connected[ 0 ], i, j ) ) < SMALL_ANG ) { // [2]
		    continue;
		}
	    }
	    else if ( it->num_connected == 2 ) { // [2]
		// * If angle MIN((node_k1->node-i), (node_k2->node-i)) with new edge
		// * (node_i->node-j) is too small, continue. * //
		if ( ( ang = min( angle_nodes( it->connected[ 0 ], i, j ), angle_nodes( it->connected[ 1 ], i, j ) ) ) < SMALL_ANG ) { // [3]
			continue;
		}

	    }
	    else { // [2]
		ang = 120;
	    }

	    // * Evaluate the significance (adj) of the new edge. * //
	    it->close_id[ it->num_close ] = j;
	    it->close_adj[ it->num_close ] = /*fac **/ ( ang - SMALL_ANG ) * black_line_value( weightedWhiteValues, i, j ) / ( dist + 1 );//prefer large ang, small dist, high brightness
	    it->num_close++;

	    if ( it->num_close == MAX_CLOSE ) { // [2]
		break;
	    }

	} // [1*]

    } // [0]

    return;

}; // END of FindCloseNodes METHOD



// ****************************************************************************** //

void FindNodes::ConnectCloseNodes ( /* in */ const float ( & weightedWhiteValues )[ H ][ W ]/* IN const Trans_Buffer * m_TransBuffer, */ /* IN_OUT Node_Buffer * m_NodeBuffer, */ )
// ****************************************************************************** //
{

    int min_adj;
    int adj, max_adj;
    int i, j, k, l, max_i = 0, max_j = 0;
    int num_j, angle, max_angle, min_angle;
    int iteration = 0;
    int num_ends = 0;
    int MinAdj = params.graphNode.Min_adj->get();

    // * compute the number of open ends. * //
    Node_Buffer::iterator it;
    for ( it = m_NodeBuffer->begin( ); it != m_NodeBuffer->end( ); it++ ) {
	if ( it->num_connected < 2 ) {
	    num_ends += 2 - it->num_connected;
	}
    }
//     cout<<"num_ends s: "<<num_ends<<endl;
    
    // * find the best candidate connection and connect the nodes i and j, until termination
    // * condition is TRUE. * //
    do {
// 	min_adj = max( MinAdj, 500000 - ( MinAdj * ( int ) num_ends ) );
// 	min_adj = max( 100, 1000000 - ( MinAdj * ( int ) num_ends ) );
	min_adj = max(1, MinAdj);
	max_adj = min_adj;
        
	i = -1;
	// * for all nodes ... * //
	for ( it = m_NodeBuffer->begin( ); it != m_NodeBuffer->end( ); it++ ) {
	    i++;

	    // * for all candidates ... * //
	    for ( k = 0; k < it->num_close; k++ ) {

		j = it->close_id[ k ];
		adj = it->close_adj[ k ];
//                 cout<<"adj: "<< adj<<endl;
		num_j = ( * m_NodeBuffer )[ j ].num_connected;

		if ( num_j != 2 ) {
		    adj += adj / 2;
		}
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
		adj += ( adj * ( max_angle - 90 ) ) / 90;
		adj += ( adj * min_angle ) / 90;
		if ( closes_loop( i, j ) ) {
		    continue;
		}
		if ( adj > max_adj ) {
		    max_adj = adj, max_i = i, max_j = j;
		}
	    }	    
	    
	}
	// * If the best candidate is good enough, insert it. * //
	if ( max_adj > min_adj ) {

	    // *  update the number of open ends. * //
	    if ( ( * m_NodeBuffer )[ max_i ].num_connected < 2 ) {
		num_ends--;
	    }
	    if ( ( * m_NodeBuffer )[ max_j ].num_connected < 2 ) {
		num_ends--;
	    }
	    connect_nodes( max_i, max_j );

	    update_close_nodes( weightedWhiteValues, max_i );
	    if ( ( * m_NodeBuffer )[ max_j ].num_connected < 4 ) {
		update_close_nodes( weightedWhiteValues, max_j );
	    }
	    iteration = 0;
	}
	else {
	    if ( iteration == 0 ) {
		FindCloseNodes( weightedWhiteValues );
	    }
	    iteration++;
	}
    } while ( max_adj > min_adj || iteration < 2 );
   
  
    return;

}; // END of ConnectCloseNodes METHOD


  // ****************************************************************************** //
  void FindNodes::SmoothNodes ( /* IN_OUT Node_Buffer * m_NodeBuffer, */ )
  // ****************************************************************************** //
  {
      int i;
      int j, k;
      i=-1;
      Node_Buffer::iterator it;
      for ( it = m_NodeBuffer->begin( ); it != m_NodeBuffer->end( ); it++ ) {
          ++i;
	  
	  if ( it->num_connected != 2  || (it->num_connected==2 &&  (angle_nodes(it->connected[ 0 ], i, it->connected[ 1 ])-90)<30)) {
	      it->f_x_pos = ( float ) it->x_pos;
	      it->f_y_pos = ( float ) it->y_pos;
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
	  if ( it->num_connected != 2 || (it->num_connected==2 &&  (angle_nodes(it->connected[ 0 ], i, it->connected[ 1 ])-90)<30)) {
	      continue;
	  }
	  
	  it->f_x_pos /= 4.0;
	  it->f_y_pos /= 4.0;
      }

      return;

  }; // END of SmoothNodes METHOD


  
  
// ****************************************************************************** //
void FindNodes::DeleteNodes ( /* IN const Trans_Buffer * m_TransBuffer, */ /* IN_OUT Node_Buffer * m_NodeBuffer, */ /* in */ )
// ****************************************************************************** //
{

    int i, j, k, l, m, n;
    int max_angle;
    int counter;
    int del, ang;
    int num_j, num_k;
    float min_dist, max_dist, dist;
    float dist_j, dist_k;
    int a;
    int min_ang;
    int angle;
    
    int MergeJointDist = params.graphNode.MergeJointDist->get();
    int MinDistForMerge =  params.graphNode.MinDistForMerge->get();

    // * Parameter indicating 'large' or 'small', depending on the size of the image. * //
    max_dist = ( float ) max(9,  params.graphNode.MaxDist->get() );
//     ( float ) max( 9.0, ( H * H ) / 128.0 );
    
    
    ang = 180;

    // * try to delete nodes for decreasing values of ang * //
      do { // [0]
	counter = 0;
	Node_Buffer::iterator it_i;
	i = -1;
	for ( it_i = m_NodeBuffer->begin( ); it_i != m_NodeBuffer->end( ); it_i++ ) { // [1]
	    i++;

	    // * If deg(node_i)=2 do ... * //
	    if ( it_i->num_connected == 2 ) { // [2]

		del = 0;

		j = it_i->connected[ 0 ];
		num_j = ( * m_NodeBuffer )[ j ].num_connected;

		k = it_i->connected[ 1 ];
		num_k = ( * m_NodeBuffer )[ k ].num_connected;

		dist_j = f_distance_nodes( i, j ,m_NodeBuffer);
		dist_k = f_distance_nodes( i, k ,m_NodeBuffer);
		min_dist = min( dist_j, dist_k );

		if ( min_dist < max_dist ) { // [3]
		    min_ang = 90, angle = ang - 30;
		}
		else { // [3]
		    min_ang = 120, angle = ang;
		}
		if ( ( dist_k < max_dist * 2 ) && ( ( num_k == 1 ) || ( num_k >= 3 ) ) ) { // [3]
		    angle -= 10, min_ang -= 20;
		}
		if ( ( dist_j < max_dist * 2 ) && ( ( num_j == 1 ) || ( num_j >= 3 ) ) ) { // [3]
		    angle -= 10, min_ang -= 20;
		}
		// * If angle at node_i is large, delete node_i. * //
		if ( ( a = ( int ) f_angle_nodes( j, i, k) ) > angle ) { // [3]
		    del = 1;
		}
		// * If angle at node_i is small, don't delete node_i. * //
		else { // [3]

		    if ( a < min_ang ) { // [4]
			continue;
		    }

		    // * Look at nodes that have a distance of 2 to node_i. * //
		    if ( ( num_j > 1 ) && ( dist_j < max_dist * 2 ) ) { // [4]
			max_angle = 0;
			for ( n = 0; n < num_j; n++ ) { // [5]

			    if ( ( l = ( * m_NodeBuffer )[ j ].connected[ n ] ) == i ) { // [6]
				continue;
			    }
			    a = ( int ) f_angle_nodes( k, i, l  );
			    if ( a > max_angle ) { // [6]
				max_angle = a;
			    }
			}
			if ( max_angle > angle ) { // [5]
			    del = 1;
			}

		    } // [4]

		    if ( ( del == 0 ) && ( num_k > 1 ) && ( dist_k < max_dist * 2 ) ) { // [4]
			max_angle = 0;
			for ( n = 0; n < num_k; n++ ) { // [5]

			    if ( ( l = ( * m_NodeBuffer )[ k ].connected[ n ] ) == i ) { // [6]
				continue;
			    }
			    a = ( int ) f_angle_nodes( j, i, l );
			    if ( a > max_angle ) { // [6]
				max_angle = a;
			    }
			}
			if ( max_angle > angle ) { // [5]
			    del = 1;
			}

		    } // [4]

		} // [3]

		// * if node_i is detected for deletion,
		// * check some conditions that prevent node_i from being deleted. * //
		if ( del ) {  // [3] //<- don't delete, if the resultiong edge is too long.

		    if ( ( dist_j + dist_k > max_dist * 6 ) && ( dist_k > max_dist * 2 ) && ( dist_j > max_dist * 2 ) ) { // [4]
			continue;
		    }
		    // * don't delete, if an accute angle would be produced. * //
		    if ( num_j == 2 ) { // [4]

			if ( ( * m_NodeBuffer )[ j ].connected[ 0 ] == i ) { // [5]
			    l = ( * m_NodeBuffer )[ j ].connected[ 1 ];
			}
			else { // [5]
			    l = ( * m_NodeBuffer )[ j ].connected[ 0 ];
			}
			if ( ( f_angle_nodes( l, j, k ) <= 100 ) && ( f_angle_nodes( l, j, i ) > 100 ) ) { // [5]
			    continue;
			}

		    } // [4]

		    if ( num_k == 2 ) { // [4]

			if ( ( * m_NodeBuffer )[ k ].connected[ 0 ] == i ) { // [5]
			    l = ( * m_NodeBuffer )[ k ].connected[ 1 ];
			}
			else { // [5]
			    l = ( * m_NodeBuffer )[ k ].connected[ 0 ];
			}
			if ( ( f_angle_nodes( l, k, j ) <= 100 ) && ( f_angle_nodes( l, k, i ) > 100 ) ) { // [5]
			    continue;
			}

		    } // [4]

		    // * delete node_i by removing it's connections. * //
		    disconnect_nodes( i, j , m_NodeBuffer);
		    disconnect_nodes( i, k , m_NodeBuffer);
		    if(closes_loop(j,k)){continue;}
		    
		    if ( !is_connected( j, k ) ) { // [4]
			connect_nodes( j, k );
		    }
		    counter++;

		} // [3]

	    }  // [2]

	    // * if node_i is an end of a line. * //
	    else if ( it_i->num_connected == 1 ) {  // [2]

		j = it_i->connected[ 0 ];

		// * delete node_i, if there is a joint, or a cross near by. * //
		if ( ( * m_NodeBuffer )[ j ].num_connected < 3 ) {  // [3]
		    continue;
		}
		if ( ( ( * m_NodeBuffer )[ j ].num_connected == 3 ) && ( f_distance_nodes( i, j ,m_NodeBuffer) > max_dist ) ) {  // [3]
		    continue;
		}
		if ( ( ( * m_NodeBuffer )[ j ].num_connected == 4 ) && ( f_distance_nodes( i, j ,m_NodeBuffer) > max_dist / 2 ) ) {  // [3]
		    continue;
		}
		disconnect_nodes( i, j ,m_NodeBuffer);
		counter++;

	    }  // [2]

	    // * if node_i is a joint. * //
	    else if ( it_i->num_connected == 3 ) {  // [2]
		min_dist = FLT_MAX; //<- maximum distance.
		j = -1;
		for ( k = 0; k < 3; k++ ) {  // [3]
		    m = it_i->connected[ k ];

		    dist = f_distance_nodes( i, m ,m_NodeBuffer);

		    if ( ( ( * m_NodeBuffer )[ m ].num_connected == 3 ) && ( dist < min_dist ) ) {  // [4]
			j = m, min_dist = dist;
		    }

		}  // [3]

		// * try to detect and merge two connected joints to a cross, if they are close. * //
		if ( j > -1 ) {  // [3]

		    if ( min_dist > MergeJointDist ) {  // [4]
			continue;
		    }
		    disconnect_nodes( i, j ,m_NodeBuffer );

		    num_j = ( * m_NodeBuffer )[ j ].num_connected;

		    for ( m = 0; m < num_j; m++ ) {  // [4]
			l = ( * m_NodeBuffer )[ j ].connected[ 0 ];
			disconnect_nodes( j, l ,m_NodeBuffer );
			if ( !is_connected( i, l ) && !closes_loop(i,l)) {  // [5]
			    connect_nodes( i, l );
			}

		    } // [4]

		    it_i->f_x_pos = ( float ) ( ( it_i->f_x_pos + ( * m_NodeBuffer )[ j ].f_x_pos ) / 2.0 );
		    it_i->f_y_pos = ( float ) ( ( it_i->f_y_pos + ( * m_NodeBuffer )[ j ].f_y_pos ) / 2.0 );

		    counter++;

		}  // [3]

	    }  // [2]

	}  // [1]

	if ( ang >= 155 ) {  // [1]
	    ang -= 5;
	}

      } while ( counter > 0 || ang >= 155 );   // [0]


    Node_Buffer::iterator it;
/*
    // * Remove Nodes that are not surrounded by Green. * //
    i = -1;
    int green_count;

    for ( it = m_NodeBuffer->begin( ); it != m_NodeBuffer->end( ); it++ ) { // [0]
	i++;

	if ( 0 == it->num_connected ) { // [1]
	    continue;
	}

	green_count = 0;

	if ( it->x_pos > 2 && it->x_pos < W - 2 && it->y_pos < H - 2 ) { // [1]

	    for ( int y = -2; y < 3; y++ ) { // [2]
		for ( int x = -2; x < 3; x++ ) { // [3]

		    if ( matrix[ it->y_pos + y ][ it->x_pos + x ] > 0 ) { // [4]
			green_count++;
		    }

		} // [3]
	    } // [2]

	} // [1]

	if ( green_count < 5 ) { // [1]

	    for ( int ind = 0; ind < it->num_connected; ind++ ) { // [2]

		disconnect_nodes( m_NodeBuffer, i, it->connected[ ind ] ,m_NodeBuffer);
	    }

	} // [1]

    } // [0]
*/

    // * Merge nodes that are too close. * //
    

    
    int counter2=1; 
    
   
    while(counter2 >0){
        counter2 = 0;
	i = -1; 
	int tmpj = 0, tmpk =0;
	for ( it = m_NodeBuffer->begin( ); it != m_NodeBuffer->end( ); it++ ) { // [0]
	    i++;

	    if ( 2 != it->num_connected ) { // [1]
		continue;
	    }

	    j = it->connected[ 0 ];
	    k = it->connected[ 1 ];

	   
	    if ( f_distance_nodes( i, j ,m_NodeBuffer) <= MinDistForMerge ) { // [1]

		if ( 2 == ( * m_NodeBuffer )[ j ].num_connected ) { // [2]
		    
		    for ( int ind = 0; ind < 2; ind++ ) { // [3]
			if ( i != ( * m_NodeBuffer )[ j ].connected[ ind ] ) { // [4]
			    tmpj = ( * m_NodeBuffer )[ j ].connected[ ind ];
			}
		    }

		    if ( angle_nodes( i, j, tmpj ) > angle_nodes( k, i, j ) ) { // [3]
		        disconnect_nodes ( j, i ,m_NodeBuffer  );
		        disconnect_nodes ( j, tmpj ,m_NodeBuffer);
			if(!closes_loop(i, tmpj) && !is_connected(i, tmpj)){
			    connect_nodes    ( i, tmpj );
			    
			}
			counter2++;

		    }


		} // [2]

	    } // [1]
	    if ( f_distance_nodes( i, k ,m_NodeBuffer) <= MinDistForMerge) { // [1]

		if ( 2 == ( * m_NodeBuffer )[ k ].num_connected ) { // [2]
		    
		    for ( int ind = 0; ind < 2; ind++ ) { // [3]
			if ( i != ( * m_NodeBuffer )[ k ].connected[ ind ] ) { // [4]
			    tmpk = ( * m_NodeBuffer )[ k ].connected[ ind ];
// 			    if(tmp2==tmp1){continue;}
			}
		    }
		  
		    if ( angle_nodes( i, k, tmpk ) > angle_nodes( k, i, j ) ) { // [3]
                        disconnect_nodes ( k, i ,m_NodeBuffer);
			disconnect_nodes ( k, tmpk ,m_NodeBuffer);
			if(!closes_loop(i, tmpk)&& !is_connected(i, tmpk)){
			    connect_nodes    ( i, tmpk );
			}
			counter2++;

		    }


		} // [2]

	    } // [1]


    /*
	    // * Delete nodes that doesn't have any connection. * //
	    it = m_NodeBuffer->begin( );
	    while ( it != m_NodeBuffer->end( ) ) {
		
		if ( 0 == it->num_connected ) {
		    it = m_NodeBuffer->erase( it );
		}
		else {
		    it++;
		}

	    }
    */

	} // [0]
    
    }
  
  
  
  
   i=-1;
   Node_Buffer::iterator it1,it2;
   for ( it1 = m_NodeBuffer->begin( ); it1 != m_NodeBuffer->end( ); it1++ ) { // [0]
        ++i;
	if(it1->num_connected >= MAX_CONN ||it1->num_connected ==0){continue;}
	    int j=-1;
	    for ( it2 = m_NodeBuffer->begin( ); it2!= m_NodeBuffer->end( ); it2++ ) { // [0]
		  ++j;
		  if( i==j ||it2->num_connected ==0 ||(it2->num_connected +it1->num_connected) > MAX_CONN 
		    || is_connected(i,j ) ||closes_loop(i,j)){continue;}
		  
		  if(f_distance_nodes(i,j, m_NodeBuffer ) < MaxDistanceForMergeNode1){
		    
		      bool merge=true;
		     
		      for( int m=0; m<it2->num_connected; ++m ){
			  if(closes_loop(i, it2->connected[m])){merge=false;break;}
		      }
		      
		      if(merge==true){
			   for( int m=0; m<it2->num_connected; ++m ){
				disconnect_nodes(j, it2->connected[m],  m_NodeBuffer);
				connect_nodes(i, it2->connected[m] );
			    }
			    (* m_NodeBuffer)[i].f_x_pos +=(* m_NodeBuffer)[j].f_x_pos;
			    (* m_NodeBuffer)[i].f_y_pos +=(* m_NodeBuffer)[j].f_y_pos;
			    
			    (* m_NodeBuffer)[i].f_x_pos /=2.0;
			    (* m_NodeBuffer)[i].f_y_pos /=2.0;

		      }
		    
		      

		  }
	    }
   }
  
 

    return;

} // END of DeleteNodes METHOD

  

  
  void  FindNodes::InsertCrossPoints( /* IN_OUT Node_Buffer * m_NodeBuffer, */ )
// ****************************************************************************** //
{

    int a, b, c, d;
    int i, j;
    float x, y;
    int num_nodes;
    Node_Buffer::iterator it_old_end = m_NodeBuffer->end( );

    a = -1;
    Node_Buffer::iterator it_a;
    for ( it_a = m_NodeBuffer->begin( ); it_a != ( it_old_end - 1 ); it_a++ ) { // [0]
	a++;

	for ( i = 0; i < it_a->num_connected; i++ ) { // [1]

	    c = a;
	    Node_Buffer::iterator it_c;
	    for ( it_c = ( it_a + 1 ); it_c != it_old_end; it_c++ ) { // [2]
		c++;

		for ( j = 0; j < it_c->num_connected; j++ ) { // [3]

		    b = it_a->connected[ i ];
		    d = it_c->connected[ j ];

		    // * Create new node at intersection. * //
		    if ( intersect( a, b, c, d, &x, &y ) ) { // [4]

			// * Create a new node struct. * //
			Node node;
			node.x_pos = x;
			node.y_pos = y;
			node.f_x_pos = x;
			node.f_y_pos = y;
			node.num_connected = 0;
			m_NodeBuffer->push_back( node );

			num_nodes = m_NodeBuffer->size( ) - 1;

			disconnect_nodes( a, b ,m_NodeBuffer);
			disconnect_nodes( c, d ,m_NodeBuffer);
			connect_nodes( a, num_nodes );
			connect_nodes( b, num_nodes );
			connect_nodes( c, num_nodes );
			connect_nodes( d, num_nodes );

		    }

		} // [3]

	    } // [2]

	} // [1]

    } // [0]

    return;

}; // END of InsertCrossPoints METHOD

  // ****************************************************************************** //
 
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





void FindNodes::findReachableNodes(int startNode, std::vector<int>& reachable, int only_direction/*=-1*/, bool remove_first/*false*/)
{
      reachable.clear();
      std::vector<int> stack;
      stack.push_back(startNode);
      reachable.push_back(startNode);
      int cnt = 0;
      while(!stack.empty()){
	      int nidx = stack.back();
	      Node& n = (*m_NodeBuffer)[nidx];
	      stack.pop_back();
	      for(int i=0;i<n.num_connected;i++){
		      if(cnt == 0 && only_direction>=0 && i!=only_direction){
			      // search only in one direction from start point
			      continue;
		      }
		      int neigh = n.connected[i];
		      Node& n2  = (*m_NodeBuffer)[neigh]; // current node
		      std::vector<int>::reverse_iterator it = find(reachable.rbegin(),reachable.rend(),neigh);
		      if(it!=reachable.rend()){ 
			      // prevent circles
			      continue;
		      }
		      if(n2.num_connected > 2){
			      // this is a line ending, put it in the list, but do not add it to the stack!
			      while(n2.num_connected>0){
				      disconnect_nodes(neigh,n2.connected[0],m_NodeBuffer);
			      }
			      //reachable.push_back(neigh);
			      continue;
		      }
		      // now num_connected is either 1 or 2
		      if(n2.num_connected == 2){	
			      // if n2 has num_connected == 2, we need to put it on the stack for further investigation
			      stack.push_back(neigh);
		      }
		      reachable.push_back(neigh);  // its _reachable_ OK.
	      }
	      cnt++;
      }
      if(remove_first)
	      reachable.erase(reachable.begin()); // hmpf: expensive!
}



void FindNodes::findReachableNodes2(int startNode, LinearGraphComponent & lComp, int only_direction/*=-1*/)
{     
  
       lComp.reachableNodeIds.clear();
       lComp.Lines.clear();
       lComp.sumOfLength=0;
      
      std::vector<int> stack;
      stack.push_back(startNode);
      lComp.reachableNodeIds.push_back(startNode);
//       int cnt = 0;
      int l_id=0;
      while(!stack.empty()){
	      int nidx = stack.back();
	      Node& n = (* m_NodeBuffer_tmp)[nidx];
	      stack.pop_back();
	      
	      while(n.num_connected>0){
// 		      if(cnt == 0 && only_direction>=0 && i!=only_direction){
// 			      // search only in one direction from start point
// 			      continue;
// 		      }
		
		      int neigh = n.connected[0];
		      
		      Node& n2  = (* m_NodeBuffer_tmp)[neigh]; // current node
		      std::vector<int>::reverse_iterator it = find( lComp.reachableNodeIds.rbegin(), lComp.reachableNodeIds.rend(),neigh);
		      if(it == lComp.reachableNodeIds.rend()){ 
			      // prevent circles
			       lComp.reachableNodeIds.push_back(neigh);
		      }
		      
		      Vec2i p1( (* m_NodeBuffer_tmp)[nidx].f_x_pos, (* m_NodeBuffer_tmp)[nidx].f_y_pos );
		      Vec2i p2( (* m_NodeBuffer_tmp)[neigh].f_x_pos, (* m_NodeBuffer_tmp)[neigh].f_y_pos );
		      
		      lComp.Lines.push_back(Line(l_id++ ,  p1, p2, nidx, neigh));
		      lComp.sumOfLength += f_distance_nodes(nidx,neigh,m_NodeBuffer_tmp);
		      
		      if(n2.num_connected >= 2){
			  stack.push_back(neigh);
		      }
		      
		      disconnect_nodes(neigh, nidx, m_NodeBuffer_tmp);
		      
	      }
	    // cnt++;
      }
      
      
//       if(remove_first)
// 	      reachable.erase(reachable.begin()); // hmpf: expensive!
}


void FindNodes::connectMoreNodes(){
   Node_Buffer::iterator it1,it2;
   int i=-1;
   for ( it1 = m_NodeBuffer->begin( ); it1 != m_NodeBuffer->end( ); it1++ ) { // [0]
        ++i;
	if(it1->num_connected >= MAX_CONN){continue;}
	for( int l=0; l<it1->num_connected; ++l ){
	  
	    int j=-1;
	    for ( it2 = m_NodeBuffer->begin( ); it2!= m_NodeBuffer->end( ); it2++ ) { // [0]
	        ++j;
		if( i==j || it2->num_connected >= MAX_CONN ||is_reacherable(i, j)){continue;}
	        
		     for( int m=0; m<it2->num_connected; ++m ){
// 		        if(!is_connected(it1->connected[l],j) && !closes_loop(i,j)){
		              if(is_reacherable(i, j) ){continue;}
		              
			      Vec2i pi( (* m_NodeBuffer)[i].f_x_pos, (* m_NodeBuffer)[i].f_y_pos );
			      Vec2i pl( (* m_NodeBuffer)[it1->connected[l]].f_x_pos, (* m_NodeBuffer)[it1->connected[l]].f_y_pos );
			      Vec2i pj( (* m_NodeBuffer)[j].f_x_pos, (* m_NodeBuffer)[j].f_y_pos );
			      Vec2i pm( (* m_NodeBuffer)[it2->connected[m]].f_x_pos, (* m_NodeBuffer)[it2->connected[m]].f_y_pos );
			      Line a(1 ,  pi, pl);
			      Line b(1 ,  pj, pm);
			    

			      if( m_Math::AngDif( a.ang, b.ang) < AngleToMerge
			      &&  m_Math::getShortestDistance(a, b) <MaxLineSegDistance
			      &&  m_Math::GetProjectiveDistance(a.getMidPoint(), b) < MaxProjectedDistance
			      &&  m_Math::GetProjectiveDistance(b.getMidPoint(), a) < MaxProjectedDistance){
				
				      int id1,id2;
				
				      if(m_Math::Point2LineSegDistance(pi, b) < m_Math::Point2LineSegDistance(pl,b)){
					  id1 = i;
				      }
				      else { id1 =it1->connected[l] ;   }
				    
				      if(m_Math::Point2LineSegDistance(pj,a) < m_Math::Point2LineSegDistance(pm, a)){
					  id2=j;
				      }
				      else { id2 = it2->connected[m] ; }
				      
				      if( id1==id2){   cout<<"is reachable "<<i<<"   "<<id1<<"          "<<j<<"    "<<id2<<"    "<< is_reacherable(i,j)<<endl;   }
				    
// 				      if(is_reacherable(id1,id2)){continue;}
				      connect_nodes(id1,id2);
				      
				      update_reachable( id1, id2 );
				      
			      
			      }
		      
		    }

	      
	    }

	}

   }
   
   
   
   i=-1;
   for ( it1 = m_NodeBuffer->begin( ); it1 != m_NodeBuffer->end( ); it1++ ) { // [0]
        ++i;
	if(it1->num_connected >= MAX_CONN ||it1->num_connected ==0){continue;}

	   int j=-1;
	    for ( it2 = m_NodeBuffer->begin( ); it2!= m_NodeBuffer->end( ); it2++ ) { // [0]
		  ++j;
		  if( i==j ||it2->num_connected ==0 ||(it2->num_connected +it1->num_connected) > MAX_CONN || is_reacherable(i, j)){continue;}
		  
		  if(f_distance_nodes(i,j, m_NodeBuffer ) < MaxDistanceForMergeNode2){
		    
		      for( int m=0; m<it2->num_connected; ++m ){
			  disconnect_nodes(j, it2->connected[m],  m_NodeBuffer);
			  connect_nodes(i, it2->connected[m] );

		      }
		      (* m_NodeBuffer)[i].f_x_pos +=(* m_NodeBuffer)[j].f_x_pos;
		      (* m_NodeBuffer)[i].f_y_pos +=(* m_NodeBuffer)[j].f_y_pos;
		      
		      (* m_NodeBuffer)[i].f_x_pos /=2.0;
		      (* m_NodeBuffer)[i].f_y_pos /=2.0;
		      
		      
			update_reachable( i, j );

		  }
	    }
   }
  
}



void FindNodes::mainLoop(FrameGrabber & CamFrm){

    LINE_WIDTH= params.graphNode.LineWidth->get();
    SMALL_ANG = params.graphNode.SmallAngle->get();
    int MinCmpLineLength = params.graphNode.MinCmpLineLength->get();
    AngleToMerge = params.graphNode.AngleToMerge->get();
    MaxLineSegDistance = params.graphNode.MaxLineSegDistance->get();
    MaxProjectedDistance = params.graphNode.MaxProjectedDistance->get();
    MaxDistanceForMergeNode1 = params.graphNode.MaxDistanceForMergeNode1->get();
    MaxDistanceForMergeNode2 = params.graphNode.MaxDistanceForMergeNode2->get();
  
   
    UNIT = CamFrm.MaximunNodeNum;
    findNodes(CamFrm.skeletonPixelMatrix);
    //   cout<<"num of nodes  "<<m_NodeBuffer->size( )<<endl;
    ConnectTouchingNodes (CamFrm.skeletonPixelMatrix);

    //   cout<<"1 Num of nodes: "<< m_NodeBuffer->size()<<endl;
    FindMoreNodes(CamFrm.skeletonPixelMatrix);
    //   cout<<"2 More Nodes nodes: "<< m_NodeBuffer->size()<<endl;
    FindCloseNodes( CamFrm.weightedWhiteValues );
    ConnectCloseNodes (CamFrm.weightedWhiteValues);
    SmoothNodes ();
    DeleteNodes ();

  
    InsertCrossPoints();
    //   SmoothNodes2 ( /* IN_OUT Node_Buffer * m_NodeBuffer, */ ) ;


    m_NodeBuffer_tmp->clear();

    Node_Buffer::iterator it;
    for ( it = m_NodeBuffer->begin( ); it != m_NodeBuffer->end(); it++ ) { // [1]
	Node node= (*it);
	m_NodeBuffer_tmp->push_back(node);

    }

    //   PrintNodeBuffer("NodeBuffer_tmp1.dat", m_NodeBuffer_tmp );


    m_LinearGraph_Buffer->clear();
    int i=-1;
    for ( it =  m_NodeBuffer_tmp->begin( ); it !=  m_NodeBuffer_tmp->end(); it++ ) { // [1]
	++i;
	LinearGraphComponent linearGraphComp;
	findReachableNodes2(i, linearGraphComp, -1);
	if(linearGraphComp.reachableNodeIds.size()>1){
		m_LinearGraph_Buffer->push_back( linearGraphComp );
	}
    }
  

//       PrintNodeBuffer("NodeBuffer.dat", m_NodeBuffer );


      connectMoreNodes();
//       PrintReacherableComp("ReachableComp.dat" );
      
// 		     else if( m_Math::AngDif( a.ang, b.ang) > (90- AngleToMerge)
// 		          &&  m_Math::getShortestDistance(a, b) <MaxLineSegDistanceForJoint){
// 			
// 		       	    for( int k=0; k< s2; ++k){
// 				  Line c =  (*m_LinearGraph_Buffer)[n].Lines[k];
// 				  c.id += (s1);
// 				  (*m_LinearGraph_Buffer)[m].Lines.push_back (c);
// 				  
// 			    }
// 			    
// 			    for(unsigned int k=0; k< (*m_LinearGraph_Buffer)[n].reachableNodeIds.size(); ++k){
// 				  int  nodeId =  (*m_LinearGraph_Buffer)[n].reachableNodeIds[k];
// 				  (*m_LinearGraph_Buffer)[m].reachableNodeIds.push_back (nodeId);
// 				  
// 			    }
// 			    (*m_LinearGraph_Buffer)[m].sumOfLength += (*m_LinearGraph_Buffer)[n].sumOfLength;
// 			    (*m_LinearGraph_Buffer)[n].reachableNodeIds.clear();
// 			    (*m_LinearGraph_Buffer)[n].Lines.clear();
// 			    (*m_LinearGraph_Buffer)[n].sumOfLength =0;
// 			    
// 			    merge=1;
// 			
// 			
// 			    if( m_Math::getShortestDistance(a, b) >0){
// 				
// 				Vec2i intersetP;
// 				Node node;
// 				node.x_pos = intersetP[0];
// 				node.y_pos = intersetP[1];
// 				node.f_x_pos = intersetP[0];
// 				node.f_y_pos = intersetP[1];
// 				node.num_connected = 0;
// 				m_NodeBuffer->push_back( node );
// 
// 				int nodeIds = m_NodeBuffer->size( ) - 1;
// 				int nodeIde;
// 				
// 				
// 				
// 				
// 				if( m_Math::intersect(  a,  b , intersetP)){
// 				    Vec2i NewEdge_s, NewEdge_e1, NewEdge_e2;
// 				    float dist ; float d;
// 				    NewEdge_s = intersetP;
// 				    
// 				    dist = m_Math::TwoPointDistance( intersetP, b.s);   NewEdge_e1 = b.s; nodeIde = b.nodeId_s;
// 				    if((d = m_Math::TwoPointDistance( intersetP, b.e))< dist){dist =d;  NewEdge_e1 = b.e;nodeIde = b.nodeId_e;}
// 				      
// 				    dist = m_Math::TwoPointDistance( intersetP, a.s);  NewEdge_e2 = a.s;nodeIde = a.nodeId_s;
// 				    if((d = m_Math::TwoPointDistance( intersetP, a.e))< dist){dist =d;  NewEdge_e2 = a.e;nodeIde = a.nodeId_e;}
// 				    
// 				    
// 				    if( m_Math::Point2LineSegDistance(intersetP, a)==0  ){
// 					  (*m_LinearGraph_Buffer)[m].Lines.push_back ( Line( (*m_LinearGraph_Buffer)[m].Lines.size(), NewEdge_s, NewEdge_e1, nodeIds, nodeIde));  
// 				    }
// 				    else if(m_Math::Point2LineSegDistance(intersetP, b)==0 ){
// 				          (*m_LinearGraph_Buffer)[m].Lines.push_back ( Line( (*m_LinearGraph_Buffer)[m].Lines.size(), NewEdge_s, NewEdge_e2, nodeIds, nodeIde)); 
// 				    }
// 				    else {
// 				      (*m_LinearGraph_Buffer)[m].Lines.push_back ( Line( (*m_LinearGraph_Buffer)[m].Lines.size(), NewEdge_s, NewEdge_e1, nodeIds, nodeIde)); 
// 				      (*m_LinearGraph_Buffer)[m].Lines.push_back ( Line( (*m_LinearGraph_Buffer)[m].Lines.size(), NewEdge_s, NewEdge_e2, nodeIds, nodeIde)); 
// 				      
// 				    }

      
      
            
      vector<Line> alllines;
      LinearGraph_Buffer::iterator it_;
      for ( it_ = m_LinearGraph_Buffer->begin( ); it_ != m_LinearGraph_Buffer->end( ); it_++ ) { // [1]
	  vector<Line> ls = it_->Lines;
	  for(unsigned int lidx=0; lidx<ls.size();++lidx){
	    
	    alllines.push_back(ls[lidx] );
	    
	  }
      }
      
      
     float dist = params.graphNode.SamplePointDist->get();
     m_Math::SamplePointsOnLines(alllines, dist, NodeSamplePoins);
      
    undistortedNodeSamplePoins.clear();
    distortionModel.UndistortP(NodeSamplePoins, undistortedNodeSamplePoins);
  
//     NodeSamplePoins.clear();
//     distortionModel.DistortP(undistortedNodeSamplePoins, NodeSamplePoins);
//     undistortedNodeSamplePoins.clear();
//     distortionModel.UndistortP(NodeSamplePoins, undistortedNodeSamplePoins);

  
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

      LINE_WIDTH= params.graphNode.LineWidth->get();
    
    
    
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
float FindNodes::f_distance_nodes( /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j ,Node_Buffer * nodeBuffer )
// ****************************************************************************** //
{
    float x_dist, y_dist;

    x_dist = ( ( * nodeBuffer )[ i ].f_x_pos - ( * nodeBuffer )[ j ].f_x_pos );
    y_dist = ( ( * nodeBuffer )[ i ].f_y_pos - ( * nodeBuffer )[ j ].f_y_pos );

    return ( x_dist * x_dist ) + ( y_dist * y_dist );

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


int FindNodes::black_line_value ( /* in */ const float ( & weightedWhiteValues )[ H ][ W ], /* IN const Node_Buffer * m_NodeBuffer, */ /* in */ const int i, /* in */ const int j )
// ****************************************************************************** //
{
    int xm;
    int ym;
    int vm, vij;
    int res;

    // * Compute coordinates of pixel (and its grey-value) in between node_i and node_j * //
    xm = ( ( ( * m_NodeBuffer )[ i ].x_pos + ( * m_NodeBuffer )[ j ].x_pos + 1 ) >> 1 );
    ym = ( ( ( * m_NodeBuffer )[ i ].y_pos + ( * m_NodeBuffer )[ j ].y_pos + 1 ) >> 1 );
    vm = weightedWhiteValues[ ym ][  xm ];

    // * Compute sum of grey-values of node_i and node_j and pixel in between * //
    vij = vm + weightedWhiteValues[ ( * m_NodeBuffer )[ i ].y_pos ][ ( * m_NodeBuffer )[ i ].x_pos ] 
	      + weightedWhiteValues[ ( * m_NodeBuffer )[ j ].y_pos ][ ( * m_NodeBuffer )[ j ].x_pos ];

    if ( vij == 0 ) {
	return 0;
    }
    // * Compute res in (0, 63). The larger res is, the darker is the middle pixel,
    // * compared to the average. (=Desired case) * //
    res = ( min( 255, ( vm * 765 ) / vij ) );

    return ( res * res ) >> 2;

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

     string pkg_path = ros::package::getPath("");
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
	      
	      CompInfo<<i<<":    ";
	      CompInfo<<"ids: ";
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
	  