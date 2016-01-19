cmake_minimum_required(VERSION 2.8.3)
project(visual_tracking)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	tf 
	image_transport 
	message_generation 
	config_server 
	cmake_modules 
	cv_bridge
	eigen_conversions
	std_msgs
	pcl_ros
)

find_library(GSL_LIBRARY
	NAMES gsl libgsl
	PATHS  ${GSL_POSSIBLE_ROOT_DIRS}
	PATH_SUFFIXES lib
	DOC "GSL library"
)
FIND_LIBRARY(GSLCBLAS_LIBRARY
	NAMES gslcblas libgslcblas
	PATHS  ${GSL_POSSIBLE_ROOT_DIRS}
	PATH_SUFFIXES lib
	DOC "GSL cblas library"
)


find_package(Boost REQUIRED COMPONENTS system chrono timer)


include_directories(include
   ${catkin_INCLUDE_DIRS}
   ${OpenCV_INCLUDE_DIRS})
   
catkin_package(
        INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime
	eigen_conversions
	LIBRARIES vision_tracking
)

find_package(GTK2)
if(GTK2_FOUND)
	include_directories(${GTK2_INCLUDE_DIRS})
	add_definitions(-DHAVE_GTK)
endif()


find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


find_package(OpenCV REQUIRED)


install(FILES Black.lut Objects.lut White.lut #maskForCamera plugin_nodelet.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(visual_tracking_node src/vision_main.cpp)
target_link_libraries(visual_tracking_node ${catkin_LIBRARIES} ${GTK2_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} visual_tracking)


add_library(visual_tracking
	src/rgb_yuyv/yuvClasses.cpp 
	
	src/Parameters/camera_parameters.cpp
	src/Parameters/Parameters.cpp
	
	src/FieldObjects/findField.cpp
	src/FieldObjects/findGoalPosts.cpp
	
	src/Projection/ForwardProjection.cpp
	src/Projection/BackwardProjection.cpp
	src/Projection/DistortionModel.cpp

        
        src/ParticleFilter/singleParticle.cpp
        src/ParticleFilter/ParticleFilter.cpp
        src/ParticleFilter/PclClustering.cpp
        
        src/Tools/VisionRate.cpp
        src/Tools/tools.cpp
        src/Tools/m_Math.cpp
        
        src/camera/Camera.cpp
        src/camera/CameraDummy.cpp
        src/camera/MatPublisher.cpp 
        
        src/LineMatcher/lineMatcher.cpp
        src/LineMatcher/findLines.cpp
        src/LineMatcher/findNodes.cpp

        
        src/CalculatePose/calculatePose.cpp 
        src/CalculatePose/kalmanFilter.cpp 
        src/CalculatePose/hillClimbing.cpp 
        
        src/DataAssociation/dataAssociation.cpp 
        
        src/IcpPointMatcher/icp.cpp
        src/IcpPointMatcher/icpPointToPlane.cpp
        src/IcpPointMatcher/icpPointToPoint.cpp
        src/IcpPointMatcher/kdtree.cpp
        src/IcpPointMatcher/matrix.cpp
        

    )
 target_link_libraries(visual_tracking 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}  
    ${Boost_LIBRARIES} 	
    ${GSL_LIBRARY}
    ${GSLCBLAS_LIBRARY})   
    

install(TARGETS visual_tracking visual_tracking_node
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


        
add_executable(bgr2yuyv src/rgb_yuyv/rgb2yuyv.cpp)
target_link_libraries(bgr2yuyv ${catkin_LIBRARIES} ${GTK2_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
    
        

