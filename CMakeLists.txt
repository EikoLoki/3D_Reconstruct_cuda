CMAKE_MINIMUM_REQUIRED( VERSION 2.8 ) #设定版本
PROJECT( ar ) #设定工程名
SET( CMAKE_CXX_COMPILER "g++") #设定编译器
	
#设定可执行二进制文件的目录
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 

#并且把该目录设为连接目录
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)
	
#设定头文件目录
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)

# 增加opencv的依赖
FIND_PACKAGE( OpenCV 3.4 )
# add pcl dependency
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )


# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )


ADD_EXECUTABLE( disparity ./src/disparity.cpp )
TARGET_LINK_LIBRARIES( disparity ${OpenCV_LIBS})

ADD_EXECUTABLE( generatePointCloud ./src/generatePointCloud.cpp )
TARGET_LINK_LIBRARIES( generatePointCloud ${OpenCV_LIBS} ${PCL_LIBRARIES} )


ADD_EXECUTABLE( depth ./src/depth.cpp )
TARGET_LINK_LIBRARIES( depth ${OpenCV_LIBS})

ADD_EXECUTABLE( undistortion ./src/undistortion.cpp )
TARGET_LINK_LIBRARIES( undistortion ${OpenCV_LIBS})

add_executable( blurDetection ./src/blurDetection.cpp )
target_link_libraries(blurDetection ${OpenCV_LIBS})

# add_executable( stereo_calibration ./src/stereoCalibration.cpp)
# target_link_libraries( stereo_calibration ${OpenCV_LIBS})
