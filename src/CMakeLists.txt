ADD_EXECUTABLE( undistortion undistortion.cpp )
TARGET_LINK_LIBRARIES( undistortion 3D-Reconstruct)

ADD_EXECUTABLE( disparity disparity.cpp)
TARGET_LINK_LIBRARIES( disparity 3D-Reconstruct)

ADD_EXECUTABLE( depth depth.cpp)
TARGET_LINK_LIBRARIES( depth 3D-Reconstruct)

ADD_EXECUTABLE( generatePointCloud generatePointCloud.cpp)
TARGET_LINK_LIBRARIES( generatePointCloud 3D-Reconstruct )
