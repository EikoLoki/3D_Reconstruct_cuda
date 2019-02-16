ADD_LIBRARY( 3D-Reconstruct SHARED
    config.cpp
    stereoCamera.cpp
    rectify.cpp
    disparity.cpp
    depth.cpp
    pcd_saver.cpp
    common.h)

TARGET_LINK_LIBRARIES(3D-Reconstruct ${THIRD_PARTY_LIBRARIES})
