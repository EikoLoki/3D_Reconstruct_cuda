ADD_LIBRARY( 3D-Reconstruct SHARED
    config.cpp
    stereoCamera.cpp
    rectifier.cpp
    disparityCalculator.cpp
    depthCalculator.cpp
    pcdSaver.cpp)


TARGET_LINK_LIBRARIES(3D-Reconstruct ${THIRD_PARTY_LIBRARIES} ${libSGM_LIB})
