CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(cairomm cairomm-1.0)
INCLUDE_DIRECTORIES(${cairomm_INCLUDE_DIRS})
LINK_DIRECTORIES(${cairomm_LIBRARY_DIRS})

PKG_CHECK_MODULES(gtkmm REQUIRED gtkmm-3.0)
INCLUDE_DIRECTORIES(${gtkmm_INCLUDE_DIRS})
LINK_DIRECTORIES(${gtkmm_LIBRARY_DIRS})

PKG_CHECK_MODULES(gsl REQUIRED gsl)
INCLUDE_DIRECTORIES(${gsl_INCLUDE_DIRS})
LINK_DIRECTORIES(${gsl_LIBRARY_DIRS})

PKG_CHECK_MODULES(opencv REQUIRED opencv)
INCLUDE_DIRECTORIES(${opencv_INCLUDE_DIRS})
LINK_DIRECTORIES(${opencv_LIBRARY_DIRS})

LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/lib)

SET( LIBS_NEEDED -lopencv_core -lopencv_imgproc -lopencv_objdetect -lopencv_ml -lopencv_video) #

SET( SOURCE_FILES arrow.cpp imagecv.cpp progeo.c image_utils.cpp observer.cpp blob.cpp  cvutils.cpp  main.cpp  movement_model.cpp  stats.cpp  vehicle_model.cpp background_model.cpp  camera_model.cpp planar_geom.cpp  trafficmonitor_algorithm.cpp  viewgtk.cpp viewfile.cpp view.cpp classifier.cpp  feature_tracking.cpp proximity_tracking.cpp model.cpp  road_detection.cpp  vehicle.cpp database.cpp trafficmonitor_config.cpp rectify.cpp svm.cpp)

#SET( CMAKE_CXX_FLAGS "-O0 -g -std=c++11 -pg")
SET( CMAKE_CXX_FLAGS " -O3 -std=c++11")

INCLUDE_DIRECTORIES(
  ${cairomm_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR})

ADD_EXECUTABLE (trafficmonitor ${SOURCE_FILES})

TARGET_LINK_LIBRARIES( trafficmonitor
  ${LIBS_NEEDED}
  ${opencv_LIBRARIES}
  ${gtkmm_LIBRARIES}
  ${cairomm_LIBRARIES}
  ${gsl_LIBRARIES}
  )
