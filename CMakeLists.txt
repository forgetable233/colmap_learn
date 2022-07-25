cmake_minimum_required(VERSION 3.22)
project(test)

set(CMAKE_CXX_STANDARD 14)


find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED)
#find_package(PCL 1.9 REQUIRED COMPONENTS common io)

add_executable(test main.cpp
        img_loader.cc img_loader.h
        math_functions.cpp math_functions.h
        thresholds.h
        point2d.cpp point2d.h
        camera_model.cpp camera_model.h
        edge.cpp edge.h
        incremental_rebuild.cpp incremental_rebuild.h
        correspondence_graph.cpp correspondence_graph.h
        point3d.cpp point3d.h)

target_link_libraries(test PRIVATE Eigen3::Eigen  ${OpenCV_LIBS} ${PCL_LIBRARIES})

