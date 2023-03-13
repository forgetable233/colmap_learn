cmake_minimum_required(VERSION 3.16)
project(test)

set(CMAKE_CXX_STANDARD 14)


find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})
include_directories("/usr/lib/jvm/java-17-openjdk-amd64/include")
include_directories("/usr/lib/jvm/java-17-openjdk-amd64/include/linux")

find_path(MYSQL_ROOT_DIR mysql)
include(FindPackageHandleStandardArgs)
include_directories(/usr/include/mysql)
include_directories(/usr/include/cppconn)
link_directories(/usr/lib/x86_64-linux-gnu)
find_package_handle_standard_args(mysql REQUIRED_VARS MYSQL_ROOT_DIR)

add_executable(test main.cpp
        img_loader.cc img_loader.h
        math_functions.cpp math_functions.h
        thresholds.h
        point2d.cpp point2d.h
        camera_model.cpp camera_model.h
        edge.cpp edge.h
        incremental_rebuild.cpp incremental_rebuild.h
        correspondence_graph.cpp correspondence_graph.h
        point3d.cpp point3d.h
        point_viewer.cpp point_viewer.h
        reprojection_error.cpp reprojection_error.h
        mysql.h mysql.cpp)

add_executable(testJava testSQL.cpp mysql.cpp mysql.h)

#target_include_directories(testJava PRIVATE ${MYSQL_ROOT_DIR}/include)
#target_link_directories(testJava PRIVATE ${MYSQL_ROOT_DIR}/lib)
target_link_libraries(testJava mysqlclient)
target_link_libraries(testJava mysqlcppconn)

target_link_libraries(test
        PRIVATE Eigen3::Eigen
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        mysqlclient
        mysqlcppconn)

#target_link_libraries(test mysqlclient mysqlcppconn)

