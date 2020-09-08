cmake_minimum_required(VERSION 3.5)

project(hospital_robot LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED COMPONENTS Widgets)
find_package(catkin REQUIRED COMPONENTS rviz roscpp geometry_msgs sensor_msgs jackal_msgs actionlib_msgs)
find_package(OpenCV REQUIRED)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(hospital_robot
    main.cpp
    mainwindow.cpp
    mainwindow.h
    rosthread.cpp
    rosthread.h
    emgthread.cpp
    emgthread.h
    statusthread.cpp
    statusthread.h
    goalthread.cpp
    goalthread.h
    tempthread.cpp
    tempthread.h
    resource.qrc
    mainwindow.ui
  )

target_link_libraries(hospital_robot ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} Qt5::Widgets)
