cmake_minimum_required(VERSION 3.8)
project(lgs_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(QT6 /home/josue/Qt/6.4.0)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(QT_DIR ${QT6}/gcc_64/lib/cmake/Qt6)
set(Qt6_DIR ${QT6}/gcc_64/lib/cmake/Qt6)
set(Qt6CoreTools_DIR ${QT6}/gcc_64/lib/cmake/Qt6CoreTools/)
set(Qt6GuiTools_DIR ${QT6}/gcc_64/lib/cmake/Qt6GuiTools/)

#set runtime path
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# find dependencies
find_package(QT NAMES Qt6 REQUIRED COMPONENTS Core Gui Multimedia MultimediaWidgets Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Multimedia MultimediaWidgets Widgets)

find_package(pipecrawler REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

set(PROJECT_SOURCES
        src/main.cpp
        src/lgs_action_cli.cpp
        src/lgs_action_cli.h
        src/control_panel.cpp
        src/control_panel.h
        src/manager.h
        src/components/module_button.cpp
        src/components/module_button.h
)

add_executable(user_interface ${PROJECT_SOURCES})


target_include_directories(user_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(user_interface PUBLIC c_std_99 cxx_std_17) 

target_link_libraries(user_interface PUBLIC
  Qt::Core
  Qt::Gui
  Qt::Multimedia
  Qt::MultimediaWidgets
  Qt::Widgets
)

ament_target_dependencies(user_interface PUBLIC 
  pipecrawler
  rclcpp
  rclcpp_action
  std_msgs
)

install(TARGETS
  user_interface
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
