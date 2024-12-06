cmake_minimum_required(VERSION 3.5)
project(gyakuenki_cpp)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jitsuyo REQUIRED)
find_package(keisan REQUIRED)
find_package(ninshiki_interfaces REQUIRED)
find_package(gyakuenki_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/node/gyakuenki_cpp_node"
  "src/${PROJECT_NAME}/projections/ipm.cpp"
  "src/${PROJECT_NAME}/utils/camera_info.cpp")

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  OpenCV
  jitsuyo
  keisan
  ninshiki_interfaces
  gyakuenki_interfaces)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(main "src/gyakuenki_cpp_main.cpp")
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(main ${PROJECT_NAME})

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  OpenCV
  jitsuyo
  keisan
  ninshiki_interfaces
  gyakuenki_interfaces)

ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})

ament_package()
