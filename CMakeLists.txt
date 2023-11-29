cmake_minimum_required(VERSION 3.8)
project(rawproxy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(CycloneDDS REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rclcpp REQUIRED)

find_package(tutorial_interfaces REQUIRED)                      # CHANGE

set(IDL_FILE_PATH "${CMAKE_SOURCE_DIR}/Ddstype.idl")
message("Set default idl file path: ${IDL_FILE_PATH}")
set(IDL_LIB_NAME "${IDL_FILE_NAME}_lib")
idlc_generate(TARGET ${IDL_LIB_NAME} FILES ${IDL_FILE_PATH})

add_executable(rawproxy src/rawproxy.cpp)
ament_target_dependencies(rawproxy rclcpp tutorial_interfaces)
target_link_libraries(rawproxy ${IDL_LIB_NAME} CycloneDDS::ddsc)    # CHANGE

install(TARGETS
  rawproxy
  DESTINATION lib/${PROJECT_NAME})

ament_package()
