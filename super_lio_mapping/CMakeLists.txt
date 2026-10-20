cmake_minimum_required(VERSION 3.16)
project(super_lio_mapping LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} SHARED src/store.cpp src/export.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} Threads::Threads)

add_executable(superm_export src/export_main.cpp)
target_link_libraries(superm_export ${PROJECT_NAME})
add_executable(mapping_check tests/mapping_check.cpp)
target_link_libraries(mapping_check ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} EXPORT mappingTargets LIBRARY DESTINATION lib)
install(TARGETS superm_export mapping_check DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(mappingTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL Threads)
ament_package()
