set(component hybridization)
set(components ${components} ${component} PARENT_SCOPE)

find_package(Open3D REQUIRED)
find_package(OpenMP REQUIRED)
find_package(poisson-recon)

add_executable(${component}
  src/main.cpp
  )

target_link_libraries(${component}
  PRIVATE
  OpenMP::OpenMP_CXX
  ${Open3D_LIBRARIES}
  poisson-recon::poisson-recon
  )

target_include_directories(${component}
  PRIVATE
  ${Open3D_INCLUDE_DIRS}
  )

install(TARGETS ${component} EXPORT ${component}Targets
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )

install(EXPORT ${component}Targets
  FILE "${PROJECT_NAME}-${component}Targets.cmake"
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
  COMPONENT ${component}
  )

configure_file("${component}Config.cmake.in"
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-${component}Config.cmake"
  @ONLY
  )

install(
  FILES
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-${component}Config.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
  COMPONENT ${component}
)

if (BUILD_TESTS)
  #    add_subdirectory(test)
endif ()