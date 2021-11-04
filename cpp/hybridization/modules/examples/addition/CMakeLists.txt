set(component addition)
set(components ${components} ${component} PARENT_SCOPE)

#find_package(<package-name> <pakage-version> REQUIRED COMPONENTS <package-components>)

add_library(${component} SHARED
  #<src files>
  src/addition.cpp
  )
add_library(${PROJECT_NAME}::${component} ALIAS ${component})

target_include_directories(${component}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  #<finded packages include dirs (commonly ${package_name_INCLUDE_DIRS})>
  )
target_link_directories(${component}
  PUBLIC
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}>
  )
target_link_libraries(${component}
  PUBLIC
  ${PROJECT_NAME}::operator
  #<finded packages libraries (commonly ${package_name_LIBRARIES})>
  )

install(TARGETS ${component} EXPORT ${component}Targets
  COMPONENT ${component}
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

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  TYPE INCLUDE
  COMPONENT ${component}
  FILES_MATCHING PATTERN "*.hpp"
)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()