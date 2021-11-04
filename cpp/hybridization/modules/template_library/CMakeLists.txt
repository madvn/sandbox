# Define the component name
set(component template_library)
# Add this component to the list of components in the project
set(components ${components} ${component} PARENT_SCOPE)

# Find packages that this component depends on
#find_package(<package-name> <pakage-version> REQUIRED COMPONENTS <package-components>)

# Define the library that is built from this component
# SHARED - generates a daynamicly loadable .so library - referenced by other libs/executables
# STATIC - generates a compiled object file .a archive - directly included in other libs/executables
# INTERFACE - defines a header only library - headers would be #include[d] in other projects
# list each .cpp file which makes up the library
add_library(${component} SHARED
  #<src files>
  src/template_class.cpp
  )

# Define an alias to refer to this target
# This alias is used as a link target in other components of this project
# The structure of the alias matches the exported target (which would be used by other projects)
add_library(${PROJECT_NAME}::${component} ALIAS ${component})

# Specify which directories to search for #include[d] header files
# PRIVATE - Directories are used only when compiling this target,
# PUBLIC - Directories are used when compiling this target or linking to this target.
#          Directories containing your public header files (ones #include[d] from .cpp files)
#          and other header files (from finded packages) which are #include[d] in your public headers.
#          These also become part of the INTERFACE include directories
# INTERFACE - Directories containing your public headers which are not #include[d] in any .cpp files
# public and interface directories listed here become part of the include directores for any projects whih link against this library
target_include_directories(${component}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  #<finded packages include dirs; entries should be omitted for exported targets listed in target_link_libraries below (commonly ${package_name_INCLUDE_DIRS})>
  )

# Specify which directories downstream projects should search to find this library
target_link_directories(${component}
  INTERFACE
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}>
  )

# Specify which libraries to link against when building this library
# PRIVATE - only used when building this library
# PUBLIC - used when building this library and when building libraries which link against it
# INTERFACE - only used when linking against this library
target_link_libraries(${component}
  PUBLIC
  #<finded packages libraries; prefer exported targets over cmake variables (commonly package-name::component or ${package_name_LIBRARIES})>
  )

# Define which targets are exported from this component and where they are installed
install(TARGETS ${component} EXPORT ${component}Targets
  COMPONENT ${component}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )

# Generate a Targets.cmake file which lists your exported targets so they can be found by other projects
install(EXPORT ${component}Targets
  FILE "${PROJECT_NAME}-${component}Targets.cmake"
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
  COMPONENT ${component}
  )

# Generate a configuration file for this component from the template ([...].cmake.in file)
configure_file("${component}Config.cmake.in"
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-${component}Config.cmake"
  @ONLY
  )

# Specify where the component configuration file is installed
install(
  FILES
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-${component}Config.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
  COMPONENT ${component}
)

# Specify which headers get installed (these are your PUBLIC and INTERFACE headers)
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  TYPE INCLUDE
  COMPONENT ${component}
  FILES_MATCHING PATTERN "*.hpp"
)

# Process the CMakeLists file in the test directory
if (BUILD_TESTS)
  add_subdirectory(test)
endif ()
