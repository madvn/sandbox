# Specify the minimum required version of cmake needed to build this project
cmake_minimum_required(VERSION 3.14.0)
# Define the project name, version, and supported languages
project(cmake_template VERSION 0.1.0 LANGUAGES CXX)

# Forces any exported shared libraries to be prepended with the project name so they are unique
set(CMAKE_SHARED_LIBRARY_PREFIX "${CMAKE_SHARED_LIBRARY_PREFIX}_${PROJECT_NAME}_")

# Set the default build type
# xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif ()

# Build the tests if this is not a release
if (NOT CMAKE_BUILD_TYPE MATCHES .*Rel.*)
    set(BUILD_TESTS on)
endif ()

# Set the default c++ standard
# xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
set(CMAKE_CXX_STANDARD 17)

# Include testing capabilities
# xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
if (BUILD_TESTS)
    include(CTest)
    enable_testing()
endif()

include(GNUInstallDirs)

# Building the libraries and executables
# xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
set(components) # This variable will accumulate component names
# Directory order is important for resolving internal component dependencies
add_subdirectory(modules/template_library)
add_subdirectory(modules/examples/operator)
add_subdirectory(modules/examples/addition)
add_subdirectory(modules/examples/calculator)
add_subdirectory(modules/examples/calculator-cli)
add_subdirectory(apps/examples/calculator-runner)
add_subdirectory(apps/examples/hybridization)

if (BUILD_TESTS)
    add_subdirectory(test)
endif ()

# Iterate over the itens in the components variable and print them
message(STATUS "${PROJECT_NAME} configured components:")
foreach(_comp ${components})
    message(STATUS "${_comp}")
endforeach()

# Installation Configuration
# xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
# Generage a project configuration file from the template ([...].cmake.in file)
configure_file(${PROJECT_NAME}Config.cmake.in
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    @ONLY
    )

# Generate a version file for this project
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Specify where this project config files are installed
install(
    FILES
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# CPack Options for generating installers
# xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
set(CPACK_GENERATOR DEB)
set(CPACK_PACKAGE_VENDOR "Path Robotics")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/deb/doc")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "CMake project template and examples.")
set(CPACK_PACKAGE_CONTACT "developers@path-robotics.com")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "/usr")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "") # comma separated list of dependent runtime packages in the form '<package name> (>= <package version>)'
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/deb/postinst;${CMAKE_CURRENT_SOURCE_DIR}/deb/postrm")

include(CPack)
