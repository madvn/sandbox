set(component calculator-runner)
set(components ${components} ${component} PARENT_SCOPE)

add_executable(${component}
    src/main.cpp
    )

target_link_libraries(${component}
    PRIVATE
    ${PROJECT_NAME}::calculator-cli
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