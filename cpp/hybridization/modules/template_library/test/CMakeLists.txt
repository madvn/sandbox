find_package(GTest REQUIRED)

include(GoogleTest)

add_executable(${PROJECT_NAME}_${component}_test
  test_main.cpp
  test_template_class.cpp
  )
#target_include_directories(${PROJECT_NAME}_${component}_test
#    )
target_link_libraries(${PROJECT_NAME}_${component}_test
  ${PROJECT_NAME}::template_library
  GTest::GTest
  GTest::Main
  )
gtest_discover_tests(${PROJECT_NAME}_${component}_test)