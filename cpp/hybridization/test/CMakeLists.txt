find_package(GTest REQUIRED)

include(GoogleTest)

add_executable(${PROJECT_NAME}_test
  test_main.cpp
  test_template_library.cpp
  test_calculator.cpp
  )
#target_include_directories(${PROJECT_NAME}_test
#    )
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}::template_library
  ${PROJECT_NAME}::calculator
  ${PROJECT_NAME}::addition
  GTest::GTest
  GTest::Main
  )
gtest_discover_tests(${PROJECT_NAME}_test)