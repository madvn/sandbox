find_package(GTest REQUIRED)

include(GoogleTest)

add_executable(${PROJECT_NAME}_${component}_test
  test_main.cpp
  test_calculator.cpp
  )
#target_include_directories(${PROJECT_NAME}_${component}_test
#    )
target_link_libraries(${PROJECT_NAME}_${component}_test
  ${PROJECT_NAME}::calculator
  GTest::GTest
  GTest::Main
  )
gtest_discover_tests(${PROJECT_NAME}_${component}_test)