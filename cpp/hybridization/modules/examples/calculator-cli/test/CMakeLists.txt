find_package(GTest REQUIRED)

include(GoogleTest)

add_executable(${PROJECT_NAME}_${component}_test
  test_main.cpp
  test_calculator-cli.cpp
  )
#target_include_directories(${PROJECT_NAME}_${component}_test
#    )
target_link_libraries(${PROJECT_NAME}_${component}_test
  ${PROJECT_NAME}::calculator-cli
  ${PROJECT_NAME}::calculator
  ${PROJECT_NAME}::addition
  GTest::GTest
  GTest::Main
  )
gtest_discover_tests(${PROJECT_NAME}_${component}_test)