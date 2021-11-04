#include <cmake_template/calculator/calculator.h>

#include "foo_operator.h"

#include <gtest/gtest.h>

namespace cmake_template
{
namespace calculator
{
class TestCalculator : public ::testing::Test
{
protected:
  Calculator calculator;

  void SetUp()
  {
  }
  void TearDown()
  {
  }
};

TEST_F(TestCalculator, should_start_with_zero_operaters)
{
  EXPECT_EQ(calculator.getOperators().size(), 0);
}

TEST_F(TestCalculator, should_allow_me_to_add_an_operator)
{
  ASSERT_NO_THROW(calculator.addOperator(std::make_shared<foo_op::FooOperator>()));

  ASSERT_EQ(calculator.getOperators().size(), 1);
  EXPECT_EQ(calculator.getOperators()[0], foo_op::OPERATION_NAME);
}

TEST_F(TestCalculator, should_allow_me_to_execute_an_added_operator)
{
  ASSERT_NO_THROW(calculator.addOperator(std::make_shared<foo_op::FooOperator>()));

  ASSERT_NO_THROW(calculator.evaluate(1, foo_op::OPERATION_NAME, 2));
}
}  // namespace calculator
}  // namespace cmake_template