#include <cmake_template/calculator/calculator.h>
#include <cmake_template/addition/addition.h>

#include <gtest/gtest.h>

#include <memory>

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

TEST_F(TestCalculator, should_support_the_addition_operator)
{
  EXPECT_NO_THROW(calculator.addOperator(std::make_shared<addition::Addition>()));
}

TEST_F(TestCalculator, should_do_addition)
{
  calculator.addOperator(std::make_shared<addition::Addition>());

  EXPECT_DOUBLE_EQ(calculator.evaluate(1, '+', 2), 3);
}
}  // namespace calculator
}  // namespace cmake_template