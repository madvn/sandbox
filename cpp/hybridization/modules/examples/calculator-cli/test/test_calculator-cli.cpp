#include <cmake_template/calculator-cli/calculator-cli.h>
#include <cmake_template/calculator/calculator.h>
#include <cmake_template/addition/addition.h>

#include <gtest/gtest.h>

#include <iostream>
#include <memory>

namespace cmake_template
{
namespace calculator_cli
{
static const std::string FIRST_NUM_TEXT = "Enter First Number: ";
static const std::string OPERATOR_TEXT = "Enter Operator: ";
static const std::string SECOND_NUM_TEXT = "Enter Second Number: ";
static const std::string OPERATORS_TEXT = "Available Operators:\n  +\n";
static const std::string KEEP_CALC_TEXT = "Keep Calculating? [Y/n/h] ";

class TestCalculatorCli : public ::testing::Test
{
protected:
  std::stringstream input;
  std::stringstream output;
  calculator::Calculator calculator;
  CalculatorCli calc{ calculator, input, output };

  void SetUp()
  {
    calculator.addOperator(std::make_shared<addition::Addition>());
  }
  void TearDown()
  {
  }
};

TEST_F(TestCalculatorCli, should_promp_for_the_first_number)
{
  calc.getFirstNumber();
  EXPECT_EQ(output.str(), FIRST_NUM_TEXT);
}

TEST_F(TestCalculatorCli, should_read_in_the_first_number)
{
  double first_num = 5;
  input << first_num;
  EXPECT_DOUBLE_EQ(calc.getFirstNumber(), first_num);
}

TEST_F(TestCalculatorCli, should_promp_for_the_operator)
{
  calc.getOperator();
  EXPECT_EQ(output.str(), OPERATOR_TEXT);
}

TEST_F(TestCalculatorCli, should_read_in_the_operator)
{
  char op = '+';
  input << op;
  EXPECT_EQ(calc.getOperator(), op);
}

TEST_F(TestCalculatorCli, should_promp_for_the_second_number)
{
  calc.getSecondNumber();
  EXPECT_EQ(output.str(), SECOND_NUM_TEXT);
}

TEST_F(TestCalculatorCli, should_read_in_the_second_number)
{
  double second_num = 5;
  input << second_num;
  EXPECT_DOUBLE_EQ(calc.getSecondNumber(), second_num);
}

TEST_F(TestCalculatorCli, should_evaluate_the_expression)
{
  double first_num = 4;
  char op = '+';
  double second_num = 5;
  input << first_num << op << second_num;
  calc.getFirstNumber();
  calc.getOperator();
  calc.getSecondNumber();
  EXPECT_DOUBLE_EQ(calc.evaluate(), first_num + second_num);
}

TEST_F(TestCalculatorCli, should_throw_an_exception_if_the_operator_is_not_registered)
{
  double first_num = 4;
  char op = '-';
  double second_num = 5;
  input << first_num << op << second_num;
  calc.getFirstNumber();
  calc.getOperator();
  calc.getSecondNumber();
  EXPECT_THROW(calc.evaluate(), calculator::Calculator::InvalidOperator);
}

TEST_F(TestCalculatorCli, should_display_operators)
{
  calc.displayOperators();
  EXPECT_EQ(output.str(), OPERATORS_TEXT);
}

TEST_F(TestCalculatorCli, keep_calculatiing_should_retun_true_if_Y_is_entered)
{
  input << "Y";
  EXPECT_TRUE(calc.keepCalculating());
}

TEST_F(TestCalculatorCli, keep_calculatiing_should_retun_true_if_y_is_entered)
{
  input << "y";
  EXPECT_TRUE(calc.keepCalculating());
}

TEST_F(TestCalculatorCli, keep_calculatiing_should_retun_true_if_yes_is_entered_ignoring_case)
{
  input << "YeS";
  EXPECT_TRUE(calc.keepCalculating());
}

TEST_F(TestCalculatorCli, keep_calculatiing_should_retun_true_and_display_operators_if_h_is_entered)
{
  input << "h";
  EXPECT_TRUE(calc.keepCalculating());
  EXPECT_EQ(output.str(), KEEP_CALC_TEXT + OPERATORS_TEXT);
}

TEST_F(TestCalculatorCli, keep_calculatiing_should_retun_true_and_display_operators_if_help_is_entered_ignoring_case)
{
  input << "hElP";
  EXPECT_TRUE(calc.keepCalculating());
  EXPECT_EQ(output.str(), KEEP_CALC_TEXT + OPERATORS_TEXT);
}

TEST_F(TestCalculatorCli, keep_calculatiing_should_retun_false_otherwise)
{
  input << "foo";
  EXPECT_FALSE(calc.keepCalculating());
}
}  // namespace calculator_cli
}  // namespace cmake_template