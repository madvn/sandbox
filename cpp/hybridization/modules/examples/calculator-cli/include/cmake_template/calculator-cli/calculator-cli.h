#ifndef CMAKE_TEMPLATE_CALCULATOR_CLI_H
#define CMAKE_TEMPLATE_CALCULATOR_CLI_H

#include <cmake_template/calculator/calculator.h>

#include <iostream>
#include <string>

namespace cmake_template
{
namespace calculator_cli
{
class CalculatorCli
{
public:
  CalculatorCli(calculator::Calculator& c);

  CalculatorCli(calculator::Calculator& c, std::istream& input, std::ostream& output);

  double getFirstNumber();

  char getOperator();

  double getSecondNumber();

  void displayOperators();

  bool keepCalculating();

  void print(const std::string& str);

  template <typename R>
  void prompt(const std::string& str, R& res);

  double evaluate();

private:
  calculator::Calculator& calculator_;

  double first_number_, second_number_;
  char operator_;

  std::istream& input_;
  std::ostream& output_;
};
}  // namespace calculator_cli
}  // namespace cmake_template

#endif  //CMAKE_TEMPLATE_CALCULATOR_CLI_H
