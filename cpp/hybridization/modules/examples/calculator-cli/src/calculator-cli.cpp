#include <cmake_template/calculator-cli/calculator-cli.h>

#include <iostream>
#include <algorithm>

namespace cmake_template
{
namespace calculator_cli
{
CalculatorCli::CalculatorCli(calculator::Calculator& c)
  : calculator_{ c }
  , first_number_{ 0 }
  , second_number_{ 0 }
  , operator_{ '\n' }
  , input_{ std::cin }
  , output_{ std::cout }
{
}

CalculatorCli::CalculatorCli(calculator::Calculator& c, std::istream& input, std::ostream& output)
  : calculator_{ c }
  , first_number_{ 0 }
  , second_number_{ 0 }
  , operator_{ '\n' }
  , input_{ input }
  , output_{ output }
{
}

double CalculatorCli::getFirstNumber()
{
  prompt("Enter First Number: ", first_number_);
  return first_number_;
}

char CalculatorCli::getOperator()
{
  prompt("Enter Operator: ", operator_);
  return operator_;
}

double CalculatorCli::getSecondNumber()
{
  prompt("Enter Second Number: ", second_number_);
  return second_number_;
}

void CalculatorCli::displayOperators()
{
  print("Available Operators:");
  for (auto& op : calculator_.getOperators())
  {
    print(std::string("  ") + op);
  }
}

bool CalculatorCli::keepCalculating()
{
  std::string ans;
  prompt("Keep Calculating? [Y/n/h] ", ans);

  std::transform(ans.begin(), ans.end(), ans.begin(), ::tolower);

  if (ans == "h" || ans == "help")
  {
    displayOperators();
    return true;
  }

  return ans == "y" || ans == "yes";
}

void CalculatorCli::print(const std::string& str)
{
  output_ << str << std::endl;
}

template <typename R>
void CalculatorCli::prompt(const std::string& str, R& res)
{
  output_ << str;
  input_ >> res;
}

double CalculatorCli::evaluate()
{
  return calculator_.evaluate(first_number_, operator_, second_number_);
}
}  // namespace calculator_cli
}  // namespace cmake_template