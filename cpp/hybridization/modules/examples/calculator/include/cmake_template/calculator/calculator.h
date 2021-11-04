#ifndef CMAKE_TEMPLATE_OPERATOR_H
#define CMAKE_TEMPLATE_CALCULATOR_H

// Include any headers from this module

// Next include any headers from other modules in this project
#include <cmake_template/operator/operator.h>

// Next include headers from other projects

// Lastly include system headers
#include <vector>
#include <map>
#include <string>
#include <memory>
#include <exception>
#include <iostream>

namespace cmake_template
{
namespace calculator
{
class Calculator
{
private:
  std::map<char, std::shared_ptr<operator_base::Operator>> operators_;

public:
  Calculator();

  std::vector<char> getOperators();

  void addOperator(const std::shared_ptr<operator_base::Operator>& o);

  double evaluate(double lhs, const char& shorthand, double rhs);

  class InvalidOperator : public std::exception
  {
  private:
    std::string what_msg_;

  public:
    explicit InvalidOperator(const char op)
      : std::exception()
      , what_msg_{ "Invalid operator: " }
    {
      what_msg_ += op;
    }

    const char* what() const noexcept
    {
      return what_msg_.c_str();
    }
  };
};
}  // namespace calculator
}  // namespace cmake_template

#endif  //CMAKE_TEMPLATE_OPERATOR_H
