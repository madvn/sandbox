#ifndef CMAKE_TEMPLATE_OPERATOR_H
#define CMAKE_TEMPLATE_OPERATOR_H

// Include any headers from this module

// Next include any headers from other modules in this project

// Next include headers from other projects

// Lastly include system headers
#include <string>

namespace cmake_template
{
namespace operator_base
{
class Operator
{
public:
  virtual char operation() const = 0;

  virtual double evaluate(double lhs, double rhs) = 0;
};
}  // namespace operator_base
}  // namespace cmake_template

#endif  //CMAKE_TEMPLATE_OPERATOR_H
