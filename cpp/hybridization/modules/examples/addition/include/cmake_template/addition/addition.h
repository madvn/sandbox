#ifndef CMAKE_TEMPLATE_ADDITION_H
#define CMAKE_TEMPLATE_ADDITION_H

// Include any headers from this module

// Next include any headers from other modules in this project
#include <cmake_template/operator/operator.h>

// Next include headers from other projects

// Lastly include system headers
#include <string>

namespace cmake_template
{
namespace addition
{
const static char OPERATION_NAME = '+';

class Addition : public operator_base::Operator
{
public:
  Addition();

  char operation() const override;

  double evaluate(double lhs, double rhs) override;

private:
};
}  // namespace addition
}  // namespace cmake_template

#endif  //CMAKE_TEMPLATE_ADDITION_H
