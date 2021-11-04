#ifndef CMAKE_TEMPLATE_FOO_OPERATOR_H
#define CMAKE_TEMPLATE_FOO_OPERATOR_H

#include <cmake_template/operator/operator.h>

namespace cmake_template
{
namespace foo_op
{
const static char OPERATION_NAME = 'f';

class FooOperator : public operator_base::Operator
{
public:
  FooOperator() = default;

  char operation() const
  {
    return OPERATION_NAME;
  }

  double evaluate(double lhs, double rhs)
  {
    return 0;
  }
};
}  // namespace foo_op
}  // namespace cmake_template

#endif  //CMAKE_TEMPLATE_FOO_OPERATOR_H
