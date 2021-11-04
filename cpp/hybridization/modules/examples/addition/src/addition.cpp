// Always include the class header first
#include <cmake_template/addition/addition.h>

// Next include any headers from this module

// Next include any headers from other modules in this project

// Next include headers from other projects

// Lastly include system headers
#include <string>

namespace cmake_template
{
namespace addition
{
Addition::Addition() = default;

char Addition::operation() const
{
  return OPERATION_NAME;
}

double Addition::evaluate(double lhs, double rhs)
{
  return lhs + rhs;
}
}  // namespace addition
}  // namespace cmake_template