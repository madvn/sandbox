// Always include the class header first
#include <cmake_template/calculator/calculator.h>

// Next include any headers from this module

// Next include any headers from other modules in this project

// Next include headers from other projects

// Lastly include system headers
#include <memory>
#include <vector>

namespace cmake_template
{
namespace calculator
{
Calculator::Calculator() = default;

void Calculator::addOperator(const std::shared_ptr<operator_base::Operator>& o)
{
  operators_.emplace(o->operation(), o);
}

double Calculator::evaluate(double lhs, const char& operation, double rhs)
{
  if (operators_.find(operation) == operators_.end())
  {
    throw calculator::Calculator::InvalidOperator(operation);
  }
  return operators_.at(operation)->evaluate(lhs, rhs);
}

std::vector<char> Calculator::getOperators()
{
  std::vector<char> operators;
  for (auto& op : operators_)
  {
    operators.emplace_back(op.first);
  }
  return operators;
}
}  // namespace calculator
}  // namespace cmake_template