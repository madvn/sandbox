// Always include the class header first
#include <cmake_template/template_library/template_class.h>

// Next include any headers from this module

// Next include any headers from other modules in this project

// Next include headers from other projects

// Lastly include system headers

namespace cmake_template  // project name
{
namespace template_library  // module name
{
static const double WELL_KNOWN_CONSTANT{ 3.141592 }; // SCREAMING_SNAKE_CASE for constants

//namespace baz = foo::bar::baz; // use namespace aliasing instead of the 'using' keyword

TemplateClass::TemplateClass()
  : member_variable_{ 1234 } // Initialize member variables in the constructor initializer list
{
}

bool TemplateClass::doWork()
{
  return delegateWork();
}

bool TemplateClass::delegateWork()
{
  bool snake_case_local_variable = true;
  return snake_case_local_variable;
}
}  // namespace template_library
}  // namespace cmake_template