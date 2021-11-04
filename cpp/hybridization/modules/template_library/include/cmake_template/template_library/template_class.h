#ifndef __CMAKE_TEMPLATE_TEMPLATE_CLASS_H_INCLUDED__
#define __CMAKE_TEMPLATE_TEMPLATE_CLASS_H_INCLUDED__

// Include any headers from this module

// Next include any headers from other modules in this project

// Next include headers from other projects

// Lastly include system headers

namespace cmake_template  // project name
{
namespace template_library  // module name
{
class TemplateClass
{
public:
  TemplateClass(); // CamelCase class names

  bool doWork(); // pascalCase function names

private:
  bool delegateWork();

  int member_variable_; // snake_case_with_trailing_underscore_ variable names
};
}  // namespace template_library
}  // namespace cmake_template

#endif  //__CMAKE_TEMPLATE_TEMPLATE_CLASS_H_INCLUDED__
