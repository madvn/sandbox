#include <cmake_template/template_library/template_class.h>

#include <gtest/gtest.h>

namespace cmake_template
{
namespace template_library
{
class TestTemplateClass : public ::testing::Test
{
protected:
  TemplateClass template_class_;

  void SetUp()
  {
  }
  void TearDown()
  {
  }
};

TEST_F(TestTemplateClass, should_do_work)
{
  EXPECT_TRUE(template_class_.doWork());
}
}  // namespace template_library
}  // namespace cmake_template