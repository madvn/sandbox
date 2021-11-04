#include <cmake_template/template_library/template_class.h>

#include <gtest/gtest.h>

namespace cmake_template
{
class TestTemplateClass : public ::testing::Test
{
protected:
  template_library::TemplateClass template_class_;

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
}  // namespace cmake_template