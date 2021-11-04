#include <cmake_template/addition/addition.h>
#include <gtest/gtest.h>
#include <cstdlib>

namespace cmake_template
{
namespace addition
{
class TestAddition : public ::testing::Test
{
protected:
  Addition op;

  void SetUp()
  {
  }
  void TearDown()
  {
  }
};

TEST_F(TestAddition, should_add_two_numbers)
{
  EXPECT_EQ(op.evaluate(1, 2), 3);
}

TEST_F(TestAddition, should_add_two_arbitrary_numbers)
{
  double first = std::rand(), second = std::rand();
  std::cout << "First: " << first << std::endl;
  std::cout << "Second: " << second << std::endl;
  EXPECT_EQ(op.evaluate(first, second), first + second);
}
}  // namespace addition
}  // namespace cmake_template