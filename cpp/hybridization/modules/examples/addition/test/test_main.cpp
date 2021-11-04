#include <gtest/gtest.h>
#include <cstdlib>

int main(int argc, char** argv)
{
  std::srand(std::time(NULL));
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
