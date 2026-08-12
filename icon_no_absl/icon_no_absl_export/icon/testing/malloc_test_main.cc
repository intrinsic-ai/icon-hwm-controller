#include <gtest/gtest.h>

// TODO(b/542544362): Add implementation once MallocGuard is available via
// BCR
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  return result;
}
