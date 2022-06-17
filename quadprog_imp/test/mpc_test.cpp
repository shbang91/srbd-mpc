#include <gtest/gtest.h>

class MPCTest : public ::testing::Test {
    protected:

    MPCTest() {
    }

    ~MPCTest() override {
    }

    void SetUp() override {
    }

    void TearDown() override {
    }

};

TEST_F(MPCTest, testtest){
    ASSERT_TRUE(true);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}