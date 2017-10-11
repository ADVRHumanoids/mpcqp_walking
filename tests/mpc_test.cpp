#include <gtest/gtest.h>
#include <mpcqp_walking/model_preview_control.h>

namespace {

class testMPC: public ::testing::Test {
public:
    testMPC()
    {

    }

    virtual ~testMPC() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

TEST_F(testMPC, test_wpg)
{

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
