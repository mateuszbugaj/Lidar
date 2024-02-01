#include <gtest/gtest.h>
#include <Controler.hpp>

TEST(ControlerTest, getDegreeDiffPositive){
  int pos = 60;
  int target = 80;
  int expectedDifference = 20;

  EXPECT_EQ(getDegreeDiff(pos, target), expectedDifference);
}

TEST(ControlerTest, getDegreeDiffPositive_2){
  int pos = 340;
  int target = 20;
  int expectedDifference = 40;

  EXPECT_EQ(getDegreeDiff(pos, target), expectedDifference);
}

TEST(ControlerTest, getDegreeDiffNegative){
  int pos = 80;
  int target = 60;
  int expectedDifference = -20;

  EXPECT_EQ(getDegreeDiff(pos, target), expectedDifference);
}

TEST(ControlerTest, getDegreeDiffNegative_2){
  int pos = 0;
  int target = 200;
  int expectedDifference = -160;

  EXPECT_EQ(getDegreeDiff(pos, target), expectedDifference);
}

TEST(ControlerTest, getDegreeDiffNegative_3){
  int pos = 20;
  int target = 220;
  int expectedDifference = -160;

  EXPECT_EQ(getDegreeDiff(pos, target), expectedDifference);
}

TEST(ControlerTest, getDegreeDiffHalf_1){
  int pos = 0;
  int target = 180;
  int expectedDifference = 180;

  EXPECT_EQ(getDegreeDiff(pos, target), expectedDifference);
}

TEST(ControlerTest, getDegreeDiffHalf_2){
  int pos = 20;
  int target = 200;
  int expectedDifference = 180;

  EXPECT_EQ(getDegreeDiff(pos, target), expectedDifference);
}