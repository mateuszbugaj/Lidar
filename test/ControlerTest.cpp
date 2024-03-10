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

TEST(ControllerTest, generatePoints_corners) {
  int width = 70;
  int height = 30;
  int resolution = 10;
  int initialAlpha = 33;
  int initialBeta = 58;

  Point point{initialAlpha, initialBeta};
  int counter = 0;

  Point prevPoint;
  prevPoint.alphaDegrees = point.alphaDegrees;
  prevPoint.betaDegrees = point.betaDegrees;

  std::vector<int> expectedAlphaDiff = {
    -30, 10, 10, 10, 10, 10, 10,
    -60, 10, 10, 10, 10, 10, 10,
    -60, 10, 10, 10, 10, 10, 10
  };

  std::vector<int> expectedBetaDiff = {
    10, 0, 0, 0, 0, 0, 0,
    -10, 0, 0, 0, 0, 0, 0,
    -10, 0, 0, 0, 0, 0, 0
  };

  for (int i = 0; i < 21; i++) {
    Point prevPoint;

    prevPoint.alphaDegrees = point.alphaDegrees;
    prevPoint.betaDegrees = point.betaDegrees;

    generatePoint(i, point, width, height, resolution);

    EXPECT_EQ(point.alphaDegrees - prevPoint.alphaDegrees, expectedAlphaDiff[i]);
    EXPECT_EQ(point.betaDegrees - prevPoint.betaDegrees, expectedBetaDiff[i]);
    counter++;
  }

}

