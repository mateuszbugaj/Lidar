#include <math.h>
#include <iostream>

struct Point {
	int16_t alphaDegrees;
	int16_t betaDegrees;
};

std::pair<bool, bool> generatePoint(int counter, Point& point, int width, int height, int resolution) {
    int numPointsX = width / resolution;
    int numPointsY = height / resolution;
    int totalPoints = numPointsX * numPointsY;

    if (counter >= totalPoints) {
        return {false, false};
    }

    int alphaDiff = 0;
    int betaDiff = 0;

    bool newRow = false;
    if(counter == 0){
      alphaDiff = -numPointsX/2*resolution;
      betaDiff = numPointsY/2*resolution;
    } else if(counter % numPointsX == 0){
      alphaDiff = -(numPointsX-1)*resolution;
      betaDiff = -resolution;
      newRow = true;
    } else {
      alphaDiff = resolution;
    }

    point.alphaDegrees+=alphaDiff;
    point.betaDegrees+=betaDiff;

    return {true, newRow};
}


int getDegreeDiff(int pos, int target) {
  int diff = target - pos;

  if (abs(diff) > 180) {
    diff = diff > 0 ? diff - 360 : diff + 360;
  }

  return diff;
}