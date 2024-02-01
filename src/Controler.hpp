#include <math.h>

int getDegreeDiff(int pos, int target) {
  int diff = target - pos;

  if (abs(diff) > 180) {
    diff = diff > 0 ? diff - 360 : diff + 360;
  }

  return diff;
}