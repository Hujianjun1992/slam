#include "utilities.h"

namespace line_extraction
{
double pi_to_pi(double angle)
{
  angle = fmod(angle , 2 * M_PI);
  if (angle >= M_PI) {
    angle -= 2 * M_PI;
    return angle;
  }
}
}
