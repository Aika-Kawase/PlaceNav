#include "../src/placenav_teb_bridge/src/avoidance_guidance.h"
#include <cassert>
#include <limits>
using namespace avoidance_guidance;
int main() {
  std::vector<Point> seed{{0,0},{.5,.3},{1,.4},{2,.4}};
  auto out=blend(seed, {{0,0},{2,0}}, {0,0}, 1.2, 2);
  assert(out.size()>3);
  assert(distance(out[1],seed[1])<1e-9);
  assert(distance(out.back(),{2,0})<1e-9);
  assert(blend(seed, {{0,0},{.2,0}}, {0,0},1.2,2).empty());
  assert(blend(seed, {{0,0},{10,0}}, {0,0},1.2,2).empty());
  assert(blend({}, {{0,0},{2,0}}, {0,0},1.2,2).empty());
  assert(blend({{0,0},{std::numeric_limits<double>::quiet_NaN(),0}},
               {{0,0},{2,0}}, {0,0},1.2,2).empty());
  assert(blend({{0,0},{2,0}}, {{0,0},{1.5,0}}, {0,0},1.4,2).empty());
}
