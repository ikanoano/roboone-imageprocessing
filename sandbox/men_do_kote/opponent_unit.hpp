#ifndef OPPONENT_UNIT
#define OPPONENT_UNIT

#include "men_do_kote.hpp"

enum class OpponentBehavior {
  STABLE_FAR_OUT_OF_RANGE,
  STABLE_FAR_IN_RANGE,
  STABLE_NEAR,
  APPROACHING_VERTICAL,
  APPROACHING_HORIZONTAL,
  EVACUATING
};

class OpponentUnit {
public:
  OpponentUnit(int fps=30) : m(fps) { }

  struct OpponentModel {
    OpponentBehavior behavior;
    float position[3][3];
    float delta[3][3];  // Expected to be very unstable
  };

  void startCamera() {} // To be implemented
  void stopCamera() {}  // To be implemented
  void survey() {}      // To be implemented
private:
  Mikiri m;
};

#endif
