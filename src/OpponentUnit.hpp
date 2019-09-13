#ifndef OPPONENT_UNIT
#define OPPONENT_UNIT

#include <boost/optional.hpp>
#include "Mikagiri.hpp"
#ifdef USE_CAMERA
  #include "Mikiri.hpp"
#endif

class OpponentUnit {
public:
  OpponentUnit(int fps=30, bool visualize=true) : m(fps,visualize) {}

  enum class OpponentBehavior {
    STABLE_FAR_OUT_OF_RANGE,
    STABLE_FAR_IN_RANGE,
    STABLE_NEAR,
    APPROACHING_VERTICAL,
    APPROACHING_HORIZONTAL,
    EVACUATING
  };
  typedef boost::optional<std::array<float, 3>> OpponentPart;
  struct OpponentModel {
    OpponentBehavior  behavior;
    OpponentPart      men;
    OpponentPart      dou;
    OpponentPart      kote;
  };

  void startCamera();
  void stopCamera();
  boost::optional<OpponentModel> survey();
private:
#ifdef USE_CAMERA
  Mikiri    m;
#else
  Mikagiri  m;
#endif
};

#endif
