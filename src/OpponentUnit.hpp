#ifndef OPPONENT_UNIT
#define OPPONENT_UNIT

#include <optional>
#include "Mikiri.hpp"

class OpponentUnit {
public:
  OpponentUnit(int fps=30);

  typedef std::optional<std::array<float, 3>> OpponentPart;
  struct OpponentModel {
    OpponentPart men;
    OpponentPart dou;
    OpponentPart kote;
  };

  void startCamera();
  void stopCamera();
  OpponentModel survey();
private:
  const char* visual_window = "MEN(red) DO(blue) KOTE(green) Visualizer";
  Mikiri m;
};

#endif
