#ifndef OPPONENT_UNIT
#define OPPONENT_UNIT

#include <deque>
#include <chrono>
#include <iostream>
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
  typedef std::chrono::system_clock::time_point time_stamp_t;
  struct OpponentPart {
    std::array<float, 3>  last;
    time_stamp_t          last_at;
    std::array<float, 3>  pred;
    time_stamp_t          pred_at;

    inline void print(const char* pre) const {
      std::cout
        << pre
        << " last @ "
        << std::chrono::system_clock::to_time_t(last_at)
        << " = (" << last[0] << ", " << last[1] << ", " << last[2] << ")"
        << "\tpred @ "
        << std::chrono::system_clock::to_time_t(pred_at)
        << " = (" << pred[0] << ", " << pred[1] << ", " << pred[2] << ")"
        << std::endl;
    }

    template<typename T>
    inline T last_xyz() const { return T{.x=last[0], .y=last[1], .z=last[2]}; }
    template<typename T>
    inline T pred_xyz() const { return T{.x=pred[0], .y=pred[1], .z=pred[2]}; }
  };
  struct OpponentModel {
    OpponentBehavior              behavior;
    boost::optional<OpponentPart> men;
    boost::optional<OpponentPart> dou;
    boost::optional<OpponentPart> kote;
  };

  void startCamera();
  void stopCamera();
  boost::optional<OpponentModel> survey(std::chrono::milliseconds predict_after);
private:
#ifdef USE_CAMERA
  Mikiri    m;
#else
  Mikagiri  m;
#endif
  std::deque<Mikagiri::men_do_kote_t>  hist_mdk;
};

#endif
